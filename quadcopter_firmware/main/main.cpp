

#include <DebugHelper.h>
#include <QuadcopterController.h>
#include <Wire.h>

#include <string>

// Magnetometer
#include <Fusion.h>
#include <QMC5883L.h>
#include <esp_partition.h>

#include "AsyncController.h"
#include "BLEController.h"
#include "Filters/KalmanFilter.h"
#include "Filters/MedianFilter.h"
#include "IMU.h"
#include "LEDController.h"
#include "Logger.h"
#include "MotorController.h"
#include "PersistentKeyValueStore.h"
#include "PersistentKeysCommon.h"
#include "SPI.h"
#include "TelemetryController.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

static std::vector<MotorController *> _speedControllers;

// LED Setup
#define LED_DATA_PIN GPIO_NUM_38

static LEDController _ledController(LED_DATA_PIN);

const gpio_num_t motorPins[NUM_MOTORS] = {GPIO_NUM_6, GPIO_NUM_5, GPIO_NUM_7, GPIO_NUM_8};

struct _memory_usage_telemetry_packet_t
{
  uint32_t freeHeap;
  uint32_t minFreeHeap;
  uint32_t totalHeapSize;
};

#define TELEM_UPDATE_INTERVAL_MILLIS 100
static TelemetryController *_telemetryController;

// Battery capacity sensing constants
#define BATTERY_SENSE_PIN 9
#define NUM_BATTERY_CELLS 1.0f
#define BATTERY_CELL_MAX_VOLTAGE 4.2f
#define BATTERY_CELL_MIN_VOLTAGE 3.3f
#define BATTERY_SCALE 0.001639280125196f

#define MIN_THROTTLE 1015

#define ARM_MICROSECONDS 1000.0f

// micro whoop
#define SPI0_CS_PIN 10
#define SPI0_MOSI_PIN 11
#define SPI0_SCLK_PIN 12
#define SPI0_MISO_PIN 13
#define IMU_INT_PIN 3

BLEController _bluetoothController;

static bool _armed = false;
static bool _previousArmStatus = false;

motor_outputs_t _previousMotorOutputs;
static bool _setMotorOutputs = false;

static bool _resetFlag = false;

static bool _motorDebugEnabled = false;
float _motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

static bool _startMonitoringPid = false;

static bool _sendDebugData = false;
static bool _recordDebugData = false;

static bool _completedFirstArm = false;

// When the quadcopter enters into unacceptable orientation for more than 2
// measurements, the drone will cut power to the motors for safety and must be
// rebooted
static bool _enteredEmergencyMode = false;

static QMC5883L _compass;

static mag_update_t _magValues;
static bool _receivedMagUpdate = false;

PersistentKeyValueStore _persistentKvStore;

static IMU *_imu = NULL;
static imu_output_t _imuValues;
static FusionAhrs _fusion;

static volatile bool _calibrate = false;

// When we start recording debug data, we want the LED to flash blue
// for RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS. This lets us align
// any video recordings with the debug data that we're recording
#define RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS 200
static uint64_t _startedRecordingDebugDataTimeMillis = 0;
static bool _recordDebugDataLEDStateChanged = false;

// We only have 4 RMT channels on the ESP32 and so in order to use 4 motors
// and 1 LED we need to share the channel
static bool _waitingForLED = false;
static void _updateLED(int red, int green, int blue)
{
  if (_waitingForLED) {
    LOG_WARN("Already waiting for LED transition to occur");
    return;
  }
  LOG_INFO("Showing color: %i, %i, %i", red, green, blue);
  if (_speedControllers.size() > 0 && _speedControllers[0]->isConnected) {
    _speedControllers[0]->disconnectRMT();
  } else if (_speedControllers.size() > 0) {
    LOG_WARN("RMT multiplexing issue: Speed controller was already disconnected");
  }
  // Calling showRGB initializes the RMT channel
  _ledController.showRGB(red, green, blue);

  // Give the LED a bit of time to display the color
  _waitingForLED = true;
  AsyncController::main.executeAfter(10, []() {
    _ledController.disconnectRMT();
    if (_speedControllers.size() > 0) {
      _speedControllers[0]->connectRMT();
    }
    _waitingForLED = false;
  });
}

static void _updateArmStatus(void)
{
  if (_armed && !_completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    for (int i = 0; i < NUM_MOTORS; i++) {
      LOG_INFO("Attaching motor %i to pin %i", i, motorPins[i]);
      MotorController *controller = new MotorController(motorPins[i]);
      _speedControllers.push_back(controller);
    }
    _completedFirstArm = true;
  }
  _telemetryController->updateTelemetryEvent(TelemetryEvent::ArmStatusChange, &_armed, sizeof(bool));
  if (!_enteredEmergencyMode) {
    _updateLED(_armed ? 255 : 0, _armed ? 0 : 255, 0);
  }
}

static void _configureMagnetometer(void)
{
  ESP_ERROR_CHECK(_compass.begin(Wire, [&](mag_update_t update) {
    _receivedMagUpdate = true;
    _magValues = update;
  }));

  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on:
   * http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  const float declinationAngle = (12.0 + (55.0 / 60.0)) / (180 / PI);
  _compass.setRange(QMC5883L::Range::GAUSS_2);
  _compass.setMeasurementMode(QMC5883L::MeasurementMode::CONTINUOUS);
  _compass.setDataRate(QMC5883L::DataRate::HZ200);
  _compass.setSamples(QMC5883L::Samples::OSR64);
  _compass.setDeclinationAngle(declinationAngle);

  if (_persistentKvStore.hasValueForKey(PersistentKeysCommon::MAG_OFFSET_X)) {
    _compass.setXOffset(_persistentKvStore.getFloatForKey(PersistentKeysCommon::MAG_OFFSET_X));
  } else {
    LOG_WARN("magnetometer: No X offset found in persistent storage, please calibrate the compass");
  }

  if (_persistentKvStore.hasValueForKey(PersistentKeysCommon::MAG_OFFSET_Y)) {
    _compass.setYOffset(_persistentKvStore.getFloatForKey(PersistentKeysCommon::MAG_OFFSET_Y));
  } else {
    LOG_WARN("magnetometer: No Y offset found in persistent storage, please calibrate the compass");
  }

  if (_persistentKvStore.hasValueForKey(PersistentKeysCommon::MAG_OFFSET_Z)) {
    _compass.setZOffset(_persistentKvStore.getFloatForKey(PersistentKeysCommon::MAG_OFFSET_Z));
  } else {
    LOG_WARN("magnetometer: No Z offset found in persistent storage, please calibrate the compass");
  }
}

static QuadcopterController *_controller;
static DebugHelper *_helper;

static void initController()
{
  delete _controller;
  _controller = new QuadcopterController(
      {
          .angleGains =
              {{// yaw
                .kP = 9.0f,
                .kI = 0.01f,
                .kD = 0.0f},
                           {// pitch
                .kP = 6.0f,
                .kI = 0.0075f,
                .kD = 0.1f},
                           {// roll
                .kP = 6.0f,
                .kI = 0.0075f,
                .kD = 0.1f}  },
          .rateGains =
              {{// yaw
                .kP = 3.1f,
                .kI = 0.01f,
                .kD = 0.0005f},
                           {// pitch
                .kP = 6.0f,
                .kI = 0.02f,
                .kD = 0.003f},
                           {// roll
                .kP = 6.0f,
                .kI = 0.02f,
                .kD = 0.003f}}
  },
      _helper,
      micros());
}

static void _handleCalibration(CalibrationType type, CalibrationResponse response)
{
  switch (type) {
    case CalibrationType::Gyro: {
      calibration_data_t dat = _imu->calibrate__deprecated();
      if (dat.success) {
        LOG_INFO("Calibration successful");
        _bluetoothController.sendCalibrationData(dat);
      } else {
        LOG_ERROR("Calibration failed");
      }
      break;
    }
    case CalibrationType::Accelerometer: {
      std::map<CalibrationResponse, std::function<void(void)>> handlers = _imu->calibrationHandlers(
          [](CalibrationRequest request) { _bluetoothController.sendCalibrationUpdate(request); });
      LOG_INFO("Handling calibration response %d", response);
      handlers[response]();
      break;
    }
    case CalibrationType::Magnetometer: {
      LOG_INFO("Calibrating magnetometer");
      mag_calibration_offsets_t calibrationData = _compass.calibrate();
      ESP_ERROR_CHECK(calibrationData.error);
      LOG_INFO(
          "Successfully calibrated compass: %f, %f, %f",
          calibrationData.x_offset,
          calibrationData.y_offset,
          calibrationData.z_offset);
      _persistentKvStore.setFloatForKey(PersistentKeysCommon::MAG_OFFSET_X, calibrationData.x_offset);
      _persistentKvStore.setFloatForKey(PersistentKeysCommon::MAG_OFFSET_Y, calibrationData.y_offset);
      _persistentKvStore.setFloatForKey(PersistentKeysCommon::MAG_OFFSET_Z, calibrationData.z_offset);
      break;
    }
  }
}

static controller_values_t _controllerValues = {
    .leftStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = 0.0f                            },
    .rightStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = INPUT_MAX_CONTROLLER_INPUT / 2.0}
};

static FusionEuler _euler;
static uint64_t _previousMicros = 0;
static const FusionMatrix gyroscopeMisalignment = {
    0.7071f,
    0.7071f,
    0.0f,  // First row
    0.7071f,
    -0.7071f,
    0.0f,  // Second row
    0.0f,
    0.0f,
    1.0f  // Third row
};
static const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
static const FusionMatrix accelerometerMisalignment = {
    0.7071f,
    0.7071f,
    0.0f,  // First row
    0.7071f,
    -0.7071f,
    0.0f,  // Second row
    0.0f,
    0.0f,
    1.0f  // Third row
};
static const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
static bool _receivedImuUpdate = false;
static bool _gotFirstIMUUpdate = false;

static float _previousFilteredAccelValues[3] = {0.0f, 0.0f, 1.0f};
static float _previousFilteredGyroValues[3] = {0.0f, 0.0f, 0.0f};

static uint64_t _imuUpdateCounter = 0;

static const float _accelerometerLowPassAlpha = 0.075f;  // lower alpha = more smoothing but more lag
static const float _gyroscopeLowPassAlpha = 0.1f;        // lower alpha = more smoothing but more lag

#define MEDIAN_FILTER_WINDOW 20
static std::array<MedianFilter<float>, 3> _gyroHistories = {
    MedianFilter<float>(MEDIAN_FILTER_WINDOW),
    MedianFilter<float>(MEDIAN_FILTER_WINDOW),
    MedianFilter<float>(MEDIAN_FILTER_WINDOW)};

static std::array<KalmanFilter<float>, 3> _accelKalmanFilters = {
    KalmanFilter<float>(0.01f, 1.0f, 0.0f, 1.0f, 0.0f),
    KalmanFilter<float>(0.01f, 1.0f, 0.0f, 1.0f, 0.0f),
    KalmanFilter<float>(0.01f, 1.0f, 0.0f, 1.0f, 0.0f)};

static std::array<KalmanFilter<float>, 3> _gyroKalmanFilters = {
    KalmanFilter<float>(0.01f, 1.5f, 0.0f, 1.0f, 0.0f),
    KalmanFilter<float>(0.01f, 1.5f, 0.0f, 1.0f, 0.0f),
    KalmanFilter<float>(0.01f, 1.5f, 0.0f, 1.0f, 0.0f)};

static void _receivedIMUUpdate(imu_update_t update)
{
  const float deltaTimeSeconds = (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();

  FusionVector gyroscope = {
      update.gyro_x,
      update.gyro_y,
      update.gyro_z};  // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {update.accel_x, update.accel_y, update.accel_z};

  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(
      accelerometer,
      accelerometerMisalignment,
      accelerometerSensitivity,
      accelerometerOffset);

  _gyroHistories[0].addValue(gyroscope.axis.x);
  gyroscope.axis.x = _gyroHistories[0].getMedian();
  _gyroHistories[1].addValue(gyroscope.axis.y);
  gyroscope.axis.y = _gyroHistories[1].getMedian();
  _gyroHistories[2].addValue(gyroscope.axis.z);
  gyroscope.axis.z = _gyroHistories[2].getMedian();

  accelerometer.axis.x = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[0]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.x);
  accelerometer.axis.y = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[1]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.y);
  accelerometer.axis.z = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[2]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.z);
  gyroscope.axis.x =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[0]) + (_gyroscopeLowPassAlpha * gyroscope.axis.x);
  gyroscope.axis.y =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[1]) + (_gyroscopeLowPassAlpha * gyroscope.axis.y);
  gyroscope.axis.z =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[2]) + (_gyroscopeLowPassAlpha * gyroscope.axis.z);

  _previousFilteredGyroValues[0] = gyroscope.axis.x;
  _previousFilteredGyroValues[1] = gyroscope.axis.y;
  _previousFilteredGyroValues[2] = gyroscope.axis.z;
  _previousFilteredAccelValues[0] = accelerometer.axis.x;
  _previousFilteredAccelValues[1] = accelerometer.axis.y;
  _previousFilteredAccelValues[2] = accelerometer.axis.z;

  // Apply Kalman filter to accelerometer data
  accelerometer.axis.x = _accelKalmanFilters[0].applyFilter(accelerometer.axis.x);
  accelerometer.axis.y = _accelKalmanFilters[1].applyFilter(accelerometer.axis.y);
  accelerometer.axis.z = _accelKalmanFilters[2].applyFilter(accelerometer.axis.z);

  gyroscope.axis.x = _gyroKalmanFilters[0].applyFilter(gyroscope.axis.x);
  gyroscope.axis.y = _gyroKalmanFilters[1].applyFilter(gyroscope.axis.y);
  gyroscope.axis.z = _gyroKalmanFilters[2].applyFilter(gyroscope.axis.z);

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateNoMagnetometer(&_fusion, gyroscope, accelerometer, deltaTimeSeconds);
  _euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&_fusion));

  _imuValues = {
      .gyroOutput = {gyroscope.axis.x, gyroscope.axis.y,   gyroscope.axis.z },
      .yawPitchRollDegrees = {_euler.angle.yaw, _euler.angle.pitch, _euler.angle.roll}
  };

  _receivedImuUpdate = true;
  _gotFirstIMUUpdate = true;

  _helper->accelFiltered[0] = accelerometer.axis.x;
  _helper->accelFiltered[1] = accelerometer.axis.y;
  _helper->accelFiltered[2] = accelerometer.axis.z;
  _helper->accelRaw[0] = update.accel_x;
  _helper->accelRaw[1] = update.accel_y;
  _helper->accelRaw[2] = update.accel_z;
  _helper->gyroFiltered[0] = gyroscope.axis.x;
  _helper->gyroFiltered[1] = gyroscope.axis.y;
  _helper->gyroFiltered[2] = gyroscope.axis.z;
  _helper->gyroRaw[0] = update.gyro_x;
  _helper->gyroRaw[1] = update.gyro_y;
  _helper->gyroRaw[2] = update.gyro_z;
  _helper->ypr[0] = _euler.angle.yaw;
  _helper->ypr[1] = _euler.angle.pitch;
  _helper->ypr[2] = _euler.angle.roll;
  _helper->magHeading = _magValues.heading;

  LOG_INFO_PERIODIC_MILLIS(
      100,  // log at most up to every 100 millis
      "%8ld, %8.2f, %8.2f, %8.2f, %8.2f, %6d, %6d, %6d, %8.2f, %8.2f, %8.2f, %8.2f, %8.2f, %8.2f, %6d, %6d, %6d, "
      "%8.2f, %f, %f, %f",
      millis(),
      deltaTimeSeconds,
      _euler.angle.yaw,
      _euler.angle.pitch,
      _euler.angle.roll,
      update.accel_raw_x,
      update.accel_raw_y,
      update.accel_raw_z,
      update.accel_x,
      update.accel_y,
      update.accel_z,
      update.gyro_x,
      update.gyro_y,
      update.gyro_z,
      (int)update.gyro_raw_x,
      (int)update.gyro_raw_y,
      (int)update.gyro_raw_z,
      _magValues.heading,
      update.gyro_dps_x,
      update.gyro_dps_y,
      update.gyro_dps_z);

  _imuUpdateCounter++;
  EXECUTE_PERIODIC(1000, {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::IMUUpdateRate, &_imuUpdateCounter, sizeof(uint64_t));
    _imuUpdateCounter = 0;
  });
}

// Initial setup
void setup()
{
  esp_log_level_set("rmt", ESP_LOG_DEBUG);
  const unsigned long initializationTime = millis();
  Serial.begin(115200);
  LOG_INFO("BEGAN APP");

  _helper = new DebugHelper();

  initController();

  Wire.setClock(400000);
  Wire.begin(18, 17);

  // Wait for serial to become available
  while (!Serial);

  LOG_INFO("Initializing bluetooth connection");

  _bluetoothController.setControlsUpdateHandler([&](controls_update_t update) {
    _controllerValues = {
        .leftStickInput = {.x = update.yaw,   .y = update.throttle},
        .rightStickInput = {.x = update.pitch, .y = update.roll    }
    };
  });

  _bluetoothController.setArmStatusUpdateHandler([&](bool armStatus) { _armed = armStatus; });

  _bluetoothController.setResetStatusUpdateHandler([&]() { _resetFlag = true; });

  _bluetoothController.setMotorDebugEnabledUpdateHandler([&](bool motorDebug) { _motorDebugEnabled = motorDebug; });

  _bluetoothController.setMotorDebugUpdateHandler([&](motor_debug_update_t motorDebugUpdate) {
    _motorDebugValues[motorDebugUpdate.motorNum] = motorDebugUpdate.motorWriteValue;
  });

  _bluetoothController.setCalibrationUpdateHandler([&](CalibrationType type, CalibrationResponse response) {
    AsyncController::main.executePossiblySync([type, response]() {
      _handleCalibration(type, response);
      LOG_INFO("Calibration update received: type = %d, response = %d", type, response);
    });
  });

  _bluetoothController.setDebugDataUpdateHandler([&](debug_recording_update_t debugDataUpdate) {
    AsyncController::main.executePossiblySync([debugDataUpdate]() {
      LOG_INFO(
          "Received debug data update: sendDebugData = %d, recordDebugData = %d",
          debugDataUpdate.sendDebugData,
          debugDataUpdate.recordDebugData);
    });
    _sendDebugData = debugDataUpdate.sendDebugData;
    _recordDebugData = debugDataUpdate.recordDebugData;
    _startedRecordingDebugDataTimeMillis = millis();
  });

  // Setup BLE Server
  _bluetoothController.beginBluetooth();

  LOG_INFO("Initializing telemetry controller");
  _telemetryController = new TelemetryController(&_bluetoothController);

  LOG_INFO("Configuring magnetometer");

  _configureMagnetometer();

  LOG_INFO("Successfully initialized magnetometer");

  _updateArmStatus();

  LOG_INFO("Initializing attitude & heading reference system");
  // FusionOffsetInitialise(&_offset, 4000);
  FusionAhrsInitialise(&_fusion);

  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.5f,
      .accelerationRejection = 10.0f,
      .recoveryTriggerPeriod = 5000, /* 5 seconds */
  };
  FusionAhrsSetSettings(&_fusion, &settings);

  LOG_INFO("Initializing IMU");
  bool setupSuccess = false;
  _imu = new IMU(
      SPI0_CS_PIN,
      SPI0_MISO_PIN,
      SPI0_MOSI_PIN,
      SPI0_SCLK_PIN,
      IMU_INT_PIN,
      [](imu_update_t update) { _receivedIMUUpdate(update); },
      &_persistentKvStore,
      &setupSuccess);

  if (!setupSuccess) {
    LOG_ERROR("IMU Setup Failure");
  } else {
    LOG_INFO("Successfully set up IMU");
  }

  while (!setupSuccess) {
  };

  LOG_INFO("Initializing Arming Signal LED");
  _updateLED(0, 255, 0);

  LOG_INFO("SETUP COMPLETE AFTER %lu ms", millis() - initializationTime);
}

static void uploadDebugData()
{
  DebugDataManager manager = _helper->dataManager;
  uint8_t *data = manager.data;
  uint64_t dataSize = manager.numSamples * DEBUG_PACKET_SIZE;
  _bluetoothController.uploadDebugData(data, dataSize);
}

static float batteryPercent(float batteryVoltage)
{
  const float averageVoltagePerCell = batteryVoltage / (float)NUM_BATTERY_CELLS;
  const float cellRange = BATTERY_CELL_MAX_VOLTAGE - BATTERY_CELL_MIN_VOLTAGE;
  return (averageVoltagePerCell - BATTERY_CELL_MIN_VOLTAGE) / cellRange;
}

static float batteryLevel()
{
  const int val = analogRead(BATTERY_SENSE_PIN);
  return BATTERY_SCALE * (float)val;
}

static void sendTelemData(float batVoltage)
{
  _telemetryController->updateTelemetryEvent(
      TelemetryEvent::MotorValues,
      _previousMotorOutputs.data(),
      NUM_MOTORS * sizeof(float));

  _telemetryController->updateTelemetryEvent(TelemetryEvent::BatteryVoltage, &batVoltage, sizeof(float));

  if (_gotFirstIMUUpdate) {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::EulerYawPitchRoll, &_euler.angle, sizeof(float) * 3);
  }

  multi_heap_info_t info;
  heap_caps_get_info(&info, MALLOC_CAP_8BIT);
  _memory_usage_telemetry_packet_t memoryStats = {
      .freeHeap = info.total_free_bytes,
      .minFreeHeap = info.minimum_free_bytes,
      .totalHeapSize = heap_caps_get_total_size(MALLOC_CAP_8BIT),
  };
  _telemetryController->updateTelemetryEvent(
      TelemetryEvent::MemoryStats,
      &memoryStats,
      sizeof(_memory_usage_telemetry_packet_t));
}

static void updateClientTelemetryIfNeeded(float batVoltage)
{
  if (!_bluetoothController.isConnected) {
    return;
  }

  EXECUTE_PERIODIC(TELEM_UPDATE_INTERVAL_MILLIS, { sendTelemData(batVoltage); });
}

static void updateMotors(motor_outputs_t outputs)
{
  if (!_completedFirstArm) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!_armed) {
      _speedControllers[i]->setSpeed(MIN_THROTTLE_RANGE);
    } else {
      _speedControllers[i]->setSpeed(map(outputs[i], 1000, 2000, MIN_THROTTLE_RANGE, MAX_THROTTLE_RANGE));
    }
  }
}

static uint64_t _loopCounter = 1;

void loop()
{
  TIMERG0.wdtwprotect.val = 0x50D83AA1;
  TIMERG0.wdtfeed.val = 1;
  TIMERG0.wdtwprotect.val = 0;
  uint64_t timestamp = micros();
  uint64_t timestampMillis = millis();
  if (_bluetoothController.isProcessingBluetoothTransaction) {
    return;
  }
  _loopCounter++;
  EXECUTE_PERIODIC(1000, {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::LoopUpdateRate, &_loopCounter, sizeof(uint64_t));
    _loopCounter = 0;
  });

  _telemetryController->loopHandler();
  _compass.loopHandler();

  if (_armed != _previousArmStatus) {
    LOG_INFO("Updating arm LED");
    _updateArmStatus();
    LOG_INFO("Updated arm LED");
    _previousArmStatus = _armed;
    LOG_INFO("Updated previous arm status");
  }

  if (_sendDebugData) {
    LOG_INFO("Beginning debug data transmission");
    _sendDebugData = false;
    uploadDebugData();
    LOG_INFO("Completed debug data transmission");
    return;
  }

  if (_resetFlag) {
    LOG_INFO("Resetting");
    _resetFlag = false;
    initController();
  }

  const float batteryVoltage = batteryLevel();
  updateClientTelemetryIfNeeded(batteryVoltage);
  _helper->voltage = batteryVoltage;

  if (!_startMonitoringPid && _controllerValues.leftStickInput.y > 20.0f) {
    _startMonitoringPid = true;
  }

  _imu->loopHandler();

  if (!_receivedImuUpdate || !_receivedMagUpdate) {
    // Wait until we have both an IMU and magnetometer read before
    // proceeding
    return;
  }
  _receivedImuUpdate = false;

  if (!_startMonitoringPid) {
    return;
  }

#ifdef ENABLE_EMERGENCY_MODE
  // Check if we need to enter into emergency mode
  if (!_enteredEmergencyMode && (fabs(_euler.angle.pitch) > 80.0f || fabs(_euler.angle.roll) > 80.0f)) {
    // The drone has entered into an unacceptable orientation - this kills
    // the motors
    _enteredEmergencyMode = true;
    _armed = false;
    _updateArmStatus();
    LOG_ERROR("ENTERED EMERGENCY MODE, killing motors: pitch = %f, roll = %f", _euler.angle.pitch, _euler.angle.roll);
  }
#endif

  if (_enteredEmergencyMode) {
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
    motor_outputs_t motors = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
    updateMotors(motors);

    // Flash LED orange to indicate we entered into emergency mode
    static bool lightOn = false;
    EXECUTE_PERIODIC(100, {
      if (!lightOn) {
        _updateLED(255, 165, 0);
      } else {
        _updateLED(0, 0, 0);
      }
      lightOn = !lightOn;
    });
    return;
  }

  if (_motorDebugEnabled) {
    motor_outputs_t motors = {_motorDebugValues[0], _motorDebugValues[1], _motorDebugValues[2], _motorDebugValues[3]};
    updateMotors(motors);
    return;
  }

  static bool beganRun = false;
  if (!beganRun && _controllerValues.leftStickInput.y < 20.0f) {
    return;
  } else if (!beganRun) {
    beganRun = true;
  }

  motor_outputs_t outputs = _controller->calculateOutputs(_imuValues, _controllerValues, micros(), _recordDebugData);

  _previousMotorOutputs = outputs;

  if (_controllerValues.leftStickInput.y < 20) {
    motor_outputs_t motors = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
    updateMotors(motors);
  } else {
    // Update motors only at 300hz
    static unsigned long lastMotorUpdateMillis = 0;
    if (timestampMillis - lastMotorUpdateMillis >= 3) {
      lastMotorUpdateMillis = timestampMillis;
      updateMotors(outputs);
    }
  }
  _setMotorOutputs = true;

  if (_recordDebugData) {
    _helper->saveValues(micros());
  }

  if (_recordDebugData && !_recordDebugDataLEDStateChanged &&
      timestampMillis - _startedRecordingDebugDataTimeMillis < RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS) {
    // Switch LED to emit blue light
    _recordDebugDataLEDStateChanged = true;
    _updateLED(0, 0, 254);
    LOG_INFO("Flashing LED blue");
  } else if (
      _recordDebugData && _recordDebugDataLEDStateChanged &&
      timestampMillis - _startedRecordingDebugDataTimeMillis > RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS) {
    // Switch LED back to emit red light
    _updateLED(254, 0, 0);
    _recordDebugDataLEDStateChanged = false;
    LOG_INFO("Switched LED back to red");
  }
}