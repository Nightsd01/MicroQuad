

#include <DebugHelper.h>
#include <QuadcopterController.h>
#include <Wire.h>

#include <string>

// Magnetometer
#include <QMC5883L.h>
#include <esp_partition.h>

#include "AsyncController.h"
#include "BLEController.h"
#include "BatteryController.h"
#include "Filters/KalmanFilter.h"
#include "Filters/MedianFilter.h"
#include "IMU.h"
#include "LEDController.h"
#include "Logger.h"
#include "MahonyAHRS.h"
#include "MotorController.h"
#include "MotorMagCompensationHandler.h"
#include "PIDPreferences.h"
#include "PersistentKeyValueStore.h"
#include "PersistentKeysCommon.h"
#include "PinDefines.h"
#include "SPI.h"
#include "TelemetryController.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

static std::vector<MotorController *> _speedControllers;

BatteryController *_batteryController;

// LED Setup

static LEDController _ledController(LED_DATA_PIN);

struct _memory_usage_telemetry_packet_t
{
  uint32_t freeHeap;
  uint32_t minFreeHeap;
  uint32_t totalHeapSize;
};

#define TELEM_UPDATE_INTERVAL_MILLIS 100
static TelemetryController *_telemetryController;

#define MIN_THROTTLE 1015

#define ARM_MICROSECONDS 1000.0f

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

static QMC5883L *_compass;

static mag_update_t _magValues;
static bool _receivedMagUpdate = false;

PersistentKeyValueStore _persistentKvStore;

static IMU *_imu = NULL;
static imu_output_t _imuValues;

static volatile bool _calibrate = false;

static PIDPreferences *_pidPreferences;

MahonyAHRS ahrs;
EulerAngle _euler;
static const float _misalignmentMatrix[3][3] = {
    {0.7071f, -0.7071f, 0.0f},
    {0.7071f, 0.7071f,  0.0f},
    {0.0f,    0.0f,     1.0f}
};

MotorMagCompensationHandler *_motorMagCompensationHandler;

motor_outputs_t _mostRecentMotorValues = {};

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

static void updateMotors(motor_outputs_t outputs)
{
  if (!_completedFirstArm) {
    return;
  }
  _mostRecentMotorValues = outputs;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!_armed) {
      _speedControllers[i]->setSpeed(MIN_THROTTLE_RANGE);
    } else {
      _speedControllers[i]->setSpeed(map(outputs[i], 1000, 2000, MIN_THROTTLE_RANGE, MAX_THROTTLE_RANGE));
      // LOG_INFO_PERIODIC_MILLIS(100, "Motor %d, %f, %f, %f, %f", i, outputs[0], outputs[1], outputs[2], outputs[3]);
    }
  }
}

static void _updateArmStatus(void)
{
  if (_armed && !_completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    for (int i = 0; i < NUM_MOTORS; i++) {
      LOG_INFO("Attaching motor %i to pin %i", i, MOTOR_PINS[i]);
      MotorController *controller = new MotorController(MOTOR_PINS[i], MOTOR_TELEM_PINS[i], false);
      _speedControllers.push_back(controller);
    }
    _completedFirstArm = true;
    updateMotors({1000.0f, 1000.0f, 1000.0f, 1000.0f});
  }
  _telemetryController->updateTelemetryEvent(TelemetryEvent::ArmStatusChange, &_armed, sizeof(bool));
  if (!_enteredEmergencyMode) {
    _updateLED(_armed ? 255 : 0, _armed ? 0 : 255, 0);
  }
}

static void _gotMagUpdate(mag_update_t update)
{
  _receivedMagUpdate = true;
  _magValues = update;
  if (_motorMagCompensationHandler->isCalibrating) {
    _motorMagCompensationHandler->updateMagValue(update);
  } else if (_completedFirstArm && _mostRecentMotorValues[0] > 0 && _motorMagCompensationHandler->isCalibrated) {
    _magValues = _motorMagCompensationHandler->applyMagneticMotorCompensation(update, _mostRecentMotorValues);
  }

  EXECUTE_PERIODIC(1000, {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::MagnetometerXYZRaw, &update, sizeof(mag_update_t));
  });
}

static void _configureMagnetometer(void)
{
  _compass = new QMC5883L(&Wire, QMC5883_ADDRESS);
  _compass->addObserver([&](mag_update_t update) { _gotMagUpdate(update); });

  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on:
   * http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  const float declinationAngle = (12.0 + (55.0 / 60.0)) / (180 / PI);
  _compass->setRange(QMC5883_RANGE_2GA);
  _compass->setMeasurementMode(QMC5883_CONTINOUS);
  _compass->setDataRate(QMC5883_DATARATE_200HZ);
  _compass->setSamples(QMC5883_SAMPLES_4);
  _compass->setDeclinationAngle(declinationAngle);

  if (!_compass->begin()) {
    LOG_ERROR("Failed to begin compass");
    abort();
  }

  if (_persistentKvStore.hasValueForKey(PersistentKeysCommon::MAG_OFFSETS)) {
    const std::vector<float> offsets = _persistentKvStore.getVectorForKey<float>(PersistentKeysCommon::MAG_OFFSETS, 3);

    if (offsets.size() != 3) {
      LOG_WARN("magnetometer: Invalid calibration offsets found in persistent storage, please calibrate the compass");
    } else {
      LOG_INFO("Setting magnetometer offsets: %f, %f, %f", offsets[0], offsets[1], offsets[2]);
      _compass->setCalibrationOffsets({offsets[0], offsets[1], offsets[2]});
    }
  } else {
    LOG_WARN("magnetometer: No calibration offsets found in persistent storage, please calibrate the compass");
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
                .kP = 5.0f,
                .kI = 0.02f,
                .kD = 0.003f},
                           {// roll
                .kP = 5.0f,
                .kI = 0.02f,
                .kD = 0.003f}}
  },
      _helper,
      micros());
}

static void _handleMagnetometerCalibration(CalibrationResponse response)
{
  switch (response) {
    case CalibrationResponse::Start: {
      LOG_INFO("Starting magnetometer calibration");
      _bluetoothController.sendCalibrationUpdate(CalibrationType::Magnetometer, CalibrationRequest::Roll360);
      break;
    }
    case CalibrationResponse::Continue: {
      LOG_INFO("Beginning compass calibration");
      _compass->calibrate(8.0f /* seconds */, [&](bool succeeded, xyz_vector_t offsets) {
        LOG_INFO("Calibration %s", succeeded ? "succeeded" : "failed");
        if (!succeeded) {
          LOG_ERROR("Failed to calibrate compass");
          _bluetoothController.sendCalibrationUpdate(CalibrationType::Magnetometer, CalibrationRequest::Failed);
          return;
        }
        LOG_INFO("Successfully calibrated compass: %f, %f, %f", offsets.x, offsets.y, offsets.z);
        _bluetoothController.sendCalibrationUpdate(CalibrationType::Magnetometer, CalibrationRequest::Complete);
        std::vector<float> offsetsVector = {offsets.x, offsets.y, offsets.z};
        _persistentKvStore.setVectorForKey(PersistentKeysCommon::MAG_OFFSETS, offsetsVector);
      });
      break;
    }
    default: {
      LOG_ERROR("Received unexpected calibration response: %d", response);
      _bluetoothController.sendCalibrationUpdate(CalibrationType::Magnetometer, CalibrationRequest::Failed);
      break;
    }
  }
}

static void _beginMagneticMotorInterferenceCalibration(void)
{
  if (!_armed) {
    LOG_ERROR("Cannot start magnetometer-motors-compensation calibration when not armed");
    _bluetoothController.sendCalibrationUpdate(
        CalibrationType::MagnetometerMotorsCompensation,
        CalibrationRequest::Failed);
    return;
  }
  _motorDebugEnabled = true;
  _motorMagCompensationHandler->beginCalibration(
      [&](motor_outputs_t outputs) {
        LOG_INFO(
            "Updating motor outputs for calibration %f, %f, %f, %f",
            outputs[0],
            outputs[1],
            outputs[2],
            outputs[3]);
        for (int i = 0; i < NUM_MOTORS; i++) {
          _motorDebugValues[i] = outputs[i];
        }
      },
      [&](bool success) {
        // completion callback
        _motorDebugEnabled = false;
        if (!success) {
          LOG_ERROR("Failed to complete magnetometer motors compensation calibration");
          _bluetoothController.sendCalibrationUpdate(
              CalibrationType::MagnetometerMotorsCompensation,
              CalibrationRequest::Failed);
          return;
        }
        _bluetoothController.sendCalibrationUpdate(
            CalibrationType::MagnetometerMotorsCompensation,
            CalibrationRequest::Complete);
      });
}

static void _handleCalibration(CalibrationType type, CalibrationResponse response)
{
  switch (type) {
    case CalibrationType::AccelerometerGyro: {
      std::map<CalibrationResponse, std::function<void(void)>> handlers =
          _imu->calibrationHandlers([](CalibrationRequest request) {
            _bluetoothController.sendCalibrationUpdate(CalibrationType::AccelerometerGyro, request);
          });
      LOG_INFO("Handling calibration response %d", response);
      handlers[response]();
      break;
    }
    case CalibrationType::Magnetometer: {
      _handleMagnetometerCalibration(response);
      break;
    }
    case CalibrationType::MagnetometerMotorsCompensation: {
      _beginMagneticMotorInterferenceCalibration();
    }
  }
}

static controller_values_t _controllerValues = {
    .leftStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = 0.0f                            },
    .rightStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = INPUT_MAX_CONTROLLER_INPUT / 2.0}
};

static uint64_t _previousMicros = 0;
static bool _receivedImuUpdate = false;
static bool _gotFirstIMUUpdate = false;

static float _previousFilteredAccelValues[3] = {0.0f, 0.0f, 1.0f};
static float _previousFilteredGyroValues[3] = {0.0f, 0.0f, 0.0f};

static uint64_t _imuUpdateCounter = 0;

static const float _accelerometerLowPassAlpha = 0.075f;  // lower alpha = more smoothing but more lag
static const float _gyroscopeLowPassAlpha = 0.1f;        // lower alpha = more smoothing but more lag

#define MEDIAN_FILTER_WINDOW 10
static std::array<MedianFilter<float>, 3> _accelHistories = {
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

  Vector3f gyroscope = {update.gyro_x, update.gyro_y, update.gyro_z};
  Vector3f accelerometer = {update.accel_x, update.accel_y, update.accel_z};
  Vector3f mag = {_magValues.x, _magValues.y, _magValues.z};

  _accelHistories[0].addValue(accelerometer.x);
  accelerometer.x = _accelHistories[0].getMedian();
  _accelHistories[1].addValue(accelerometer.y);
  accelerometer.y = _accelHistories[1].getMedian();
  _accelHistories[2].addValue(accelerometer.z);
  accelerometer.z = _accelHistories[2].getMedian();

  accelerometer.x = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[0]) +
                    (_accelerometerLowPassAlpha * accelerometer.x);
  accelerometer.y = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[1]) +
                    (_accelerometerLowPassAlpha * accelerometer.y);
  accelerometer.z = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[2]) +
                    (_accelerometerLowPassAlpha * accelerometer.z);
  gyroscope.x =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[0]) + (_gyroscopeLowPassAlpha * gyroscope.x);
  gyroscope.y =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[1]) + (_gyroscopeLowPassAlpha * gyroscope.y);
  gyroscope.z =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[2]) + (_gyroscopeLowPassAlpha * gyroscope.z);

  _previousFilteredGyroValues[0] = gyroscope.x;
  _previousFilteredGyroValues[1] = gyroscope.y;
  _previousFilteredGyroValues[2] = gyroscope.z;
  _previousFilteredAccelValues[0] = accelerometer.x;
  _previousFilteredAccelValues[1] = accelerometer.y;
  _previousFilteredAccelValues[2] = accelerometer.z;

  ahrs.update(gyroscope, accelerometer, mag, deltaTimeSeconds);
  float y, p, r;
  ahrs.getYawPitchRoll(y, p, r);
  _euler = {.yaw = y, .pitch = p, .roll = r};

  _imuValues = {
      .gyroOutput = {gyroscope.x, gyroscope.y,  gyroscope.z},
      .yawPitchRollDegrees = {_euler.yaw,  _euler.pitch, _euler.roll}
  };

  _receivedImuUpdate = true;
  _gotFirstIMUUpdate = true;

  _helper->accelFiltered[0] = accelerometer.x;
  _helper->accelFiltered[1] = accelerometer.y;
  _helper->accelFiltered[2] = accelerometer.z;
  _helper->accelRaw[0] = update.accel_x;
  _helper->accelRaw[1] = update.accel_y;
  _helper->accelRaw[2] = update.accel_z;
  _helper->gyroFiltered[0] = gyroscope.x;
  _helper->gyroFiltered[1] = gyroscope.y;
  _helper->gyroFiltered[2] = gyroscope.z;
  _helper->gyroRaw[0] = update.gyro_x;
  _helper->gyroRaw[1] = update.gyro_y;
  _helper->gyroRaw[2] = update.gyro_z;
  _helper->ypr[0] = _euler.yaw;
  _helper->ypr[1] = _euler.pitch;
  _helper->ypr[2] = _euler.roll;
  _helper->magValues[0] = _magValues.x;
  _helper->magValues[1] = _magValues.y;
  _helper->magValues[2] = _magValues.z;
  _helper->magValues[3] = _magValues.heading;

  const int stat1 = digitalRead(BATTERY_STAT1_PIN);
  const int stat2 = digitalRead(BATTERY_STAT2_PIN);
  const int pg = digitalRead(BATTERY_PG_PIN);

  LOG_INFO_PERIODIC_MILLIS(
      100,  // log at most up to every 100 millis
      "%8ld, %i, %i, %i, %8.2f, %8.2f, %8.2f, %6d, %6d, %6d, %8.2f, %8.2f, %8.2f, %8.2f, %8.2f, %8.2f, %6d, %6d, %6d, "
      "%8.2f, %f, %f, %f",
      millis(),
      stat1,
      stat2,
      pg,
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
  Wire.setClock(400000);
  Wire.begin(18, 17);
  Serial.begin(115200);
  LOG_INFO("BEGAN APP");

  _helper = new DebugHelper();

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

  LOG_INFO("Initializing PID preferences");
  _pidPreferences = new PIDPreferences(&_bluetoothController, &_persistentKvStore);

  LOG_INFO("Initializing quadcopter controller");
  initController();

  LOG_INFO("Initializing telemetry controller");
  _telemetryController = new TelemetryController(&_bluetoothController);

  LOG_INFO("Initializing battery controller");
  _batteryController = new BatteryController(_telemetryController, &_bluetoothController, _helper);

  LOG_INFO("Configuring magnetometer");
  _configureMagnetometer();
  LOG_INFO("Successfully initialized magnetometer");

  LOG_INFO("Initializing magnetometer motors compensation handler");
  _motorMagCompensationHandler = new MotorMagCompensationHandler(&_persistentKvStore);

  _updateArmStatus();

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

static void sendTelemData()
{
  _telemetryController->updateTelemetryEvent(
      TelemetryEvent::MotorValues,
      _previousMotorOutputs.data(),
      NUM_MOTORS * sizeof(float));

  if (_gotFirstIMUUpdate) {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::EulerYawPitchRoll, &_euler, sizeof(float) * 3);
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

static void updateClientTelemetryIfNeeded()
{
  if (!_bluetoothController.isConnected) {
    return;
  }

  EXECUTE_PERIODIC(TELEM_UPDATE_INTERVAL_MILLIS, { sendTelemData(); });
}

static uint64_t _loopCounter = 1;

void loop()
{
  TIMERG0.wdtwprotect.val = 0x50D83AA1;
  TIMERG0.wdtfeed.val = 1;
  TIMERG0.wdtwprotect.val = 0;
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
  _compass->loopHandler();
  _bluetoothController.loopHandler();
  _batteryController->loopHandler();

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

  updateClientTelemetryIfNeeded();

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
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
    return;
  }

#ifdef ENABLE_EMERGENCY_MODE
  // Check if we need to enter into emergency mode
  if (!_enteredEmergencyMode && (fabs(_euler.pitch) > 80.0f || fabs(_euler.roll) > 80.0f)) {
    // The drone has entered into an unacceptable orientation - this kills
    // the motors
    _enteredEmergencyMode = true;
    _armed = false;
    _updateArmStatus();
    LOG_ERROR("ENTERED EMERGENCY MODE, killing motors: pitch = %f, roll = %f", _euler.pitch, _euler.roll);
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
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
    return;
  }

  static bool beganRun = false;
  if (!beganRun && _controllerValues.leftStickInput.y < 20.0f) {
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
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