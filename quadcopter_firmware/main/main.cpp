

#include <DebugHelper.h>
#include <QuadcopterController.h>
#include <Wire.h>

#include <string>

// Magnetometer
#include <DFRobot_QMC5883.h>
#include <Fusion.h>
#include <esp_partition.h>

#include "AsyncController.h"
#include "BLEController.h"
#include "IMU.h"
#include "Logger.h"
#include "MotorController.h"
#include "SPI.h"
#include "TelemetryController.h"
#include "esp_log.h"
#include "esp_system.h"
#include "led_controller.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

static std::vector<MotorController> _speedControllers;

// LED Setup
#define LED_DATA_PIN GPIO_NUM_38

static LEDController ledController(LED_DATA_PIN);

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

bool _armed = false;
bool previousArmStatus = false;

motor_outputs_t _previousMotorOutputs;
bool setMotorOutputs = false;

bool _resetFlag = false;

bool _motorDebugEnabled = false;
float _motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

bool startMonitoringPid = false;

bool _sendDebugData = false;
bool _recordDebugData = false;

static bool _completedFirstArm = false;

// When the quadcopter enters into unacceptable orientation for more than 2
// measurements, the drone will cut power to the motors for safety and must be
// rebooted
static bool _enteredEmergencyMode = false;

DFRobot_QMC5883 _compass(&Wire, /*I2C addr*/ QMC5883_ADDRESS);

// Keep these 2 in sync
#define MAG_UPDATE_RATE_MILLIS 5
#define MAG_DATARATE QMC5883_DATARATE_200HZ
static uint64_t _lastMagnetometerRead = 0;
static sVector_t _magValues;

static IMU *imu = NULL;
static imu_output_t _imuValues;
static FusionAhrs _fusion;

static volatile bool _calibrate = false;

// When we start recording debug data, we want the LED to flash blue
// for RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS. This lets us align
// any video recordings with the debug data that we're recording
#define RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS 200
static uint64_t _startedRecordingDebugDataTimeMillis = 0;
static bool _recordDebugDataLEDStateChanged = false;

static void updateArmStatus(void)
{
  ledController.showRGB(_armed ? 255 : 0, _armed ? 0 : 255, 0);
  if (_armed && !_completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    for (int i = 0; i < NUM_MOTORS; i++) {
      LOG_INFO("Attaching motor %i to pin %i", i, motorPins[i]);
      MotorController controller = MotorController(motorPins[i]);
      _speedControllers.push_back(controller);
    }
    _completedFirstArm = true;
  }
  _telemetryController->updateTelemetryEvent(TelemetryEvent::ArmStatusChange, &_armed, sizeof(bool));
}

QuadcopterController *controller;
DebugHelper *helper;

static void initController()
{
  delete controller;
  controller = new QuadcopterController(
      {
          .angleGains =
              {{// yaw
                .kP = 9.0f,
                .kI = 0.01f,
                .kD = 0.0f},
                           {// pitch
                .kP = 8.0f,
                .kI = 0.05f,
                .kD = 0.0f},
                           {// roll
                .kP = 8.0f,
                .kI = 0.05f,
                .kD = 0.0f}   },
          .rateGains =
              {{// yaw
                .kP = 3.1f,
                .kI = 0.01f,
                .kD = 0.0005f},
                           {// pitch
                .kP = 2.7f,
                .kI = 0.07f,
                .kD = 0.0008f},
                           {// roll
                .kP = 2.7f,
                .kI = 0.07f,
                .kD = 0.0008f}}
  },
      helper,
      micros());
}

static controller_values_t _controllerValues = {
    .leftStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = 0.0f                            },
    .rightStickInput = {.x = INPUT_MAX_CONTROLLER_INPUT / 2.0, .y = INPUT_MAX_CONTROLLER_INPUT / 2.0}
};

static FusionEuler _euler;
static uint64_t _previousMicros = 0;
static const FusionMatrix gyroscopeMisalignment = {0.7071f, -0.7071f, 0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f, 0.0f, 1.0f};
static const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
static const FusionMatrix accelerometerMisalignment = {
    0.7071f, -0.7071f, 0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f, 0.0f, 1.0f};
static const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
static bool _receivedImuUpdate = false;
static bool _gotFirstIMUUpdate = false;

static float _previousFilteredAccelValues[3] = {0.0f, 0.0f, 1.0f};
static float _previousFilteredGyroValues[3] = {0.0f, 0.0f, 0.0f};
static const float _accelerometerLowPassAlpha = 0.1f;  // lower alpha = more smoothing but more lag
static const float _gyroscopeLowPassAlpha = 0.5f;      // lower alpha = more smoothing but more lag
static void _receivedIMUUpdate(imu_update_t update)
{
  const float deltaTimeSeconds = (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();

  FusionVector gyroscope = {
      update.gyro_x,
      update.gyro_y,
      update.gyro_z};  // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {update.accel_x, update.accel_y, update.accel_z};

  accelerometer.axis.x = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[0]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.x);
  accelerometer.axis.y = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[1]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.y);
  accelerometer.axis.z = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[2]) +
                         (_accelerometerLowPassAlpha * accelerometer.axis.z);
  _previousFilteredAccelValues[0] = accelerometer.axis.x;
  _previousFilteredAccelValues[1] = accelerometer.axis.y;
  _previousFilteredAccelValues[2] = accelerometer.axis.z;
  gyroscope.axis.x =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[0]) + (_gyroscopeLowPassAlpha * gyroscope.axis.x);
  gyroscope.axis.y =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[1]) + (_gyroscopeLowPassAlpha * gyroscope.axis.y);
  gyroscope.axis.z =
      ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[2]) + (_gyroscopeLowPassAlpha * gyroscope.axis.z);
  _previousFilteredGyroValues[0] = gyroscope.axis.x;
  _previousFilteredGyroValues[1] = gyroscope.axis.y;
  _previousFilteredGyroValues[2] = gyroscope.axis.z;
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(
      accelerometer,
      accelerometerMisalignment,
      accelerometerSensitivity,
      accelerometerOffset);

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateExternalHeading(&_fusion, gyroscope, accelerometer, _magValues.HeadingDegress, deltaTimeSeconds);
  _euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&_fusion));

  _imuValues = {
      .gyroOutput = {gyroscope.axis.x, gyroscope.axis.y,   gyroscope.axis.z },
      .yawPitchRollDegrees = {_euler.angle.yaw, _euler.angle.pitch, _euler.angle.roll}
  };

  _receivedImuUpdate = true;
  _gotFirstIMUUpdate = true;

  helper->accelFiltered[0] = accelerometer.axis.x;
  helper->accelFiltered[1] = accelerometer.axis.y;
  helper->accelFiltered[2] = accelerometer.axis.z;
  helper->accelRaw[0] = update.accel_x;
  helper->accelRaw[1] = update.accel_y;
  helper->accelRaw[2] = update.accel_z;
  helper->gyroFiltered[0] = gyroscope.axis.x;
  helper->gyroFiltered[1] = gyroscope.axis.y;
  helper->gyroFiltered[2] = gyroscope.axis.z;
  helper->gyroRaw[0] = update.gyro_x;
  helper->gyroRaw[1] = update.gyro_y;
  helper->gyroRaw[2] = update.gyro_z;
  helper->ypr[0] = _euler.angle.yaw;
  helper->ypr[1] = _euler.angle.pitch;
  helper->ypr[2] = _euler.angle.roll;

  LOG_INFO_PERIODIC_MILLIS(
      100,  // log at most up to every 100 millis
      "%ld, %.2f, %.2f, %.2f, %.2f, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %i, %i, %i, %.2f, %.2f, %.2f, "
      "%.2f, %.2f",
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
      _magValues.HeadingDegress,
      _controllerValues.leftStickInput.x,
      _controllerValues.leftStickInput.y,
      _controllerValues.rightStickInput.x,
      _controllerValues.rightStickInput.y);
}

// Initial setup
void setup()
{
  const unsigned long initializationTime = millis();
  Serial.begin(115200);
  LOG_INFO("BEGAN APP");

  helper = new DebugHelper();

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

  _bluetoothController.setCalibrationUpdateHandler([&]() { _calibrate = true; });

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

  while (!_compass.begin()) {
    LOG_ERROR("Failed to initialize magnetometer");
    delay(500);
  }
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on:
   * http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  const float declinationAngle = (12.0 + (55.0 / 60.0)) / (180 / PI);
  _compass.setRange(QMC5883_RANGE_2GA);
  _compass.setMeasurementMode(QMC5883_CONTINOUS);
  _compass.setDataRate(QMC5883_DATARATE_200HZ);
  _compass.setSamples(QMC5883_SAMPLES_8);
  _compass.setDeclinationAngle(declinationAngle);
  LOG_INFO("Successfully initialized magnetometer");

  updateArmStatus();

  LOG_INFO("Initializing attitude & heading reference system");
  // FusionOffsetInitialise(&_offset, 4000);
  FusionAhrsInitialise(&_fusion);

  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.2f,
      .accelerationRejection = 90.0f,
      .recoveryTriggerPeriod = 5000, /* 5 seconds */
  };
  FusionAhrsSetSettings(&_fusion, &settings);

  LOG_INFO("Initializing IMU");
  bool setupSuccess = false;
  imu = new IMU(
      SPI0_CS_PIN,
      SPI0_MISO_PIN,
      SPI0_MOSI_PIN,
      SPI0_SCLK_PIN,
      IMU_INT_PIN,
      [](imu_update_t update) { _receivedIMUUpdate(update); },
      &setupSuccess);

  if (!setupSuccess) {
    LOG_ERROR("IMU Setup Failure");
  } else {
    LOG_INFO("Successfully set up IMU");
  }

  while (!setupSuccess) {
  };

  LOG_INFO("Initializing Arming Signal LED");
  ledController.showRGB(0, 255, 0);

  LOG_INFO("SETUP COMPLETE AFTER %lu ms", millis() - initializationTime);
}

void uploadDebugData()
{
  DebugDataManager manager = helper->dataManager;
  uint8_t *data = manager.data;
  uint64_t dataSize = manager.numSamples * DEBUG_PACKET_SIZE;
  _bluetoothController.uploadDebugData(data, dataSize);
}

float batteryPercent(float batteryVoltage)
{
  const float averageVoltagePerCell = batteryVoltage / (float)NUM_BATTERY_CELLS;
  const float cellRange = BATTERY_CELL_MAX_VOLTAGE - BATTERY_CELL_MIN_VOLTAGE;
  return (averageVoltagePerCell - BATTERY_CELL_MIN_VOLTAGE) / cellRange;
}

float batteryLevel()
{
  const int val = analogRead(BATTERY_SENSE_PIN);
  return BATTERY_SCALE * (float)val;
}

void sendTelemData(float batVoltage)
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

void updateClientTelemetryIfNeeded(float batVoltage)
{
  if (!_bluetoothController.isConnected) {
    return;
  }

  EXECUTE_PERIODIC(TELEM_UPDATE_INTERVAL_MILLIS, { sendTelemData(batVoltage); });
}

void updateMotors(motor_outputs_t outputs)
{
  if (!_completedFirstArm) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!_armed) {
      _speedControllers[i].setSpeed(MIN_THROTTLE_RANGE);
    } else {
      _speedControllers[i].setSpeed(map(outputs[i], 1000, 2000, MIN_THROTTLE_RANGE, MAX_THROTTLE_RANGE));
    }
  }
}

unsigned long startedMonitoringTimestamp = 0;

static int samples = 0;
static int lastPrint = 0;

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

  _telemetryController->loopHandler();

  if (_armed != previousArmStatus) {
    LOG_INFO("Updating arm LED");
    updateArmStatus();
    LOG_INFO("Updated arm LED");
    previousArmStatus = _armed;
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

  if (timestampMillis - _lastMagnetometerRead > MAG_UPDATE_RATE_MILLIS) {
    _magValues = _compass.readRaw();
    _compass.getHeadingDegrees();
    _lastMagnetometerRead = timestampMillis;
  }

  const float batteryVoltage = batteryLevel();
  updateClientTelemetryIfNeeded(batteryVoltage);
  helper->voltage = batteryVoltage;

  if (!startMonitoringPid && _controllerValues.leftStickInput.y > 20.0f) {
    startMonitoringPid = true;
    startedMonitoringTimestamp = timestamp;
  }

  imu->loopHandler();

  if (_calibrate) {
    _calibrate = false;
    calibration_data_t dat = imu->calibrate();
    if (dat.success) {
      LOG_INFO("Calibration successful");
      _bluetoothController.sendCalibrationData(dat);
    } else {
      LOG_ERROR("Calibration failed");
    }
  }

  if (!_receivedImuUpdate || _lastMagnetometerRead == 0) {
    // Wait until we have both an IMU and magnetometer read before
    // proceeding
    return;
  }
  _receivedImuUpdate = false;

  if (!startMonitoringPid) {
    return;
  }

#ifdef ENABLE_EMERGENCY_MODE
  // Check if we need to enter into emergency mode
  if (!_enteredEmergencyMode && (fabs(_euler.angle.pitch) > 80.0f || fabs(_euler.angle.roll) > 80.0f)) {
    // The drone has entered into an unacceptable orientation - this kills
    // the motors
    _enteredEmergencyMode = true;
    _armed = false;
    updateArmStatus();
    LOG_ERROR("ENTERED EMERGENCY MODE, killing motors: pitch = %f, roll = %f", _euler.angle.pitch, _euler.angle.roll);
  }
#endif

  if (_enteredEmergencyMode) {
    motor_outputs_t motors = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
    updateMotors(motors);
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

  motor_outputs_t outputs = controller->calculateOutputs(_imuValues, _controllerValues, micros(), _recordDebugData);

  samples++;
  if (timestampMillis - lastPrint > 1000) {
    lastPrint = timestampMillis;
    // LOG_INFO("%i samples per second", samples);
    // LOG_INFO("Throttle = %f, motors %f, %f, %f, %f",
    //          _controllerValues.leftStickInput.y, outputs[0], outputs[1],
    //          outputs[2], outputs[3]);
    samples = 0;
  }

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
  setMotorOutputs = true;

  static unsigned long lastRecordMillis = timestampMillis;
  if (_recordDebugData && timestampMillis - lastRecordMillis >= 5) {
    helper->saveValues(timestampMillis);
    lastRecordMillis = timestampMillis;
  }

  if (_recordDebugData && !_recordDebugDataLEDStateChanged &&
      timestampMillis - _startedRecordingDebugDataTimeMillis < RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS) {
    // Switch LED to emit blue light
    _recordDebugDataLEDStateChanged = true;
    ledController.showRGB(0, 0, 254);
    LOG_INFO("Flashing LED blue");
  } else if (
      _recordDebugData && _recordDebugDataLEDStateChanged &&
      timestampMillis - _startedRecordingDebugDataTimeMillis > RECORD_DEBUG_DATA_LED_FLASH_INTERVAL_MILLIS) {
    // Switch LED back to emit red light
    ledController.showRGB(254, 0, 0);
    _recordDebugDataLEDStateChanged = false;
    LOG_INFO("Switched LED back to red");
  }
}