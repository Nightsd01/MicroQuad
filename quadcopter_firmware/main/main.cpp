

#include <DebugHelper.h>
#include <QuadcopterController.h>
#include <Wire.h>

#include <string>

// Magnetometer
#include <Matrix.h>
#include <QMC5883L.h>
#include <esp_partition.h>

#include <algorithm>

#include "AsyncController.h"
#include "BLEController.h"
#include "Barometer.h"
#include "BatteryController.h"
#include "Constants.h"
#include "ExtendedKalmanFilter.h"
#include "Filters/KalmanFilter.h"
#include "Filters/MedianFilter.h"
#include "IMU.h"
#include "LEDController.h"
#include "Logger.h"
#include "MotionDetector.h"
#include "MotorController.h"
#include "MotorMagCompensationHandler.h"
#include "PIDPreferences.h"
#include "PersistentKeyValueStore.h"
#include "PersistentKeysCommon.h"
#include "SPI.h"
#include "TelemetryController.h"
#include "VL53Manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif  // MAX

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif  // MIN

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

static VL53Manager _vl53Manager;

// Used at startup so we can determine when the quadcopter is not in motion, in order to
// perform a quick gyro calibration
static MotionDetector _motionDetector;

EulerAngle _euler;

static ExtendedKalmanFilter::Config _ekfConfig;

static ExtendedKalmanFilter _extendedKalmanFilter(_ekfConfig);

static Barometer _barometer;
static bool _receivedAltitudeUpdate = false;
static float _relativeAltitudeMeters = 0.0f;

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
  if (_speedControllers.size() == 0) {
    return;
  }
  if (_armed && !_completedFirstArm) {
    _completedFirstArm = true;
    updateMotors({1000.0f, 1000.0f, 1000.0f, 1000.0f});
  }
  _telemetryController->updateTelemetryEvent(TelemetryEvent::ArmStatusChange, &_armed, sizeof(bool));
  if (!_enteredEmergencyMode) {
    _updateLED(_armed ? 255 : 0, _armed ? 0 : 255, 0);
  }
}

void _setupMotors(void)
{
  LOG_INFO("Setting up motor outputs");
  for (int i = 0; i < NUM_MOTORS; i++) {
    LOG_INFO("Attaching motor %i to pin %i", i, MOTOR_PINS[i]);
    MotorController *controller = new MotorController(MOTOR_PINS[i], MOTOR_TELEM_PINS[i], false);
    _speedControllers.push_back(controller);
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
  if (_imu->completedQuickCalibration) {
    // TODO: Commented out for debugging
    // _extendedKalmanFilter.updateMagnetometer(_magValues.x, _magValues.y, _magValues.z);
  }

  EXECUTE_PERIODIC(1000, {
    LOG_INFO("Sending mag update with heading %f, from update %f", _magValues.heading, update.heading);
    _telemetryController->updateTelemetryEvent(TelemetryEvent::MagnetometerXYZRaw, &_magValues, sizeof(mag_update_t));
  });
}

static void _configureMagnetometer(void)
{
  _compass = new QMC5883L(&Wire, QMC5883_ADDRESS);
  _compass->addObserver([&](mag_update_t update) { _gotMagUpdate(update); });

  _compass->setRange(QMC5883_RANGE_2GA);
  _compass->setMeasurementMode(QMC5883_CONTINOUS);
  _compass->setDataRate(QMC5883_DATARATE_200HZ);
  _compass->setSamples(QMC5883_SAMPLES_4);
  _compass->setDeclinationAngle(DECLINATION_ANGLE_DEG);

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
  _controller = new QuadcopterController(_helper, micros());
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

static const Matrix<float, 3, 3> _misalignmentMatrix = {
    {0.7071f, -0.7071f, 0.0f},
    {0.7071f, 0.7071f,  0.0f},
    {0.0f,    0.0f,     1.0f}
};

static Vector3f _applyMisalignment(const Vector3f &v, const Matrix<float, 3, 3> &mat)
{
  return {
      .x = mat(0, 0) * v.x + mat(0, 1) * v.y + mat(0, 2) * v.z,
      .y = mat(1, 0) * v.x + mat(1, 1) * v.y + mat(1, 2) * v.z,
      .z = mat(2, 0) * v.x + mat(2, 1) * v.y + mat(2, 2) * v.z};
}

static uint64_t _previousMicros = 0;
static bool _receivedFirstImuUpdate = false;
static bool _computedEuler = false;

static uint64_t _imuUpdateCounter = 0;

constexpr float STANDARD_GRAVITY = 9.81f;  // or 9.80665f for higher precision

static void _receivedIMUUpdate(imu_update_t update)
{
  const float deltaTimeSeconds = (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();
  _receivedFirstImuUpdate = true;

  Vector3f gyroscope = {update.gyro_x, update.gyro_y, update.gyro_z};
  Vector3f accelerometer = {update.accel_x, update.accel_y, update.accel_z};
  Vector3f mag = {_magValues.x, _magValues.y, _magValues.z};

  gyroscope = _applyMisalignment(gyroscope, _misalignmentMatrix);
  accelerometer = _applyMisalignment(accelerometer, _misalignmentMatrix);
  mag = _applyMisalignment(mag, _misalignmentMatrix);

  if (!_imu->completedQuickCalibration) {
    _motionDetector.imuUpdate(update, micros());
    return;
  }
  _extendedKalmanFilter.predict(
      gyroscope.x,
      gyroscope.y,
      gyroscope.z,
      accelerometer.x * STANDARD_GRAVITY,
      accelerometer.y * STANDARD_GRAVITY,
      accelerometer.z * STANDARD_GRAVITY,
      deltaTimeSeconds);

  // NOTE: Due to all the matrix allocations, we cannot combine this with predict()
  // because of the limited stack size on the ESP32
  _extendedKalmanFilter.updateAccelerometer(
      accelerometer.x * STANDARD_GRAVITY,
      accelerometer.y * STANDARD_GRAVITY,
      accelerometer.z * STANDARD_GRAVITY);

  const auto ekfAttitudeQuaternion = _extendedKalmanFilter.getAttitudeQuaternion();
  const auto ekfYawPitchRoll = _extendedKalmanFilter.getYawPitchRollDegrees();
  auto ekfAltitude = MAX(_extendedKalmanFilter.getAltitude(), 0.0f);
  auto ekfVerticalVelocity = _extendedKalmanFilter.getVerticalVelocity();

  _euler = {.yaw = ekfYawPitchRoll(0, 0), .pitch = ekfYawPitchRoll(1, 0), .roll = ekfYawPitchRoll(2, 0)};

  _imuValues = {
      .gyroOutput = {gyroscope.z, gyroscope.y,  gyroscope.x},
      .yawPitchRollDegrees = {_euler.yaw,  _euler.pitch, _euler.roll},
      .altitudeMeters = ekfAltitude,
      .verticalVelocityMetersPerSec = ekfVerticalVelocity,
  };

  _vl53Manager.updatedAttitude(_euler);

  _computedEuler = true;

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
  _helper->ekfQuaternion[0] = ekfAttitudeQuaternion(0, 0);
  _helper->ekfQuaternion[1] = ekfAttitudeQuaternion(1, 0);
  _helper->ekfQuaternion[2] = ekfAttitudeQuaternion(2, 0);
  _helper->ekfQuaternion[3] = ekfAttitudeQuaternion(3, 0);
  _helper->ekfYawPitchRoll[0] = ekfYawPitchRoll(0, 0);
  _helper->ekfYawPitchRoll[1] = ekfYawPitchRoll(1, 0);
  _helper->ekfYawPitchRoll[2] = ekfYawPitchRoll(2, 0);
  _helper->ekfAltitude = ekfAltitude;
  _helper->ekfVerticalVelocity = ekfVerticalVelocity;

  const int stat1 = digitalRead(BATTERY_STAT1_PIN);
  const int stat2 = digitalRead(BATTERY_STAT2_PIN);
  const int pg = digitalRead(BATTERY_PG_PIN);

  _imuUpdateCounter++;
  EXECUTE_PERIODIC(1000, {
    _telemetryController->updateTelemetryEvent(TelemetryEvent::IMUUpdateRate, &_imuUpdateCounter, sizeof(uint64_t));
    _telemetryController->updateTelemetryEvent(TelemetryEvent::EKFAltitudeEstimate, &ekfAltitude, sizeof(float));
    _telemetryController->updateTelemetryEvent(
        TelemetryEvent::EKFVerticalVelocityEstimate,
        &ekfVerticalVelocity,
        sizeof(float));
    _imuUpdateCounter = 0;
  });
}

static void _armProcedure(bool arm) { _armed = arm; }

static uint64_t _firstStageSetupCompletionTimeMillis = 0;

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

  _bluetoothController.setArmStatusUpdateHandler([&](bool armStatus) { _armProcedure(armStatus); });

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

  LOG_INFO("Initializing barometric sensor");
  _barometer.begin(
      [](float relativeAltMeters) {
        _receivedAltitudeUpdate = true;
        _relativeAltitudeMeters = relativeAltMeters;
        if (std::isfinite(_relativeAltitudeMeters) && _relativeAltitudeMeters >= 0.0f &&
            _imu->completedQuickCalibration) {
          _extendedKalmanFilter.updateBarometer(relativeAltMeters);
        }
      },
      _telemetryController,
      0x76,
      &Wire);

  LOG_INFO("Initializing VL53L1X sensor");
  _vl53Manager.begin(
      [&](float distance) {
        LOG_INFO("VL53L1X distance: %fm", distance);
        if (_imu->completedQuickCalibration) {
          _extendedKalmanFilter.updateRangefinder(distance);
        }
      },
      _telemetryController,
      0x29 /* i2c bus address */,
      &Wire);

  LOG_INFO("Initializing battery controller");
  _batteryController = new BatteryController(_telemetryController, &_bluetoothController, _helper);

  LOG_INFO("Configuring magnetometer");
  _configureMagnetometer();
  LOG_INFO("Successfully initialized magnetometer");

  LOG_INFO("Initializing magnetometer motors compensation handler");
  _motorMagCompensationHandler = new MotorMagCompensationHandler(&_persistentKvStore);

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

  LOG_INFO("SETUP FIRST STAGE COMPLETE AFTER %lu ms", millis() - initializationTime);

  _firstStageSetupCompletionTimeMillis = millis();
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

  if (_computedEuler) {
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

#define FIRST_STAGE_ARM_LED_BLINK_INTERVAL_MILLIS 100

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr uint32_t _kPulsePeriodMs = 2000;
static constexpr uint8_t _kMaxWhiteBrightness = 255;  // Use less than 255 if full brightness is too intense
static constexpr uint8_t _kMinWhiteBrightness = 5;
static constexpr float _kBrightnessRange = (float)_kMaxWhiteBrightness - (float)_kMinWhiteBrightness;
static constexpr float _kBrightnessScale = _kBrightnessRange / 2.0f;
static constexpr float _kBrightnessMidpoint = (float)_kMinWhiteBrightness + _kBrightnessScale;
static constexpr float _kRadsPerMs = (2.0f * M_PI) / (float)_kPulsePeriodMs;
static bool _previousQuickGyroCalibrationState = false;
// How many milliseconds after startup should we wait before we allow gyro quick calibration to occur
static constexpr uint64_t _kMinMillisAfterFirstStage = 2000;

// If quick gyro calibration is in progress and the user moves the device,
// we will cancel the calibration and blink the LED red to signify that
// the calibration was interrupted
static constexpr uint64_t _quickGyroCalibrationInterruptedLEDFlashIntervalMillis = 100;
static constexpr uint64_t _quickGyroCalibrationInterruptedLEDFlashDurationMillis = 2000;
static uint64_t _quickGyroCalibrationInterruptedTimeMillis = INT64_MIN;
static uint64_t _quickGyroCalibrationInterruptedLEDUpdateTimeMillis = 0;
static bool _quickGyroCalibrationInterruptionLEDState = false;

static void _handleFirstStagePreCalibrationIfNeeded(void)
{
  const uint64_t currentMillis = millis();
  // If we already completed quick calibration, there's no need to proceed any further
  if (_imu->completedQuickCalibration) {
    if (!_previousQuickGyroCalibrationState) {
      _previousQuickGyroCalibrationState = true;
      LOG_INFO("Completed quick gyro calibration");

      // show green LED
      _ledController.showRGB(0, 255, 0);
      // Give the LED a bit of time to display the color
      AsyncController::main.executeAfter(10, []() {
        _ledController.disconnectRMT();
        // Since the first ESC shares the RMT channel with the RGB LED, time to set up the motors now
        _setupMotors();
        _updateArmStatus();
      });
    }
    return;
  }

  // If this code passes it means the quick calibration was interrupted,
  // we should flash the LED red to let the user know
  if (currentMillis - _quickGyroCalibrationInterruptedTimeMillis <
      _quickGyroCalibrationInterruptedLEDFlashDurationMillis) {
    if (currentMillis - _quickGyroCalibrationInterruptedLEDUpdateTimeMillis >
        _quickGyroCalibrationInterruptedLEDFlashIntervalMillis) {
      _quickGyroCalibrationInterruptedLEDUpdateTimeMillis = currentMillis;
      _ledController.showRGB(_quickGyroCalibrationInterruptionLEDState ? 255 : 20, 0, 0);
      _quickGyroCalibrationInterruptionLEDState = !_quickGyroCalibrationInterruptionLEDState;
    }
    return;
  }

  /*
    If this check passes, it means that several conditions have been met and we can proceed
    with a 'quick gyro calibration':
      1. The device has been stationary
      2. The device is upright
      3. It has been at least _kMinMillisAfterFirstStage since the first stage setup completed
      4. The quick gyro calibration is not already in progress
  */
  if (!_imu->isQuickGyroCalibrationInProgress && !_motionDetector.isInMotion && _motionDetector.isUpright &&
      _receivedFirstImuUpdate && _firstStageSetupCompletionTimeMillis > _kMinMillisAfterFirstStage) {
    _imu->beginQuickGyroCalibration();
    LOG_INFO("Beginning quick gyro calibration");
  }  // check if we need to cancel an ongoing quick calib due to device movement
  else if (_imu->isQuickGyroCalibrationInProgress && (_motionDetector.isInMotion || !_motionDetector.isUpright)) {
    LOG_WARN("Quick gyro calibration interrupted");
    _imu->cancelQuickGyroCalibration();
    _quickGyroCalibrationInterruptedTimeMillis = currentMillis;
  } else {
    // While we are waiting for quick gyro calibration - gently flash the white LED
    const float whiteValue = _kBrightnessMidpoint + (sinf((float)currentMillis * _kRadsPerMs) * _kBrightnessScale);
    // Keep within the min and max brightness range
    const float clampedwhiteValue = std::clamp(whiteValue, (float)_kMinWhiteBrightness, (float)_kMaxWhiteBrightness);
    _ledController.showRGB(clampedwhiteValue, clampedwhiteValue, clampedwhiteValue);
  }
}

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
  _barometer.loopHandler();
  _vl53Manager.loopHandler();

  _handleFirstStagePreCalibrationIfNeeded();

  if (_receivedAltitudeUpdate) {
    LOG_INFO_PERIODIC_MILLIS(1000, "Altitude: %fm", _relativeAltitudeMeters);
  }

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

  if (_motorDebugEnabled) {
    LOG_INFO_PERIODIC_MILLIS(1000, "Motor debug mode enabled");
    motor_outputs_t motors = {_motorDebugValues[0], _motorDebugValues[1], _motorDebugValues[2], _motorDebugValues[3]};
    updateMotors(motors);
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
    return;
  }

  if (!_receivedFirstImuUpdate || !_receivedMagUpdate) {
    // Wait until we have both an IMU and magnetometer read before
    // proceeding
    return;
  }

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

  static bool beganRun = false;
  if (!beganRun && _controllerValues.leftStickInput.y < 20.0f) {
    if (_recordDebugData) {
      _helper->saveValues(micros());
    }
    return;
  } else if (!beganRun) {
    beganRun = true;
  }

  motor_outputs_t outputs =
      _controller->calculateOutputs(_pidPreferences->gains, _imuValues, _controllerValues, micros(), _recordDebugData);

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