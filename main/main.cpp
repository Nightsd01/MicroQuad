

#include <QuadcopterController.h>
#include <DebugHelper.h>
#include <RegisteredParameters.h>

#include <Wire.h>
#include <string>

// Magnetometer
#include <DFRobot_QMC5883.h>

#include "SPI.h"

#include "Logger.h"

#include "driver/rmt.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "IMU.h"
#include "BLEController.h"
#include "ElectronicSpeedController.h"

#include <esp_partition.h>
#include "led_controller.h"
#include <Fusion.h>

static std::vector<ElectronicSpeedController> _speedControllers;

// LED Setup
#define LED_DATA_PIN GPIO_NUM_38

LEDController ledController(LED_DATA_PIN);

const gpio_num_t motorPins[NUM_MOTORS] = {GPIO_NUM_6, GPIO_NUM_5, GPIO_NUM_7, GPIO_NUM_8};

#define TELEM_DELIMITER ","
#define TELEM_UPDATE_INTERVAL_MILLIS 100

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

long long lastTelemetryUpdateTimeMillis = 0;

motor_outputs_t previousMotorOutputs;
bool setMotorOutputs = false;

bool _resetFlag = false;

bool _motorDebugEnabled = false;
double _motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

bool startMonitoringPid = false;

bool _sendDebugData = false;
bool _recordDebugData = false;

static bool _completedFirstArm = false;

// When the quadcopter enters into unacceptable orientation for more than 2 measurements, 
// the drone will cut power to the motors for safety and must be rebooted
static bool _enteredEmergencyMode = false;

DFRobot_QMC5883 _compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);

// Keep these 2 in sync
#define MAG_UPDATE_RATE_MILLIS 5
#define MAG_DATARATE QMC5883_DATARATE_200HZ
static uint64_t _lastMagnetometerRead = 0;
static sVector_t _magValues;

static IMU *imu = NULL;
static imu_output_t _imuValues;
static FusionAhrs _fusion;
static FusionOffset _offset;

static volatile bool _calibrate = false;
static volatile CalibrationAxis _calibrationAxis = CalibrationAxis::x;
static volatile int _calibrationValue = 0;


static void updateArmStatus(void) {
  ledController.showRGB(_armed ? 255 : 0, _armed ? 0 : 255, 0);
  if (_armed && !_completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    for (int i = 0; i < NUM_MOTORS; i++) {
      esp_err_t err = NULL;
      ElectronicSpeedController controller = ElectronicSpeedController(
        motorPins[i] /* pin */,
        i /* channel */, 
        &err /* error */
      );
      if (err != ESP_OK) {
        LOG_ERROR("Failed to initialize motor %i", i);
        break;
      }
      _speedControllers.push_back(controller);
    }
  }

  _completedFirstArm = _speedControllers.size() == NUM_MOTORS;
}

QuadcopterController *controller;
DebugHelper *helper;

static void initController()
{
  delete controller;
  controller = new QuadcopterController({
    .angleGains = {
      { // yaw
        .kP = 3.0f,
        .kI = 0.005f,
        .kD = 0.0f
      },
      { // pitch
        .kP = 4.0f,
        .kI = 0.01f,
        .kD = 0.0f
      },
      { // roll
        .kP = 4.0f,
        .kI = 0.01f,
        .kD = 0.0f
      }
    },
    .rateGains = {
      { // yaw
        .kP = 0.7f,
        .kI = 0.005f,
        .kD = 0.0001f
      },
      { // pitch
        .kP = 0.9f,
        .kI = 0.01f,
        .kD = 0.0001f
      },
      { // roll
        .kP = 0.9f,
        .kI = 0.01f,
        .kD = 0.0001f
      }
    }
  }, helper, micros());
}

static controller_values_t _controllerValues = {
  .leftStickInput = {
    .x = INPUT_MAX_CONTROLLER_INPUT / 2.0, 
    .y = 0.0f
  },
  .rightStickInput = {
    .x = INPUT_MAX_CONTROLLER_INPUT / 2.0,
    .y = INPUT_MAX_CONTROLLER_INPUT / 2.0
  }
};

static FusionEuler _euler;
static uint64_t _previousMicros = 0;
static uint64_t _previousLogMillis = 0;
static const FusionMatrix gyroscopeMisalignment = {0.7071f, -0.7071f, 0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f, 0.0f, 1.0f};
static const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
static const FusionMatrix accelerometerMisalignment = {0.7071f, -0.7071f, 0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f, 0.0f, 1.0f};
static const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
static const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
static bool _receivedImuUpdate = false;
static bool _gotFirstIMUUpdate = false;

static float _previousFilteredAccelValues[3] = {0.0f, 0.0f, 1.0f};
static float _previousFilteredGyroValues[3] = {0.0f, 0.0f, 0.0f};
static const float _accelerometerLowPassAlpha = 0.01f; // lower alpha = more smoothing but more lag
static const float _gyroscopeLowPassAlpha = 0.2f; // lower alpha = more smoothing but more lag
static void _receivedIMUUpdate(imu_update_t update) 
{
  const float deltaTimeSeconds = (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();

  FusionVector gyroscope = {update.gyro_x, update.gyro_y, update.gyro_z}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {update.accel_x, update.accel_y, update.accel_z};

  accelerometer.axis.x = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[0]) + (_accelerometerLowPassAlpha * accelerometer.axis.x);
  accelerometer.axis.y = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[1]) + (_accelerometerLowPassAlpha * accelerometer.axis.y);
  accelerometer.axis.z = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[2]) + (_accelerometerLowPassAlpha * accelerometer.axis.z);
  _previousFilteredAccelValues[0] = accelerometer.axis.x;
  _previousFilteredAccelValues[1] = accelerometer.axis.y;
  _previousFilteredAccelValues[2] = accelerometer.axis.z;
  gyroscope.axis.x = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[0]) + (_gyroscopeLowPassAlpha * gyroscope.axis.x);
  gyroscope.axis.y = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[1]) + (_gyroscopeLowPassAlpha * gyroscope.axis.y);
  gyroscope.axis.z = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[2]) + (_gyroscopeLowPassAlpha * gyroscope.axis.z);
  _previousFilteredGyroValues[0] = gyroscope.axis.x;
  _previousFilteredGyroValues[1] = gyroscope.axis.y;
  _previousFilteredGyroValues[2] = gyroscope.axis.z;
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

  
  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateExternalHeading(&_fusion, gyroscope, accelerometer, _magValues.HeadingDegress, deltaTimeSeconds);
  _euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&_fusion));

  _imuValues = {
    .gyroOutput = {gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z},
    .accelOutput = {_euler.angle.yaw, _euler.angle.pitch, _euler.angle.roll}
  };

  _receivedImuUpdate = true;
  _gotFirstIMUUpdate = true;

  helper->accelRaw[0] = accelerometer.axis.x;
  helper->accelRaw[1] = accelerometer.axis.y;
  helper->accelRaw[2] = accelerometer.axis.z;
  helper->gyroRaw[0] = gyroscope.axis.x;
  helper->gyroRaw[1] = gyroscope.axis.y;
  helper->gyroRaw[2] = gyroscope.axis.z;

  // Log to console 10x per second
  if (millis() - _previousLogMillis > 100) {
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&_fusion);
    LOG_INFO(
      "%ld, %f, %f, %f, %f, %d, %d, %d, %f, %f, %f, %f, %f, %f, %i, %i, %i, %f, %f, %f, %f, %f\n", 
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
      _controllerValues.rightStickInput.y
    );
    _previousLogMillis = millis();
  }
}

// Initial setup
void setup() {
    esp_partition_type_t type;
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
        .leftStickInput = {
          .x = update.yaw,
          .y = update.throttle
        },
        .rightStickInput = {
          .x = update.pitch,
          .y = update.roll
        }
      };
    });

    _bluetoothController.setArmStatusUpdateHandler([&](bool armStatus) {
      _armed = armStatus;
    });

    _bluetoothController.setResetStatusUpdateHandler([&]() {
      _resetFlag = true;
    });

    _bluetoothController.setMotorDebugEnabledUpdateHandler([&](bool motorDebug) {
      _motorDebugEnabled = motorDebug;
    });

    _bluetoothController.setMotorDebugUpdateHandler([&](motor_debug_update_t motorDebugUpdate) {
      _motorDebugValues[motorDebugUpdate.motorNum] = motorDebugUpdate.motorWriteValue;
    });

    _bluetoothController.setCalibrationUpdateHandler([&](calibration_update_t calibrationUpdate) {
      _calibrate = true;
      _calibrationAxis = calibrationUpdate.axis;
      _calibrationValue = calibrationUpdate.calibrationValue;
    });

    _bluetoothController.setDebugDataUpdateHandler([&](debug_recording_update_t debugDataUpdate) {
      _sendDebugData = debugDataUpdate.sendDebugData;
      _recordDebugData = debugDataUpdate.recordDebugData;
    });

    // Setup BLE Server
    _bluetoothController.beginBluetooth();
    
    LOG_INFO("Configuring magnetometer");

    while (!_compass.begin()) {
      LOG_ERROR("Failed to initialize magnetometer");
      delay(500);
    }
    /**
     * @brief  Set declination angle on your location and fix heading
     * @n      You can find your declination on: http://magnetic-declination.com/
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
    imu = new IMU(SPI0_CS_PIN, SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_SCLK_PIN, IMU_INT_PIN, [&](imu_update_t update) {
      _receivedIMUUpdate(update);
    }, &setupSuccess);

    if (!setupSuccess) {
      LOG_ERROR("IMU Setup Failure");
    } else {
      LOG_INFO("Successfully set up IMU");
    }

    while (!setupSuccess) {};

    LOG_INFO("Initializing Arming Signal LED");
    ledController.showRGB(0, 255, 0);

    LOG_INFO("SETUP COMPLETE AFTER %lu ms", millis() - initializationTime);

}

void uploadDebugData() {
  uint8_t *data = helper->data;
  uint64_t dataSize = helper->totalDataSize();
  _bluetoothController.uploadDebugData(data, dataSize);
}

float batteryPercent(float batteryVoltage) {
  const float averageVoltagePerCell = batteryVoltage / (float)NUM_BATTERY_CELLS;
  const float cellRange = BATTERY_CELL_MAX_VOLTAGE - BATTERY_CELL_MIN_VOLTAGE;
  return (averageVoltagePerCell - BATTERY_CELL_MIN_VOLTAGE) / cellRange;
}

float batteryLevel() {
    int val = analogRead(BATTERY_SENSE_PIN);
    // int val = 0.0f;
    return BATTERY_SCALE * (float)val;
}

void updateClientTelemetryIfNeeded(float batVoltage) {
  if (!_bluetoothController.isConnected || millis() - lastTelemetryUpdateTimeMillis < TELEM_UPDATE_INTERVAL_MILLIS) {
    return;
  }
  
  lastTelemetryUpdateTimeMillis = millis();

  String packet = String(_armed);
  packet += TELEM_DELIMITER;

  for (int i = 0; i < 4; i++) {
    if (setMotorOutputs) {
      packet += String(previousMotorOutputs[i]);
      packet += TELEM_DELIMITER;
    } else {
      packet += "0";
      packet += TELEM_DELIMITER;
    }
  }

  packet += String(batVoltage);
  packet += TELEM_DELIMITER;
  packet += String(batteryPercent(batVoltage));

  if (_gotFirstIMUUpdate) {
    packet += TELEM_DELIMITER;
    packet += String(_euler.angle.yaw);
    packet += TELEM_DELIMITER;
    packet += String(_euler.angle.pitch);
    packet += TELEM_DELIMITER;
    packet += String(_euler.angle.roll);
  }

  _bluetoothController.sendTelemetryUpdate(packet);
}

void updateMotors(motor_outputs_t outputs) {
  if (!_completedFirstArm) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!_armed) {
      _speedControllers[i].setMicrosecondsPulses(1000);
    } else {
      _speedControllers[i].setMicrosecondsPulses(outputs[i]);
    }
  }
}

unsigned long startedMonitoringTimestamp = 0;

static int samples = 0;
static int lastPrint = 0;

#define NUM_SAMPLES_PER_AVG 40

void loop() {
  TIMERG0.wdtwprotect.val=0x50D83AA1;
  TIMERG0.wdtfeed.val=1;
  TIMERG0.wdtwprotect.val=0;
  unsigned long timestamp = micros();
  if (_bluetoothController.isProcessingBluetoothTransaction) {
    return;
  }

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

  if (millis() - _lastMagnetometerRead > MAG_UPDATE_RATE_MILLIS) {
    _magValues = _compass.readRaw();
    _compass.getHeadingDegrees();
    _lastMagnetometerRead = millis();
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
    imu->calibrate(_calibrationAxis, _calibrationValue);
    char *axisStr = "";
    switch (_calibrationAxis) {
      case CalibrationAxis::x: 
        axisStr = "x";
        break;
      case CalibrationAxis::y: 
        axisStr = "y";
        break;
      case CalibrationAxis::z: 
        axisStr = "z";
        break;
    }
    LOG_INFO("Calibrating %s axis to %i", axisStr, _calibrationValue);
  }

  if (!_receivedImuUpdate || _lastMagnetometerRead == 0) {
    // Wait until we have both an IMU and magnetometer read before proceeding
    return;
  }
  _receivedImuUpdate = false;
  
  //   // need to flip pitch & roll
  //   const float rollVal = ypr[2];
  //   ypr[2] = ypr[1] * -1; // need to flip roll
  //   ypr[1] = rollVal;

  if (!startMonitoringPid) {
    return;
  }

    // imu_output_t positionValues = {
    //   .yaw = ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f,
    //   .pitch = ypr[1] * (180.0f / (M_PI * 2.0)),
    //   .roll = ypr[2] * (180.0f / (M_PI * 2.0))
    // };

  //   // TODO: Replace with parameter provider check instead of a compiler flag
    #ifdef ENABLE_EMERGENCY_MODE 
      // Check if we need to enter into emergency mode
      if (fabs(_euler.angle.pitch) > 80.0f || fabs(_euler.angle.roll) > 80.0f) {
        // The drone has entered into an unacceptable orientation - this kills the motors
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
        motor_outputs_t motors = {
          _motorDebugValues[0],
          _motorDebugValues[1],
          _motorDebugValues[2],
          _motorDebugValues[3]
        };
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
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      LOG_INFO("%i samples per second", samples);
      LOG_INFO("Throttle = %f, motors %f, %f, %f, %f", _controllerValues.leftStickInput.y, outputs[0], outputs[1], outputs[2], outputs[3]);
      samples = 0;
    }

    previousMotorOutputs = outputs;

  //   static bool savedCSVHeader = false;
  //   if (!savedCSVHeader) {
  //     savedCSVHeader = true;
  //     DIAGNOSTICS_SAVE("_armed, voltage, time, yaw, pitch, roll, throttle, mot1, mot2, mot3, mot4, memory usage (bytes)");
  //   }

  //   // Save a diagnostics entry to the CSV file on the SD card
  //   DIAGNOSTICS_SAVE("%d, %.03f, %lu, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %lu", _armed, batteryVoltage, millis(), positionValues.yaw, positionValues.pitch, positionValues.roll, _throttleValue, outputs.values[0], outputs.values[1], outputs.values[2], outputs.values[3], xPortGetFreeHeapSize());

    if (_controllerValues.leftStickInput.y < 20) {
      motor_outputs_t motors = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
      updateMotors(motors);
    } else {
      // Update motors only at 300hz
      static unsigned long lastMotorUpdateMillis = 0;
      if (millis() - lastMotorUpdateMillis >= 3) {
        lastMotorUpdateMillis = millis();
        updateMotors(outputs);
      }
    }
    setMotorOutputs = true;

    static unsigned long lastRecordMillis = millis();
    if (_recordDebugData && millis() - lastRecordMillis >= 5) {
        helper->saveValues(millis());
        lastRecordMillis = millis();
    }
  // }
}