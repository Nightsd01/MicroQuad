

#include <QuadcopterController.h>
#include <DebugHelper.h>
#include <ParameterProvider.h>
#include <RegisteredParameters.h>

// uses MPU6050 digital motion processor for smooth accelerometer readings
#include <Wire.h>
#include <string>

#include "BLEDevice.h"
#include "BLECharacteristic.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLE2902.h"
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_common_api.h>// ESP32 BLE

// #include "FS.h"
// #include "SD.h"
#include "SPI.h"

#include "Logger.h"

#include "led_strip.h"
#include "driver/rmt.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "IMU.h"

#include <esp_partition.h>
#include "led_controller.h"
#include <Fusion.h>

template<typename T>
class CircularBuffer {
private:
    std::vector<T> buffer;
    std::size_t capacity;
    std::size_t currentPos;
    bool wrapped;

public:
    explicit CircularBuffer(std::size_t size) : capacity(size), currentPos(0), wrapped(false) {
        buffer.resize(capacity);
    }

    void push(const T& value) {
        buffer[currentPos] = value;

        ++currentPos;
        if (currentPos == capacity) {
            wrapped = true;
            currentPos = 0;
        }
    }

    T& operator[](std::size_t index) {
        if (index >= capacity) {
            abort();
        }
        
        if (wrapped) {
            return buffer[(currentPos + index) % capacity];
        } else {
            return buffer[index];
        }
    }

    int size(void) {
      return buffer.size();
    }
};

// static bool _enableEmergencyMode = false;

// LED Setup
// micro whoop
#define LED_DATA_PIN GPIO_NUM_38

// mini quad
// #define LED_DATA_PIN GPIO_NUM_14

LEDController ledController(LED_DATA_PIN);

#define NUM_MOTORS 4

// Accelerometer i2c pin definition
// #define ACCEL_INTERRUPT_PIN 45

// Bluetooth constants
#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CONTROL_CHARACTERISTIC_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define ARM_CHARACTERISTIC_UUID "baf0bcca-634f-11eb-ae93-0242ac130002"
#define TELEM_CHARACTERISTIC_UUID "e35f992e-5e7c-11eb-ae93-0242ac130002"
#define RESET_CHARACTERISTIC_UUID "489d3a76-6fdf-11eb-9439-0242ac130002"
#define MOTOR_DEBUG_CHARACTERISTIC_UUID "dec9fad6-0cf9-11ec-82a8-0242ac130003"
#define CALIBRATION_CHARACTERISTIC_UUID "498e876e-0dd2-11ec-82a8-0242ac130003"
#define DEBUG_CHARACTERISTIC_UUID "f0a0afee-0983-4691-adc5-02ee803f5418"

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

#define DEVICE_MANUFACTURER "BradHesse"
#define DEVICE_NAME "MicroQuad"

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

// mini quad
// #define SPI0_CS_PIN 38
// #define SPI0_MOSI_PIN 36
// #define SPI0_SCLK_PIN 35
// #define SPI0_MISO_PIN 37
// #define IMU_INT_PIN 21

BLECharacteristic *controlCharacteristic;
BLECharacteristic *telemetryCharacteristic;
BLECharacteristic *armCharacteristic;
BLECharacteristic *resetCharacteristic;
BLECharacteristic *motorDebugCharacteristic;
BLECharacteristic *calibrationCharacteristic;
BLECharacteristic *debugCharacteristic;

float yawValue = 0;
float throttleValue = 0;
float pitchValue = 127.50f;
float rollValue = 127.50f;

bool updateParams = false;

bool deviceConnected = false;

bool armed = false;
bool previousArmStatus = false;

long long lastTelemetryUpdateTimeMillis = 0;

motor_outputs_t previousMotorOutputs;
bool setMotorOutputs = false;

const gpio_num_t motorPins[NUM_MOTORS] = {GPIO_NUM_6, GPIO_NUM_5, GPIO_NUM_7, GPIO_NUM_8};

bool resetFlag = false;

#define MPU_6050_I2C_ADDR 0x69

bool motorDebugEnabled = false;
double motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

bool startMonitoringPid = false;

static volatile bool interruptLock = false;

bool sendDebugData = false;
bool recordDebugData = false;

static bool completedFirstArm = false;

// When the quadcopter enters into unacceptable orientation for more than 2 measurements, 
// the drone will cut power to the motors for safety and must be rebooted
static bool enteredEmergencyMode = false;

static IMU *imu = NULL;
static position_values_t previousIMUValues;
static FusionAhrs _fusion;
static FusionOffset _offset;

static volatile bool _calibrate = false;
static volatile CalibrationAxis _calibrationAxis = CalibrationAxis::x;
static volatile int _calibrationValue = 0;


static void updateArmStatus(void) {
  ledController.showRGB(armed ? 255 : 0, armed ? 0 : 255, 0);
  if (armed && !completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    completedFirstArm = true;
    
    for (int i = 0; i < NUM_MOTORS; i++) {
      pinMode(motorPins[i], OUTPUT);
    }
  }
}

std::vector<std::string> split(std::string to_split, std::string delimiter) {
    size_t pos = 0;
    std::vector<std::string> matches{};
    do {
        pos = to_split.find(delimiter);
        int change_end;
        if (pos == std::string::npos) {
            pos = to_split.length() - 1;
            change_end = 1;
        }
        else {
            change_end = 0;
        }
        matches.push_back(to_split.substr(0, pos+change_end));

        to_split.erase(0, pos+1);
    }
    while (!to_split.empty());
    return matches;

}

static bool _didRead = false;

class MessageCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        interruptLock = true;
        if (characteristic->getUUID().equals(controlCharacteristic->getUUID())) {
          std::string data = characteristic->getValue();
          char *str = (char *)malloc((strlen(data.c_str()) * sizeof(char)) + 10);
          strcpy(str, data.c_str());
          char *ptr = strtok(str, ",");

          int i = 0;
          while (ptr != NULL) {
            double val = strtod(ptr, NULL);
            if (i == 0) {
              throttleValue = val;
            } else if (i == 1) {
              yawValue = val;
            } else if (i == 2) {
              pitchValue = val;
            } else {
              rollValue = val;
            }
            i++;
            ptr = strtok(NULL, ",");
          }

          free(str);

          updateParams = true;
        } else if (characteristic->getUUID().equals(armCharacteristic->getUUID())) {
          uint8_t *armData = characteristic->getData();
          armed = armData[0];
          if (armed) {
            // LOG_INFO("ARMED");
          } else {
            // LOG_INFO("DISARMED");
          }
        } else if (characteristic->getUUID().equals(resetCharacteristic->getUUID())) {
          // LOG_INFO("SET RESET FLAG");
          resetFlag = true;
        } else if (characteristic->getUUID().equals(motorDebugCharacteristic->getUUID())) {
          std::string value = characteristic->getValue();
          std::vector<std::string> components = split(value, ":");
          if (components.size() == 0) {
            // LOG_ERROR("ERROR: Incorrect motor debug packet (1)");
            return;
          }
          if (components[0] == "motordebug_enabled") {
            if (components.size() == 1) {
              // LOG_ERROR("ERROR: Incorrect motor debug packet (2)");
              return;
            }
            std::string value = components[1];
            motorDebugEnabled = value == "1";
            // LOG_INFO("Changing motor debug status to %s", motorDebugEnabled ? "enabled" : "disabled");
          } else if (components[0] == "motordebug_value") {
            if (components.size() < 3) {
              // LOG_ERROR("ERROR: Incorrect motor debug packet (3)");
              return;
            }
            int motorNumber = atoi(components[1].c_str()) - 1;
            int motorValue = atoi(components[2].c_str());
            double motorWriteValue = ((((double)motorValue) / 255.0f) * 1000.0f) + 1000.0f;
            // LOG_INFO("Updating motor %i to %f", motorNumber, motorWriteValue);
            motorDebugValues[motorNumber] = motorWriteValue;
          } else {
            // LOG_ERROR("ERROR: Incorrect motor debug packet (4)");
            return;
          }
        } else if (characteristic->getUUID().equals(calibrationCharacteristic->getUUID())) {
          std::string value = characteristic->getValue();
          std::vector<std::string> components = split(value, "=");
          if (components.size() != 2) {
            LOG_ERROR("Invalid calibration command");
            return;
          }
          CalibrationAxis axis;
          if (components[0] == "x") {
            axis = CalibrationAxis::x;
          } else if (components[0] == "y") {
            axis = CalibrationAxis::y;
          } else if (components[0] == "z") {
            axis = CalibrationAxis::z;
          } else {
            LOG_ERROR("Invalid calibration axis");
            return;
          }
          const int val = atoi(components[1].c_str());
          _calibrate = true;
          _calibrationAxis = axis;
          _calibrationValue = val;
        } else if (characteristic->getUUID().equals(debugCharacteristic->getUUID())) {
          if (characteristic->getValue() == "request") {
            // LOG_INFO("Recording debug data transmission");
            sendDebugData = true;
          } else if (characteristic->getValue() == "record:1") {
            // LOG_INFO("Recording debug data start");
            recordDebugData = true;
          } else if (characteristic->getValue() == "record:0") {
            // LOG_INFO("Recording debug data stop");
            recordDebugData = false;
          }
        }
        interruptLock = false;
    }

    void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param)
    {
      if (pCharacteristic->getUUID().toString() == DEBUG_CHARACTERISTIC_UUID) {
        _didRead = true;
      }
    }
};

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server)
    {
        LOG_INFO("Connected");
        deviceConnected = true;
        // Set the MTU size
        esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
        if (status != ESP_OK) {
            LOG_ERROR("set local MTU failed, error code = %x", status);
        }
    };

    void onDisconnect(BLEServer *server)
    {
        LOG_INFO("Disconnected");
        deviceConnected = false;
        armed = false;
        // Set the MTU size
        esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
        if (status != ESP_OK) {
            LOG_ERROR("set local MTU failed, error code = %x", status);
        }
        server->startAdvertising();
    }

    void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param)
    {
      LOG_INFO("MTU changed to: %i", param->mtu);
    }
};

QuadcopterController *controller;
DebugHelper *helper;

#define SD_CS_PIN 10

// Interrupt Detection Routine
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    // Since this is an interrupt, do not put much logic in here.
    mpuInterrupt = true;
}

static void initController()
{
  delete controller;
  controller = new QuadcopterController(
    {
      .yawGains = {
        .proportionalGain = 1.2f,
        .integralGain = 0.0f,
        .derivativeGain = 0.0f,
      },
      .pitchGains = {
        .proportionalGain = 1.8f,
        .integralGain = 0.0f,
        .derivativeGain = 0.0f,
      },
      .rollGains = {
        .proportionalGain = 1.8f,
        .integralGain = 0.0f,
        .derivativeGain = 0.0f,
      },
    },
    helper, 
    micros()
  );
}

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
static const float _accelerometerLowPassAlpha = 0.02f; // lower alpha = more smoothing but more lag
static const float _gyroscopeLowPassAlpha = 0.02f; // lower alpha = more smoothing but more lag
static void _receivedIMUUpdate(imu_update_t update) 
{
  const float deltaTimeSeconds = (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();

  FusionVector gyroscope = {update.gyro_x, update.gyro_y, update.gyro_z}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {update.accel_x, update.accel_y, update.accel_z};

  accelerometer.axis.x = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[0]) + (_accelerometerLowPassAlpha * accelerometer.axis.x);
  accelerometer.axis.y = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[1]) + (_accelerometerLowPassAlpha * accelerometer.axis.y);
  accelerometer.axis.z = ((1.0f - _accelerometerLowPassAlpha) * _previousFilteredAccelValues[2]) + (_accelerometerLowPassAlpha * accelerometer.axis.z);
  gyroscope.axis.x = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[0]) + (_gyroscopeLowPassAlpha * gyroscope.axis.x);
  gyroscope.axis.y = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[1]) + (_gyroscopeLowPassAlpha * gyroscope.axis.y);
  gyroscope.axis.z = ((1.0f - _gyroscopeLowPassAlpha) * _previousFilteredGyroValues[2]) + (_gyroscopeLowPassAlpha * gyroscope.axis.z);
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

  
  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateNoMagnetometer(&_fusion, gyroscope, accelerometer, deltaTimeSeconds);
  _euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&_fusion));
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
    printf("%ld, %f, %f, %f, %f, %d, %d, %d, %f, %f, %f, %f, %f, %f, %i, %i, %i\n", millis(), deltaTimeSeconds, _euler.angle.yaw, _euler.angle.pitch, _euler.angle.roll, update.accel_raw_x, update.accel_raw_y, update.accel_raw_z, update.accel_x, update.accel_y, update.accel_z, update.gyro_x, update.gyro_y, update.gyro_z, (int)update.gyro_raw_x, (int)update.gyro_raw_y, (int)update.gyro_raw_z);
    _previousLogMillis = millis();
  }
}

// 1 - correct
// 2 - wrong
// 3 - wrong
// 4 - correct

// Initial setup
static std::string someStr;
static std::string previousStr;
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

    // Setup BLE Server
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(180);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());

    BLEService *service = server->createService(BLEUUID(std::string(SERVICE_UUID)), 30, 0);

    controlCharacteristic = service->createCharacteristic(CONTROL_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    controlCharacteristic->setCallbacks(new MessageCallbacks());
    controlCharacteristic->addDescriptor(new BLE2902());

    telemetryCharacteristic = service->createCharacteristic(TELEM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    telemetryCharacteristic->setCallbacks(new MessageCallbacks());
    telemetryCharacteristic->addDescriptor(new BLE2902());
    telemetryCharacteristic->setWriteNoResponseProperty(true);

    armCharacteristic = service->createCharacteristic(ARM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    armCharacteristic->setCallbacks(new MessageCallbacks());
    armCharacteristic->addDescriptor(new BLE2902());

    resetCharacteristic = service->createCharacteristic(RESET_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    resetCharacteristic->setCallbacks(new MessageCallbacks());
    resetCharacteristic->addDescriptor(new BLE2902());

    motorDebugCharacteristic = service->createCharacteristic(MOTOR_DEBUG_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    motorDebugCharacteristic->setCallbacks(new MessageCallbacks());
    motorDebugCharacteristic->addDescriptor(new BLE2902());

    calibrationCharacteristic = service->createCharacteristic(CALIBRATION_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    calibrationCharacteristic->setCallbacks(new MessageCallbacks());
    calibrationCharacteristic->addDescriptor(new BLE2902());

    debugCharacteristic = service->createCharacteristic(DEBUG_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
    debugCharacteristic->setCallbacks(new MessageCallbacks());
    debugCharacteristic->addDescriptor(new BLE2902());

    service->start();

    // Set the MTU size
    esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
    if (status != ESP_OK) {
        LOG_ERROR("set local MTU failed, error code = %x", status);
    }

    // Register device info service, that contains the device's UUID, manufacturer and name.
    service = server->createService(DEVINFO_UUID);
    BLECharacteristic *characteristic = service->createCharacteristic(DEVINFO_MANUFACTURER_UUID, BLECharacteristic::PROPERTY_READ);
    characteristic->setValue(DEVICE_MANUFACTURER);
    characteristic = service->createCharacteristic(DEVINFO_NAME_UUID, BLECharacteristic::PROPERTY_READ);
    characteristic->setValue(DEVICE_NAME);
    characteristic = service->createCharacteristic(DEVINFO_SERIAL_UUID, BLECharacteristic::PROPERTY_READ);
    String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
    characteristic->setValue(chipId.c_str());
    service->start();
  

    // Advertise services
    BLEAdvertising *advertisement = server->getAdvertising();
    BLEAdvertisementData adv;
    adv.setName(DEVICE_NAME);
    adv.setCompleteServices(BLEUUID(SERVICE_UUID));
    advertisement->setAdvertisementData(adv);
    advertisement->start();

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

    LOG_INFO("Configuring controller");
    controller->setControlScalingValues({
      .yaw = 180.0f,
      .pitch = 20.0f,
      .roll = 20.0f  
    }, 1.0f);  
    LOG_INFO("Initializing Arming Signal LED");
    ledController.showRGB(0, 255, 0);

    LOG_INFO("SETUP COMPLETE AFTER %lu ms", millis() - initializationTime);

}

void uploadDebugData() {
  LOG_INFO("Begin debug data upload");
  int currentByteIndex = 0;
  const int packetSize = 160;
  uint8_t *data = helper->data;
  uint64_t dataSize = helper->totalDataSize();
  String firstInfo = String("debug:") + String((int)dataSize);
  debugCharacteristic->setValue(firstInfo.c_str());
  debugCharacteristic->notify();
  delay(100);
  while (currentByteIndex < dataSize) {
    if (currentByteIndex + packetSize < dataSize) {
      debugCharacteristic->setValue(&data[currentByteIndex], packetSize);
    } else {
      debugCharacteristic->setValue(&data[currentByteIndex], dataSize - currentByteIndex);
    }
    debugCharacteristic->notify();
    delay(30);
    currentByteIndex += packetSize;
    _didRead = false;
  }

  debugCharacteristic->setValue("==TERMINATE==");
  debugCharacteristic->notify();
  LOG_INFO("End debug data upload");
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
  if (!deviceConnected || millis() - lastTelemetryUpdateTimeMillis < TELEM_UPDATE_INTERVAL_MILLIS) {
    return;
  }
  
  lastTelemetryUpdateTimeMillis = millis();

  String packet = String(armed);
  packet += TELEM_DELIMITER;

  for (int i = 0; i < 4; i++) {
    if (setMotorOutputs) {
      packet += String(previousMotorOutputs.values[i]);
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


  telemetryCharacteristic->setValue(packet.c_str());
  telemetryCharacteristic->notify();
}

void updateMotors(motor_outputs_t outputs) {
  if (!completedFirstArm) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!armed) {
      analogWrite(motorPins[i], 0);
    } else {
      analogWrite(motorPins[i], map(outputs.values[i], 1000, 2000, 0, 255));
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
  if (interruptLock) {
    return;
  }
  if (Serial.available()) { // Check if data is available to read
    char receivedChar = (char)Serial.read(); // Read a byte
    Serial.print("Received: ");
    Serial.println(receivedChar); // Echo the received byte
  }

  if (Serial.available() > 0) {
    String serialString = Serial.readString();
    if (serialString.indexOf("enable_emerg") != -1) {
      LOG_INFO("Updating");
      // _parameterProvider.update(ENABLE_EMERGENCY_MODE_DETECTION, 1);
    }

    if (serialString.indexOf("testing") != -1) {
      LOG_INFO("Updating string");
      // _parameterProvider.update(TEST_VALUE, std::string("some_new_val2"));
    }

    LOG_INFO("Got string: %s", serialString.c_str());
  }

  if (someStr != previousStr) {
    LOG_INFO("static string changed: %s", someStr.c_str());
    previousStr = someStr;
  }

  if (armed != previousArmStatus) {
    LOG_INFO("Updating arm LED");
    updateArmStatus();
    LOG_INFO("Updated arm LED");
    previousArmStatus = armed;
    LOG_INFO("Updated previous arm status");
  }

  if (sendDebugData) {
    LOG_INFO("Beginning debug data transmission");
    sendDebugData = false;
    uploadDebugData();
    LOG_INFO("Completed debug data transmission");
    return;
  }

  if (resetFlag) {
    LOG_INFO("Resetting");
    resetFlag = false;
    initController();
  }
  
  const float batteryVoltage = batteryLevel();
  updateClientTelemetryIfNeeded(batteryVoltage);
  helper->voltage = batteryVoltage;

  if (!startMonitoringPid && throttleValue > 20.0f) {
    startMonitoringPid = true;
    startedMonitoringTimestamp = timestamp;
    controller->startMonitoringPID();
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

  if (!_receivedImuUpdate) {
    return;
  }
  _receivedImuUpdate = false;
  
  //   // need to flip pitch & roll
  //   const float rollVal = ypr[2];
  //   ypr[2] = ypr[1] * -1; // need to flip roll
  //   ypr[1] = rollVal;

    controller_values_t controllerValues = {
      .leftStickInput = {
        .x = yawValue,
        .y = throttleValue
      },
      .rightStickInput = {
        .x = rollValue,
        .y = pitchValue
      }
    };

    // position_values_t positionValues = {
    //   .yaw = ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f,
    //   .pitch = ypr[1] * (180.0f / (M_PI * 2.0)),
    //   .roll = ypr[2] * (180.0f / (M_PI * 2.0))
    // };

    position_values_t positionValues = {
      .yaw = _euler.angle.yaw,
      .pitch = _euler.angle.pitch,
      .roll = _euler.angle.roll
    };

  //   // TODO: Replace with parameter provider check instead of a compiler flag
    #ifdef ENABLE_EMERGENCY_MODE 
      // Check if we need to enter into emergency mode
      if ((fabs(positionValues.pitch) > 80.0f && fabs(previousIMUValues.pitch) > 80.0f) 
          || (fabs(positionValues.roll) > 80.0f && fabs(previousIMUValues.roll) > 80.0f)) {
        // The drone has entered into an unacceptable orientation - this kills the motors
        enteredEmergencyMode = true;
        armed = false;
        updateArmStatus();
        LOG_ERROR("ENTERED EMERGENCY MODE, killing motors: pitch = %f, roll = %f", positionValues.pitch, positionValues.roll);
      }
    #endif

    if (enteredEmergencyMode) {
      updateMotors({.values = {1000.0f, 1000.0f, 1000.0f, 1000.0f}});
      return;
    }

    previousIMUValues = positionValues;

    if (motorDebugEnabled) {
        updateMotors({.values = {
          motorDebugValues[0],
          motorDebugValues[1],
          motorDebugValues[2],
          motorDebugValues[3]
        }});
        return;
    }

    static bool beganRun = false;
    if (!beganRun && throttleValue < 20.0f) {
      return;
    } else if (!beganRun) {
      beganRun = true;
    }

    motor_outputs_t outputs = controller->calculateOutputs(positionValues, controllerValues, micros(), recordDebugData);

    samples++;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      LOG_INFO("%i samples per second", samples);
      LOG_INFO("Throttle = %f", throttleValue);
      samples = 0;
    }

    previousMotorOutputs = outputs;

  //   static bool savedCSVHeader = false;
  //   if (!savedCSVHeader) {
  //     savedCSVHeader = true;
  //     DIAGNOSTICS_SAVE("armed, voltage, time, yaw, pitch, roll, throttle, mot1, mot2, mot3, mot4, memory usage (bytes)");
  //   }

  //   // Save a diagnostics entry to the CSV file on the SD card
  //   DIAGNOSTICS_SAVE("%d, %.03f, %lu, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %lu", armed, batteryVoltage, millis(), positionValues.yaw, positionValues.pitch, positionValues.roll, throttleValue, outputs.values[0], outputs.values[1], outputs.values[2], outputs.values[3], xPortGetFreeHeapSize());

    if (throttleValue < 20) {
      updateMotors({.values = {1000.0f, 1000.0f, 1000.0f, 1000.0f}});
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
    if (recordDebugData && millis() - lastRecordMillis >= 5) {
        helper->saveValues(millis());
        lastRecordMillis = millis();
    }
  // }
}