#include <QuadcopterController.h>
#include <DebugHelper.h>

// uses MPU6050 digital motion processor for smooth accelerometer readings
#include <I2Cdev.h>
#include <Wire.h>
#include <string>

#include "BLEDevice.h"
#include "BLECharacteristic.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLE2902.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Logger.h"

#include <ESP32_Servo.h> 

#include "led_strip.h"
#include "driver/rmt.h"
#include "esp_system.h"
#include "esp_log.h"

#include <FastLED.h>

#define ENABLE_EMERGENCY_MODE 1

// LED Setup
#define NUM_LEDS 1
#define LED_DATA_PIN 32
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];

#define NUM_MOTORS 4

// Accelerometer i2c pin definition
#define ACCEL_INTERRUPT_PIN 35

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
#define BATTERY_SENSE_PIN 34
#define NUM_BATTERY_CELLS 2.0f
#define BATTERY_CELL_MAX_VOLTAGE 4.2f
#define BATTERY_CELL_MIN_VOLTAGE 3.3f
#define BATTERY_SCALE 0.001777081468218f

#define MIN_THROTTLE 1015

#define ARM_MICROSECONDS 1000.0f

BLECharacteristic *controlCharacteristic;
BLECharacteristic *telemetryCharacteristic;
BLECharacteristic *armCharacteristic;
BLECharacteristic *resetCharacteristic;
BLECharacteristic *motorDebugCharacteristic;
BLECharacteristic *calibrationCharacteristic;
BLECharacteristic *debugCharacteristic;

float yawValue = 0;
float throttleValue = 0;
float pitchValue = 0;
float rollValue = 0;

bool updateParams = false;

bool deviceConnected = false;

bool armed = false;
bool previousArmStatus = false;

long long lastTelemetryUpdateTimeMillis = 0;

motor_outputs_t previousMotorOutputs;
bool setMotorOutputs = false;

const gpio_num_t motorPins[NUM_MOTORS] = {GPIO_NUM_33, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25};

bool resetFlag = false;

#define MPU_6050_I2C_ADDR 0x69

bool motorDebugEnabled = false;
double motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

bool startMonitoringPid = false;

static volatile bool interruptLock = false;

bool sendDebugData = false;
bool recordDebugData = false;

Servo motors[NUM_MOTORS];

static bool completedFirstArm = false;

// When the quadcopter enters into unacceptable orientation for more than 2 measurements, 
// the drone will cut power to the motors for safety and must be rebooted
static bool enteredEmergencyMode = false;
static position_values_t previousIMUValues;

static void updateArmStatus(void) {
  FastLED.clear();
  leds[0] = (armed ? CRGB::Green : CRGB::Red);
  FastLED.show();
  if (armed && !completedFirstArm) {
    LOG_INFO("Setting up motor outputs");
    completedFirstArm = true;
    
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].attach(motorPins[i]);
      motors[i].write(0);
    }
    delay(2000);
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
            LOG_INFO("ARMED");
          } else {
            LOG_INFO("DISARMED");
          }
        } else if (characteristic->getUUID().equals(resetCharacteristic->getUUID())) {
          LOG_INFO("SET RESET FLAG");
          resetFlag = true;
        } else if (characteristic->getUUID().equals(motorDebugCharacteristic->getUUID())) {
          std::string value = characteristic->getValue();
          std::vector<std::string> components = split(value, ":");
          if (components.size() == 0) {
            LOG_ERROR("ERROR: Incorrect motor debug packet (1)");
            return;
          }
          if (components[0] == "motordebug_enabled") {
            if (components.size() == 1) {
              LOG_ERROR("ERROR: Incorrect motor debug packet (2)");
              return;
            }
            std::string value = components[1];
            motorDebugEnabled = value == "1";
            LOG_INFO("Changing motor debug status to %s", motorDebugEnabled ? "enabled" : "disabled");
          } else if (components[0] == "motordebug_value") {
            if (components.size() < 3) {
              LOG_ERROR("ERROR: Incorrect motor debug packet (3)");
              return;
            }
            int motorNumber = atoi(components[1].c_str()) - 1;
            int motorValue = atoi(components[2].c_str());
            double motorWriteValue = ((((double)motorValue) / 255.0f) * 1000.0f) + 1000.0f;
            LOG_INFO("Updating motor %i to %f", motorNumber, motorWriteValue);
            motorDebugValues[motorNumber] = motorWriteValue;
          } else {
            LOG_ERROR("ERROR: Incorrect motor debug packet (4)");
            return;
          }
        } else if (characteristic->getUUID().equals(calibrationCharacteristic->getUUID())) {
          LOG_INFO("CALIBRATING");
          // ESP_LOGI(TAG, "Calibrating MPU");
          // mpud::raw_axes_t accelBias, gyroBias;
          // ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
          // ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
          // ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));
          // calibrateMPU();
        } else if (characteristic->getUUID().equals(debugCharacteristic->getUUID())) {
          if (characteristic->getValue() == "request") {
            LOG_INFO("Recording debug data transmission");
            sendDebugData = true;
          } else if (characteristic->getValue() == "record:1") {
            LOG_INFO("Recording debug data start");
            recordDebugData = true;
          } else if (characteristic->getValue() == "record:0") {
            LOG_INFO("Recording debug data stop");
            recordDebugData = false;
          }
        }
        interruptLock = false;
    }
};

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server)
    {
        LOG_INFO("Connected");
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *server)
    {
        LOG_INFO("Disconnected");
        deviceConnected = false;
        armed = false;
    }
};

MPU6050 mpu(MPU_6050_I2C_ADDR);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[512]; // FIFO storage buffer

VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion quat;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

QuadcopterController *controller;
DebugHelper *helper;

#define SD_CS_PIN 12
#define DEBUG_LOG_PATH "/logs"
#define DIAGNOSTICS_PATH "/diags"

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
        .proportionalGain = 0.0f,
        .integralGain = 0.0f,
        .derivativeGain = 0.0f,
      },
      .pitchGains = {
        .proportionalGain = 0.1f,
        .integralGain = 0.001f,
        .derivativeGain = 0.01f,
      },
      .rollGains = {
        .proportionalGain = 0.1f,
        .integralGain = 0.001f,
        .derivativeGain = 0.01f,
      },
    },
    helper, 
    0
  );
}

// 1 - correct
// 2 - wrong
// 3 - wrong
// 4 - correct

// Initial setup
void setup() {
    const unsigned long initializationTime = millis();
    Serial.begin(115200);
    LOG_INFO("BEGAN APP");

    LOG_INFO("Initializing SD diagnostics");

    DebugUtilityInitialize(DEBUG_LOG_PATH, DIAGNOSTICS_PATH, SD_CS_PIN, LogLevel::verbose);

    initController();

    Wire.begin();
    Wire.setClock(400000);

    // Wait for serial to become available
    while (!Serial);

    LOG_INFO("Initializing Arming Signal LED");
    FastLED.addLeds<WS2812B, LED_DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
    FastLED.clear();
    leds[0] = CRGB::Green;
    FastLED.show();

    LOG_INFO("Initializing bluetooth connection");

    // Setup BLE Server
    BLEDevice::init(DEVICE_NAME);
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

    // initialize device
    LOG_INFO(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(ACCEL_INTERRUPT_PIN, INPUT);
    
    // verify connection
    LOG_INFO(F("Testing device connections..."));
    const bool initialized = mpu.testConnection();
    initialized ? LOG_INFO("MPU6050 connection successful") : LOG_ERROR("MPU6050 connection failed");

    updateArmStatus();

    if (!initialized) {
      return;
    }

    LOG_INFO("Initializing DMP");
    uint8_t dmpStatus = mpu.dmpInitialize();
    LOG_INFO("Initialized DMP");

    if (dmpStatus == 0) {
      LOG_INFO(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(ACCEL_INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      LOG_INFO("Supplying DMP offsets");
      
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(3898);
      mpu.setYAccelOffset(971);
      mpu.setZAccelOffset(1402);
      mpu.setXGyroOffset(55);
      mpu.setYGyroOffset(21);
      mpu.setZGyroOffset(-17);
    } else {
      LOG_ERROR("Accelerometer DMP initialization failed with status code = %i", dmpStatus);
    }

    LOG_INFO("Configuring controller");
    controller->setControlScalingValues({
      .yaw = 180.0f,
      .pitch = 20.0f,
      .roll = 20.0f  
    }, 1.0f);  

    LOG_INFO("SETUP COMPLETE");
    LOG_INFO("SETUP COMPLETE AFTER %lu ms", millis() - initializationTime);

    while (true) {
      loop();
    }
}

void uploadDebugData() {
  int currentByteIndex = 0;
  const int packetSize = 20;
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
  }
}

float batteryPercent(float batteryVoltage) {
  const float averageVoltagePerCell = batteryVoltage / (float)NUM_BATTERY_CELLS;
  const float cellRange = BATTERY_CELL_MAX_VOLTAGE - BATTERY_CELL_MIN_VOLTAGE;
  return (averageVoltagePerCell - BATTERY_CELL_MIN_VOLTAGE) / cellRange;
}

float batteryLevel() {
    int val = analogRead(BATTERY_SENSE_PIN);
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

  telemetryCharacteristic->setValue(packet.c_str());
  telemetryCharacteristic->notify();
}

void updateMotors(motor_outputs_t outputs) {
  if (!completedFirstArm) {
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!armed) {
      motors[i].write(map(ARM_MICROSECONDS, 1000.0f, 2000.0f, 0, 180));
    } else {
      motors[i].write(map(outputs.values[i], 1000, 2000, 0, 180));
    }
  }
}

unsigned long startedMonitoringTimestamp = 0;

static int samples = 0;
static int lastPrint = 0;

#define NUM_SAMPLES_PER_AVG 40

void loop() {
  unsigned long timestamp = micros();
  if (interruptLock) {
    return;
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

  if (!startMonitoringPid && throttleValue > 20.0f) {
    startMonitoringPid = true;
    startedMonitoringTimestamp = timestamp;
    controller->startMonitoringPID();
  }
  
  if (!dmpReady) {
    LOG_ERROR("DMP NOT READY");
    return;
  };

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if (fifoCount < packetSize) {
    return;
  }

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount > 1024) {
    mpu.resetFIFO();
    LOG_ERROR("MPU6050 FIFO overflow!");
    return;
  }

  if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // NOTE TO SELF:
    // The pitch and roll axes are flipped
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quat);
    mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);

    // need to flip pitch & roll
    const float rollVal = ypr[2];
    ypr[2] = ypr[1] * -1; // need to flip roll
    ypr[1] = rollVal;

    if (!updateParams) {
      return;
    }

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

    position_values_t positionValues = {
      .yaw = ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f,
      .pitch = ypr[1] * (180.0f / (M_PI * 2.0)),
      .roll = ypr[2] * (180.0f / (M_PI * 2.0))
    };

    // TODO: Replace with parameter provider check instead of a compiler flag
    #ifdef ENABLE_EMERGENCY_MODE 
      // Check if we need to enter into emergency mode
      if ((fabs(positionValues.pitch) > 40.0f && fabs(previousIMUValues.pitch) > 40.0f) 
          || (fabs(positionValues.roll) > 40.0f && fabs(previousIMUValues.roll) > 40.0f)) {
        // The drone has entered into an unacceptable orientation - this kills the motors
        enteredEmergencyMode = true;
        LOG_ERROR("ENTERED EMERGENCY MODE, killing motors: pitch = %f, roll = %f", positionValues.pitch, positionValues.roll);
      }
    #endif

    previousIMUValues = positionValues;

    motor_outputs_t outputs = controller->calculateOutputs(positionValues, controllerValues, timestamp - startedMonitoringTimestamp, recordDebugData);

    samples++;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      LOG_INFO("%i samples per second", samples);
      samples = 0;
    }

    previousMotorOutputs = outputs;

    static bool savedCSVHeader = false;
    if (!savedCSVHeader) {
      savedCSVHeader = true;
      DIAGNOSTICS_SAVE("armed, voltage, time, yaw, pitch, roll, throttle, mot1, mot2, mot3, mot4, memory usage (bytes)");
    }

    // Save a diagnostics entry to the CSV file on the SD card
    DIAGNOSTICS_SAVE("%d, %.03f, %lu, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %lu", armed, batteryVoltage, millis(), positionValues.yaw, positionValues.pitch, positionValues.roll, throttleValue, outputs.values[0], outputs.values[1], outputs.values[2], outputs.values[3], xPortGetFreeHeapSize());

    if (motorDebugEnabled) {
        updateMotors({.values = {
          motorDebugValues[0],
          motorDebugValues[1],
          motorDebugValues[2],
          motorDebugValues[3]
        }});
    } else if (throttleValue < 20 || enteredEmergencyMode) {
      updateMotors({.values = {1000.0f, 1000.0f, 1000.0f, 1000.0f}});
    } else {
      updateMotors(outputs);
    }
    setMotorOutputs = true;
  }
}