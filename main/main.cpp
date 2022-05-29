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

#include "diagnostics.h"

#include "servoControl.h"

#include "led_strip.h"
#include "driver/rmt.h"
#include "esp_system.h"
#include "esp_log.h"

#include <FastLED.h>

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

long long lastTelemetryUpdateTimeMillis = 0;

motor_outputs_t previousMotorOutputs;
bool setMotorOutputs = false;

const gpio_num_t motorPins[NUM_MOTORS] = {GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27};

bool resetFlag = false;

#define MPU_6050_I2C_ADDR 0x69

bool motorDebugEnabled = false;
double motorDebugValues[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
ledc_channel_t motorChannels[4] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3}; 
ledc_timer_t motorTimers[4] = {LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3};

bool startMonitoringPid = false;

static volatile bool interruptLock = false;

bool sendDebugData = false;
bool recordDebugData = false;

servoControl motors[NUM_MOTORS];

static void updateArmStatus(void) {
  FastLED.clear();
  leds[0] = (armed ? CRGB::Green : CRGB::Red);
  FastLED.show();
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
            Serial.println("ARMED");
          } else {
            Serial.println("DISARMED");
          }
          updateArmStatus();
        } else if (characteristic->getUUID().equals(resetCharacteristic->getUUID())) {
          Serial.println("SET RESET FLAG");
          resetFlag = true;
        } else if (characteristic->getUUID().equals(motorDebugCharacteristic->getUUID())) {
          std::string value = characteristic->getValue();
          std::vector<std::string> components = split(value, ":");
          if (components.size() == 0) {
            Serial.println("ERROR: Incorrect motor debug packet (1)");
            return;
          }
          if (components[0] == "motordebug_enabled") {
            if (components.size() == 1) {
              Serial.println("ERROR: Incorrect motor debug packet (2)");
              return;
            }
            std::string value = components[1];
            motorDebugEnabled = value == "1";
            Serial.println("Changing motor debug status to " + String(motorDebugEnabled ? "enabled" : "disabled"));
          } else if (components[0] == "motordebug_value") {
            if (components.size() < 3) {
              Serial.println("ERROR: Incorrect motor debug packet (3)");
              return;
            }
            int motorNumber = atoi(components[1].c_str()) - 1;
            int motorValue = atoi(components[2].c_str());
            double motorWriteValue = ((((double)motorValue) / 255.0f) * 1000.0f) + 1000.0f;
            Serial.println("Updating motor " + String(motorNumber) + " to " + String(motorWriteValue));
            motorDebugValues[motorNumber] = motorWriteValue;
          } else {
            Serial.println("ERROR: Incorrect motor debug packet (4)");
            return;
          }
        } else if (characteristic->getUUID().equals(calibrationCharacteristic->getUUID())) {
          Serial.println("CALIBRATING");
          // ESP_LOGI(TAG, "Calibrating MPU");
          // mpud::raw_axes_t accelBias, gyroBias;
          // ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
          // ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
          // ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));
          // calibrateMPU();
        } else if (characteristic->getUUID().equals(debugCharacteristic->getUUID())) {
          if (characteristic->getValue() == "request") {
            Serial.println("Recording debug data transmission");
            sendDebugData = true;
          } else if (characteristic->getValue() == "record:1") {
            Serial.println("Recording debug data start");
            recordDebugData = true;
          } else if (characteristic->getValue() == "record:0") {
            Serial.println("Recording debug data stop");
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
        Serial.println("Connected");
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *server)
    {
        Serial.println("Disconnected");
        deviceConnected = false;
        armed = false;
        updateArmStatus();
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
DiagnosticsWriter *diags;

#define SD_CS_PIN 12
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
        .proportionalGain = 1.0f,
        .integralGain = 0.001f,
        .derivativeGain = 0.1f,
      },
      .rollGains = {
        .proportionalGain = 1.0f,
        .integralGain = 0.001f,
        .derivativeGain = 0.1f,
      },
    },
    helper, 
    0
  );
}

// 1 - wrong
// 2 - correct
// 3 - correct
// 4 - wrong

// Initial setup
void setup() {
    const unsigned long initializationTime = millis();
    Serial.begin(115200);
    Serial.println("BEGAN APP");

    initController();

    Serial.println("Initializing Arming Signal LED");
    FastLED.addLeds<WS2812B, LED_DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
    leds[0] = CRGB::Green;

    Serial.println("Setting up motor outputs");
    
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].attach(motorPins[i], 1000, 2000, motorChannels[i], motorTimers[i]);
      motors[i].write(91);
    }
    delay(3000);

    Wire.begin();
    Wire.setClock(400000);

    // Wait for serial to become available
    while (!Serial);

    if(!SD.begin(12)){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }
    helper = new DebugHelper();

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    Serial.println("Initializing bluetooth connection");

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
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(ACCEL_INTERRUPT_PIN, INPUT);
    
    // verify connection
    const bool initialized = mpu.testConnection();
    Serial.println(F("Testing device connections..."));
    Serial.println(initialized ? "MPU6050 connection successful" : "MPU6050 connection failed");

    if (!initialized) {
      return;
    }

    Serial.println("Initializing DMP");
    uint8_t dmpStatus = mpu.dmpInitialize();
    Serial.println("Initialized DMP");

    if (dmpStatus == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(ACCEL_INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      Serial.println("Supplying DMP offsets");
      
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(91);
      mpu.setYGyroOffset(61);
      mpu.setZGyroOffset(-5);
      mpu.setZAccelOffset(2020); // 1688 factory default for my test chip
      mpu.setXAccelOffset(1891);
      mpu.setYAccelOffset(-3958);
    } else {
      Serial.println("Accelerometer DMP initialization failed with status code = " + String(dmpStatus));
    }

    Serial.println("Configuring controller");
    controller->setControlScalingValues({
      .yaw = 180.0f,
      .pitch = 20.0f,
      .roll = 20.0f  
    }, 1.0f);  

    Serial.println("Initializing SD diagnostics");
    diags = new DiagnosticsWriter(DIAGNOSTICS_PATH, SD_CS_PIN);

    Serial.println("SETUP COMPLETE");
    diags->writeDiagnostics("SETUP COMPLETE AFTER " + String(millis() - initializationTime) + " ms");

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

void updateClientTelemetryIfNeeded() {
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

  const float batVoltage = batteryLevel();
  packet += String(batVoltage);
  packet += TELEM_DELIMITER;
  packet += String(batteryPercent(batVoltage));

  telemetryCharacteristic->setValue(packet.c_str());
  telemetryCharacteristic->notify();
}

void updateMotors(motor_outputs_t outputs) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!armed) {
      motors[i].write(map(ARM_MICROSECONDS, 1000, 2000, 0, 180));
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
  if (sendDebugData) {
    Serial.println("Beginning debug data transmission");
    sendDebugData = false;
    uploadDebugData();
    Serial.println("Completed debug data transmission");
    return;
  }

  if (resetFlag) {
    Serial.println("Resetting");
    resetFlag = false;
    initController();
  }
  
  updateClientTelemetryIfNeeded();

  if (!startMonitoringPid && throttleValue > 20.0f) {
    startMonitoringPid = true;
    startedMonitoringTimestamp = timestamp;
    controller->startMonitoringPID();
  }
  
  if (!dmpReady) {
    Serial.println("DMP NOT READY");
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
    Serial.println("MPU6050 FIFO overflow!");
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

//    Serial.println("Accel yaw = " + String(ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f) + ", pitch = " + String(ypr[1] * (180.0f / (M_PI * 2.0)) + 90.0f) + ", roll = " + String(ypr[2] * (180.0f / (M_PI * 2.0)) + 90.0f));

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
      .pitch = ypr[2] * (180.0f / (M_PI * 2.0)) + 90.0f,
      .roll = ypr[1] * (180.0f / (M_PI * 2.0)) + 90.0f
    };




    motor_outputs_t outputs = controller->calculateOutputs(positionValues, controllerValues, timestamp - startedMonitoringTimestamp, recordDebugData);

    samples++;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.println(String(samples) + " samples per second");
      samples = 0;
    }

    previousMotorOutputs = outputs;
    
    static uint64_t lastUpdateTs = 0;

    if (millis() - lastUpdateTs > 50) {
      lastUpdateTs = millis();
      diags->writeDiagnostics("yaw = " + String(ypr[0]) + ", pitch = " + String(ypr[1]) + ", roll = " + String(ypr[2]));
      diags->writeDiagnostics("throttle = " + String(throttleValue) + ", 1 = " + String(outputs.values[0]) + ", 2 = " + String(outputs.values[1]) + ", 3 = " + String(outputs.values[2]) + ", 4 = " + String(outputs.values[3]));
    }


    if (motorDebugEnabled) {
        updateMotors({.values = {
          motorDebugValues[0],
          motorDebugValues[1],
          motorDebugValues[2],
          motorDebugValues[3]
        }});
    } else {
      if (throttleValue < 20) {
        updateMotors({.values = {1000.0f, 1000.0f, 1000.0f, 1000.0f}});
      } else {
        updateMotors(outputs);
      }
    }
    setMotorOutputs = true;

//    Serial.print(String(outputs.values[0]));
//    Serial.print(", ");
//    Serial.print(String(outputs.values[1]));
//    Serial.print(", ");
//    Serial.print(String(outputs.values[2]));
//    Serial.print(", ");
//    Serial.println(String(outputs.values[3]));
  }
}