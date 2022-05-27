#include <QuadcopterController.h>

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

#define ACCEL_INTERRUPT_PIN 35

#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CONTROL_CHARACTERISTIC_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define ARM_CHARACTERISTIC_UUID "baf0bcca-634f-11eb-ae93-0242ac130002"
#define TELEM_CHARACTERISTIC_UUID "e35f992e-5e7c-11eb-ae93-0242ac130002"
#define RESET_CHARACTERISTIC_UUID "489d3a76-6fdf-11eb-9439-0242ac130002"

#define THROTTLE_KEY "throttle"
#define YAW_KEY "yaw"
#define PITCH_KEY "pitch"
#define ROLL_KEY "roll"

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

#define DEVICE_MANUFACTURER "BradHesse"
#define DEVICE_NAME "MicroQuad"

#define TELEM_DELIMITER ","
#define TELEM_UPDATE_INTERVAL_MILLIS 100

#define BATTERY_SENSE_PIN 34

#define NUM_BATTERY_CELLS 1.0f
#define BATTERY_CELL_MAX_VOLTAGE 4.2f
#define BATTERY_CELL_MIN_VOLTAGE 3.3f
#define BATTERY_SCALE 0.001777081468218f

#define MIN_THROTTLE 1015

BLECharacteristic *controlCharacteristic;
BLECharacteristic *telemetryCharacteristic;
BLECharacteristic *armCharacteristic;
BLECharacteristic *resetCharacteristic;

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

const int motorPins[4] = {33, 25, 26, 27};

bool resetFlag = false;

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
    }
};

class MessageCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        if (characteristic->getUUID().equals(controlCharacteristic->getUUID())) {
          std::string data = characteristic->getValue();
          char *str = (char *)malloc((strlen(data.c_str()) * sizeof(char)) + 10);
          strcpy(str, data.c_str());
          char *ptr = strtok(str, ",");

          int i = 0;
          while (ptr != NULL) {
            float val = strtof(ptr, NULL);
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
        } else if (characteristic->getUUID().equals(resetCharacteristic->getUUID())) {
          Serial.println("SET RESET FLAG");
          resetFlag = true;
        }
    }
};

MPU6050 mpu(0x69);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[512]; // FIFO storage buffer

VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

QuadcopterController *controller;
DiagnosticsWriter *diags;
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
  controller = new QuadcopterController({
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
  }, 0);
}

// Initial setup
void setup() {
    const unsigned long initializationTime = millis();
    Serial.begin(115200);
    Serial.println("BEGAN APP");

    initController();
    
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

    Serial.println("Setting up motor outputs");
    
    for (int i = 0; i < 4; i++) {
      ledcSetup(i, 5000, 8);
      ledcAttachPin(motorPins[i], i);
    }

    Serial.println("Initializing bluetooth connection");

    // Setup BLE Server
    BLEDevice::init(DEVICE_NAME);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());
    
    BLEService *service = server->createService(SERVICE_UUID);
    
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
    
    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);
    ledcSetup(2, 5000, 8);
    ledcSetup(3, 5000, 8);
  
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(33, 0);
    ledcAttachPin(25, 1);
    ledcAttachPin(26, 2);
    ledcAttachPin(27, 3);

    Serial.println("Initializing SD diagnostics");
    diags = new DiagnosticsWriter(DIAGNOSTICS_PATH);

    Serial.println("SETUP COMPLETE");
    diags->writeDiagnostics("SETUP COMPLETE AFTER " + String(millis() - initializationTime) + " ms");

    while (true) {
      loop();
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
  for (int i = 0; i < 4; i++) {
    if (!armed) {
      ledcWrite(i, 0);
    } else {
      ledcWrite(i, ((outputs.values[i] - 1000.0f) / 1000.0f) * 255.0f);
    }
  }
}

void loop() {
  if (resetFlag) {
    Serial.println("Resetting");
    resetFlag = false;
    initController();
  }
  
  updateClientTelemetryIfNeeded();
  
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
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//    Serial.println("Accel yaw = " + String(ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f) + ", pitch = " + String(ypr[1] * (180.0f / (M_PI * 2.0)) + 90.0f) + ", roll = " + String(ypr[2] * (180.0f / (M_PI * 2.0)) + 90.0f));

    if (!updateParams) {
      return;
    }

    controller_values_t controllerValues = {
      .leftStickInput = {
        .x = (uint8_t)yawValue,
        .y = (uint8_t)throttleValue
      },
      .rightStickInput = {
        .x = (uint8_t)rollValue,
        .y = (uint8_t)pitchValue
      }
    };

    position_values_t positionValues = {
      .yaw = ypr[0] * (180.0f / (M_PI * 2.0)) + 90.0f,
      .pitch = ypr[2] * (180.0f / (M_PI * 2.0)) + 90.0f,
      .roll = ypr[1] * (180.0f / (M_PI * 2.0)) + 90.0f
    };



    motor_outputs_t outputs = controller->calculateOutputs(positionValues, controllerValues, millis());

    previousMotorOutputs = outputs;
    
    static uint64_t lastUpdateTs = 0;

    if (millis() - lastUpdateTs > 50) {
      lastUpdateTs = millis();
      diags->writeDiagnostics("yaw = " + String(ypr[0]) + ", pitch = " + String(ypr[1]) + ", roll = " + String(ypr[2]));
      diags->writeDiagnostics("throttle = " + String(throttleValue) + ", 1 = " + String(outputs.values[0]) + ", 2 = " + String(outputs.values[1]) + ", 3 = " + String(outputs.values[2]) + ", 4 = " + String(outputs.values[3]));
    }

    if (throttleValue < 20) {
      updateMotors({.values = {1000.0f, 1000.0f, 1000.0f, 1000.0f}});
    } else {
      updateMotors(outputs);
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