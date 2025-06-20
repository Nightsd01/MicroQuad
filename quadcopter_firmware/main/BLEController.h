#pragma once

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <PIDController.h>

#include <functional>
#include <string>
#include <vector>

#include "BLECharacteristic.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "CalibrationEvent.h"
#include "CrossPlatformEnum.h"
#include "IMU.h"

// Bluetooth constants
#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CONTROL_CHARACTERISTIC_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define ARM_CHARACTERISTIC_UUID "baf0bcca-634f-11eb-ae93-0242ac130002"
#define TELEM_CHARACTERISTIC_UUID "e35f992e-5e7c-11eb-ae93-0242ac130002"
#define RESET_CHARACTERISTIC_UUID "489d3a76-6fdf-11eb-9439-0242ac130002"
#define MOTOR_DEBUG_CHARACTERISTIC_UUID "dec9fad6-0cf9-11ec-82a8-0242ac130003"
#define CALIBRATION_CHARACTERISTIC_UUID "498e876e-0dd2-11ec-82a8-0242ac130003"
#define DEBUG_CHARACTERISTIC_UUID "f0a0afee-0983-4691-adc5-02ee803f5418"
#define PID_CONSTANTS_CHARACTERISTIC_UUID "58471750-7394-4659-bc69-09331eed05a3"

// Current Time Service (CTS)
#define CTS_SERVICE_UUID (uint16_t)0x1805
#define CTS_CURRENT_TIME_CHARACTERISTIC_UUID (uint16_t)0x2A2B

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

#define DEVICE_MANUFACTURER "BradHesse"
#define DEVICE_NAME "MicroQuad"

struct controls_update_t
{
  float throttle;
  float yaw;
  float pitch;
  float roll;
};

struct motor_debug_update_t
{
  int motorNum;
  float motorWriteValue;
};

struct debug_recording_update_t
{
  bool recordDebugData;
  bool sendDebugData;
};

#define DEBUG_PACKET_SIZE_BYTES 160

class BLEController : public BLEServerCallbacks, public BLECharacteristicCallbacks
{
 public:
  BLEController();
  void beginBluetooth(void);
  bool isConnected;
  volatile bool isProcessingBluetoothTransaction;
  void uploadDebugData(uint8_t *data, size_t length);
  void sendTelemetryUpdate(uint8_t *data, size_t size);
  void sendCalibrationUpdate(CalibrationType type, CalibrationRequest calibrationEvent);
  void loopHandler(void);

  // update handlers - these are always called on the main core
  void setControlsUpdateHandler(std::function<void(controls_update_t)> controlsUpdateHandler);
  void setTimeUpdateHandler(std::function<void(double)> timeUpdateHandler);
  void setArmStatusUpdateHandler(std::function<void(bool)> armStatusUpdateHandler);
  void setResetStatusUpdateHandler(std::function<void(void)> resetHandler);
  void setMotorDebugEnabledUpdateHandler(std::function<void(bool)> motorDebugEnabledUpdateHandler);
  void setMotorDebugUpdateHandler(std::function<void(motor_debug_update_t)> motorDebugUpdateHandler);
  void setCalibrationUpdateHandler(std::function<void(CalibrationType, CalibrationResponse)> calibrationUpdateHandler);
  void setDebugDataUpdateHandler(std::function<void(debug_recording_update_t)> debugDataUpdateHandler);
  void setTelemetryTransmissionCompleteHandler(std::function<void()> telemetryTransmissionCompleteHandler);
  void setPIDConstantsUpdateHandler(
      std::function<void(ControlAxis, PIDType, gains_t)> pidConstantsUpdateHandler);  // axis, angle + rate PIDs

  // Override methods from BLEServerCallbacks
  void onConnect(BLEServer *pServer) override;
  void onDisconnect(BLEServer *pServer) override;
  void onMtuChanged(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override;

  // Override methods from BLECharacteristicCallbacks
  void onWrite(BLECharacteristic *pCharacteristic) override;
  void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code) override;

 private:
  BLEServer *_server;
  BLEService *_service;
  BLECharacteristic *_controlCharacteristic;
  BLECharacteristic *_telemetryCharacteristic;
  BLECharacteristic *_armCharacteristic;
  BLECharacteristic *_resetCharacteristic;
  BLECharacteristic *_motorDebugCharacteristic;
  BLECharacteristic *_calibrationCharacteristic;
  BLECharacteristic *_debugCharacteristic;
  BLECharacteristic *_pidConstantsCharacteristic;
  BLECharacteristic *_currentTimeCharacteristic;

  std::function<void(controls_update_t)> _controlsUpdateHandler;
  std::function<void(double)> _timeUpdateHandler;
  std::function<void(bool)> _armStatusUpdateHandler;
  std::function<void(void)> _resetStatusUpdateHandler;
  std::function<void(bool)> _motorDebugEnabledUpdateHandler;
  std::function<void(motor_debug_update_t)> _motorDebugUpdateHandler;
  std::function<void(CalibrationType, CalibrationResponse)> _calibrationUpdateHandler;
  std::function<void(debug_recording_update_t)> _debugDataUpdateHandler;
  std::function<void()> _telemetryTransmissionCompleteHandler;
  std::function<void(ControlAxis, PIDType, gains_t)> _pidConstantsUpdateHandler;

  uint8_t *_debugData;
  size_t _debugDataSize;
  int _currentByteIndex;
  bool _transferringDebugData = false;
  uint64_t _lastDebugDataTransmissionTimestampMillis = 0;
};

#endif  // MATLAB_SIM