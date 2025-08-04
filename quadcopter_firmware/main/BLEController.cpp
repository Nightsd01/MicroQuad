#include "BLEController.h"

#ifndef MATLAB_SIM

#include <AsyncController.h>
#include <DebugHelper.h>
#include <Logger.h>
#include <NimBLEDevice.h>
#include <mbedtls/md5.h>
#include <stdio.h>   // For sprintf, if you want a hex string output
#include <string.h>  // For memset, if needed

#include "Constants.h"

#define MAX_CALIBRATION_PACKET_SIZE_BYTES 10
#define TERMINATE_UPLOAD_STREAM_SIGNAL "==TERMINATE=="

BLEController::BLEController()
{
  isConnected = false;
  _controlCharacteristic = NULL;
  _telemetryCharacteristic = NULL;
  _armCharacteristic = NULL;
  _resetCharacteristic = NULL;
  _motorDebugCharacteristic = NULL;
  _calibrationCharacteristic = NULL;
  _debugCharacteristic = NULL;
}

static std::vector<std::string> _split(std::string to_split, std::string delimiter)
{
  size_t pos = 0;
  std::vector<std::string> matches{};
  do {
    pos = to_split.find(delimiter);
    int change_end;
    if (pos == std::string::npos) {
      pos = to_split.length() - 1;
      change_end = 1;
    } else {
      change_end = 0;
    }
    matches.push_back(to_split.substr(0, pos + change_end));

    to_split.erase(0, pos + 1);
  } while (!to_split.empty());
  return matches;
}

std::string _calculateMD5HashString(const uint8_t *data, size_t length)
{
  uint8_t md5_result[16];
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);

  // Initialize MD5 state
  mbedtls_md5_starts(&ctx);

  // Process the data
  mbedtls_md5_update(&ctx, data, length);

  // Finalize and get the hash
  mbedtls_md5_finish(&ctx, md5_result);

  // Free the context
  mbedtls_md5_free(&ctx);

  char md5_string[33];  // 32 hex characters + null terminator
  for (int i = 0; i < 16; i++) {
    sprintf(&md5_string[i * 2], "%02x", md5_result[i]);
  }
  md5_string[32] = '\0';

  return std::string(md5_string);
}

void BLEController::beginBluetooth(void)
{
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(517);  // Max MTU for BLE 4.2+
  _server = NimBLEDevice::createServer();
  _server->setCallbacks(this);

  _service = _server->createService(SERVICE_UUID);

  // Control characteristic
  _controlCharacteristic = _service->createCharacteristic(
      CONTROL_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _controlCharacteristic->setCallbacks(this);

  // Telemetry characteristic
  _telemetryCharacteristic = _service->createCharacteristic(
      TELEM_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _telemetryCharacteristic->setCallbacks(this);

  // Arm characteristic
  _armCharacteristic = _service->createCharacteristic(
      ARM_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _armCharacteristic->setCallbacks(this);

  // Reset characteristic
  _resetCharacteristic = _service->createCharacteristic(
      RESET_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _resetCharacteristic->setCallbacks(this);

  // Motor debug characteristic
  _motorDebugCharacteristic = _service->createCharacteristic(
      MOTOR_DEBUG_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _motorDebugCharacteristic->setCallbacks(this);

  // Calibration characteristic
  _calibrationCharacteristic = _service->createCharacteristic(
      CALIBRATION_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _calibrationCharacteristic->setCallbacks(this);

  // Debug characteristic
  _debugCharacteristic = _service->createCharacteristic(
      DEBUG_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _debugCharacteristic->setCallbacks(this);

  // PID constants characteristic
  _pidConstantsCharacteristic = _service->createCharacteristic(
      PID_CONSTANTS_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR);
  _pidConstantsCharacteristic->setCallbacks(this);

  // Current Time Service Characteristic
  _currentTimeCharacteristic = _service->createCharacteristic(
      CTS_CURRENT_TIME_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  _currentTimeCharacteristic->setCallbacks(this);


  // Device info characteristics
  NimBLECharacteristic *manufacturerCharacteristic =
      _service->createCharacteristic(DEVINFO_MANUFACTURER_UUID, NIMBLE_PROPERTY::READ);
  manufacturerCharacteristic->setValue(DEVICE_MANUFACTURER);

  NimBLECharacteristic *deviceNameCharacteristic =
      _service->createCharacteristic(DEVINFO_NAME_UUID, NIMBLE_PROPERTY::READ);
  deviceNameCharacteristic->setValue(DEVICE_NAME);

  NimBLECharacteristic *deviceMacAddressCharacteristic =
      _service->createCharacteristic(DEVINFO_SERIAL_UUID, NIMBLE_PROPERTY::READ);
  String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
  deviceMacAddressCharacteristic->setValue(chipId.c_str());

  // Start the service
  _service->start();

  LOG_INFO_ASYNC_ON_MAIN("BLE setup complete");

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponseData(pAdvertising->getAdvertisementData());
  pAdvertising->start();

  LOG_INFO_ASYNC_ON_MAIN("BLE setup complete");
}

void BLEController::onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
{
  LOG_INFO_ASYNC_ON_MAIN("Connected");
  isConnected = true;

  // Update connection parameters for better throughput
  pServer->updateConnParams(connInfo.getConnHandle(), 6, 6, 0, 400);
}

void BLEController::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
{
  LOG_INFO_ASYNC_ON_MAIN("Disconnected, reason: %d", reason);
  isConnected = false;

  // Restart advertising
  NimBLEDevice::startAdvertising();

  if (_armStatusUpdateHandler) {
    _armStatusUpdateHandler(false);
  }
}

void BLEController::onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo)
{
  LOG_INFO_ASYNC_ON_MAIN("MTU changed to: %i", MTU);
}

void BLEController::onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo)
{
  // TODO: switch this to a mutex
  isProcessingBluetoothTransaction = true;

  if (characteristic->getUUID().equals(NimBLEUUID(CONTROL_CHARACTERISTIC_UUID))) {
    std::string data = characteristic->getValue();
    char *str = (char *)malloc((strlen(data.c_str()) * sizeof(char)) + 10);
    strcpy(str, data.c_str());
    char *ptr = strtok(str, ",");

    controls_update_t controlsUpdate;

    int i = 0;
    while (ptr != NULL) {
      double val = strtod(ptr, NULL);
      if (i == 0) {
        controlsUpdate.throttle = val;
      } else if (i == 1) {
        controlsUpdate.yaw = val;
      } else if (i == 2) {
        controlsUpdate.pitch = val;
      } else {
        controlsUpdate.roll = val;
      }
      i++;
      ptr = strtok(NULL, ",");
    }

    free(str);

    if (_controlsUpdateHandler) {
      _controlsUpdateHandler(controlsUpdate);
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(ARM_CHARACTERISTIC_UUID))) {
    NimBLEAttValue value = characteristic->getValue();
    if (value.length() > 0) {
      bool armed = value[0];
      if (_armStatusUpdateHandler) {
        _armStatusUpdateHandler(armed);
      }
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(RESET_CHARACTERISTIC_UUID))) {
    LOG_INFO_ASYNC_ON_MAIN("SET RESET FLAG");
    if (_resetStatusUpdateHandler) {
      _resetStatusUpdateHandler();
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(MOTOR_DEBUG_CHARACTERISTIC_UUID))) {
    std::string value = characteristic->getValue();
    std::vector<std::string> components = _split(value, ":");
    if (components.size() == 0) {
      LOG_ERROR_ASYNC_ON_MAIN("ERROR: Incorrect motor debug packet (1)");
      return;
    }
    if (components[0] == "motordebug_enabled") {
      if (components.size() == 1) {
        LOG_ERROR_ASYNC_ON_MAIN("ERROR: Incorrect motor debug packet (2)");
        return;
      }
      std::string value = components[1];
      bool motorDebugEnabled = value == "1";
      if (_motorDebugEnabledUpdateHandler) {
        _motorDebugEnabledUpdateHandler(motorDebugEnabled);
      }
      LOG_INFO_ASYNC_ON_MAIN("Changing motor debug status to %s", motorDebugEnabled ? "enabled" : "disabled");
    } else if (components[0] == "motordebug_value") {
      if (components.size() < 3) {
        LOG_ERROR_ASYNC_ON_MAIN("ERROR: Incorrect motor debug packet (3)");
        return;
      }
      int motorNumber = atoi(components[1].c_str()) - 1;
      int motorValue = atoi(components[2].c_str());
      double motorWriteValue = ((((double)motorValue) / 255.0f) * 1000.0f) + 1000.0f;
      LOG_INFO_ASYNC_ON_MAIN("Updating motor %i to %f", motorNumber, motorWriteValue);
      if (_motorDebugUpdateHandler) {
        const motor_debug_update_t motorDebugUpdate = {
            .motorNum = motorNumber,
            .motorWriteValue = (float)motorWriteValue};
        if (_motorDebugUpdateHandler) {
          _motorDebugUpdateHandler(motorDebugUpdate);
        }
      }
    } else {
      LOG_ERROR_ASYNC_ON_MAIN("ERROR: Incorrect motor debug packet (4)");
      return;
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(CALIBRATION_CHARACTERISTIC_UUID))) {
    if (_calibrationUpdateHandler) {
      std::string value = characteristic->getValue();
      std::vector<std::string> components = _split(value, ":");
      if (components.size() < 2) {
        LOG_ERROR_ASYNC_ON_MAIN("Incorrect calibration packet");
        return;
      }
      // WARNING: if the bluetooth app sends anything other than a valid
      // integer, this will treat it as a 0 (calibration start)
      const int calibrationResponse = atoi(components[1].c_str());
      const int calibrationType = atoi(components[0].c_str());
      LOG_INFO_ASYNC_ON_MAIN(
          "Received calibration update: type = %d, response = %d",
          calibrationType,
          calibrationResponse);
      _calibrationUpdateHandler((CalibrationType)calibrationType, (CalibrationResponse)calibrationResponse);
    } else {
      LOG_ERROR_ASYNC_ON_MAIN("No calibration update handler set");
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(DEBUG_CHARACTERISTIC_UUID))) {
    std::string val = characteristic->getValue();
    LOG_INFO_ASYNC_ON_MAIN("Received debug command: %s", val.c_str());
    debug_recording_update_t debugDataUpdate = {.recordDebugData = false, .sendDebugData = false};
    if (val == "request") {
      LOG_INFO_ASYNC_ON_MAIN("Recording debug data transmission");
      debugDataUpdate.sendDebugData = true;
    } else if (val == "record:1") {
      LOG_INFO_ASYNC_ON_MAIN("Recording debug data start");
      debugDataUpdate.recordDebugData = true;
    } else if (val == "record:0") {
      LOG_INFO_ASYNC_ON_MAIN("Recording debug data stop");
      debugDataUpdate.recordDebugData = false;
    }
    if (_debugDataUpdateHandler) {
      _debugDataUpdateHandler(debugDataUpdate);
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(PID_CONSTANTS_CHARACTERISTIC_UUID))) {
    LOG_INFO_ASYNC_ON_MAIN("Received PID constants update");
    NimBLEAttValue value = characteristic->getValue();
    if (value.length() != 14) {
      LOG_ERROR_ASYNC_ON_MAIN("Incorrect PID constants packet size %i", value.length());
      return;
    }
    const uint8_t *pidConstants = value.data();
    // Assumes the following packet structure:
    // byte 0 - axis enum
    // byte 1 - PID type enum
    // bytes 2 to 14 - angle gains (kp, ki, kd)
    ControlAxis axis = (ControlAxis)pidConstants[0];
    PIDType type = (PIDType)pidConstants[1];
    float kP, kI, kD;
    memcpy(&kP, &pidConstants[2], sizeof(float));
    memcpy(&kI, &pidConstants[6], sizeof(float));
    memcpy(&kD, &pidConstants[10], sizeof(float));

    gains_t gains = {.kP = kP, .kI = kI, .kD = kD};
    if (_pidConstantsUpdateHandler) {
      AsyncController::main.executePossiblySync(
          [this, axis, type, gains]() { _pidConstantsUpdateHandler(axis, type, gains); });
      LOG_INFO_ASYNC_ON_MAIN(
          "Updating PID constants for axis %d, type %d, p = %f, i = %f, d = %f",
          axis,
          type,
          gains.kP,
          gains.kI,
          gains.kD);
    }
  } else if (characteristic->getUUID().equals(NimBLEUUID(CTS_CURRENT_TIME_CHARACTERISTIC_UUID))) {
    LOG_INFO_ASYNC_ON_MAIN("Received Current Time update");
    NimBLEAttValue value = characteristic->getValue();

    // The timestamp is sent as a Double from iOS, which is 8 bytes.
    if (value.length() == sizeof(double)) {
      double timestamp;
      memcpy(&timestamp, value.data(), sizeof(double));

      if (_timeUpdateHandler) {
        _timeUpdateHandler(timestamp);
      }
    } else {
      LOG_ERROR_ASYNC_ON_MAIN(
          "Invalid Current Time data received - expected %zu bytes, got %zu",
          sizeof(double),
          value.length());
    }
    }
  }
  // TODO: switch this to a mutex
  isProcessingBluetoothTransaction = false;
}

void BLEController::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
  String typeStr;
  switch (code) {
    case 0:  // SUCCESS_INDICATE/NOTIFY - code 0 means success
      typeStr = "SUCCESS";
      if (pCharacteristic->getUUID().equals(NimBLEUUID(TELEM_CHARACTERISTIC_UUID))) {
        AsyncController::main.executePossiblySync([this]() {
          if (_telemetryTransmissionCompleteHandler != nullptr) {
            _telemetryTransmissionCompleteHandler();
            _telemetryTransmissionCompleteHandler = nullptr;
          }
        });
      }
      break;
    case BLE_HS_ENOTCONN:
      typeStr = "ERROR_NO_CLIENT";
      break;
    case BLE_HS_EINVAL:
      typeStr = "ERROR_INVALID";
      break;
    default:
      typeStr = "ERROR_CODE_" + String(code);
      break;
  }

  LOG_VERBOSE_ASYNC_ON_MAIN(
      "received BLE status update = %s, code = %d, characteristic UUID = %s",
      typeStr.c_str(),
      code,
      pCharacteristic->getUUID().toString().c_str());
}

void BLEController::uploadDebugData(uint8_t *data, size_t length)
{
  LOG_INFO("Begin debug data upload - %d bytes, MD5 = %s", length, _calculateMD5HashString(data, length).c_str());
  String firstInfo = String("debug:") + String((int)length);

  _debugData = data;
  _debugDataSize = length;
  _transferringDebugData = true;
  _debugCharacteristic->setValue(firstInfo.c_str());
  _debugCharacteristic->notify();
}

void BLEController::loopHandler(void)
{
  if (!_transferringDebugData) {
    return;
  }

  if (millis() - _lastDebugDataTransmissionTimestampMillis < 30) {
    return;
  }

  if (_currentByteIndex >= _debugDataSize) {
    _debugCharacteristic->setValue(TERMINATE_UPLOAD_STREAM_SIGNAL);
    _debugCharacteristic->notify();
    _transferringDebugData = false;
    _debugData = NULL;
    _currentByteIndex = 0;
    LOG_INFO("End debug data upload");
    return;
  }

  if (_currentByteIndex + DEBUG_PACKET_SIZE_BYTES < _debugDataSize) {
    _debugCharacteristic->setValue(&_debugData[_currentByteIndex], DEBUG_PACKET_SIZE_BYTES);
  } else {
    _debugCharacteristic->setValue(&_debugData[_currentByteIndex], _debugDataSize - _currentByteIndex);
  }
  _lastDebugDataTransmissionTimestampMillis = millis();
  _debugCharacteristic->notify();
  _currentByteIndex += DEBUG_PACKET_SIZE_BYTES;
}

void BLEController::sendTelemetryUpdate(uint8_t *data, size_t size)
{
  _telemetryCharacteristic->setValue(data, size);
  _telemetryCharacteristic->notify();
}

void BLEController::sendCalibrationUpdate(CalibrationType type, CalibrationRequest calibrationEvent)
{
  String packet = String((int)type) + ":" + String((int)calibrationEvent);
  _calibrationCharacteristic->setValue(packet.c_str());
  _calibrationCharacteristic->notify();
  delay(30);
}

// update handlers
void BLEController::setControlsUpdateHandler(std::function<void(controls_update_t)> controlsUpdateHandler)
{
  _controlsUpdateHandler = controlsUpdateHandler;
}

void BLEController::setArmStatusUpdateHandler(std::function<void(bool)> armStatusUpdateHandler)
{
  _armStatusUpdateHandler = armStatusUpdateHandler;
}

void BLEController::setResetStatusUpdateHandler(std::function<void(void)> resetHandler)
{
  _resetStatusUpdateHandler = resetHandler;
}

void BLEController::setMotorDebugEnabledUpdateHandler(std::function<void(bool)> motorDebugEnabledUpdateHandler)
{
  _motorDebugEnabledUpdateHandler = motorDebugEnabledUpdateHandler;
}

void BLEController::setMotorDebugUpdateHandler(std::function<void(motor_debug_update_t)> motorDebugUpdateHandler)
{
  _motorDebugUpdateHandler = motorDebugUpdateHandler;
}

void BLEController::setCalibrationUpdateHandler(
    std::function<void(CalibrationType, CalibrationResponse)> calibrationUpdateHandler)
{
  _calibrationUpdateHandler = calibrationUpdateHandler;
}

void BLEController::setDebugDataUpdateHandler(std::function<void(debug_recording_update_t)> debugDataUpdateHandler)
{
  _debugDataUpdateHandler = debugDataUpdateHandler;
}

void BLEController::setTelemetryTransmissionCompleteHandler(std::function<void()> telemetryTransmissionCompleteHandler)
{
  if (_telemetryTransmissionCompleteHandler != nullptr) {
    LOG_ERROR("Telemetry transmission complete handler already set");
    return;
  }
  _telemetryTransmissionCompleteHandler = telemetryTransmissionCompleteHandler;
}

void BLEController::setPIDConstantsUpdateHandler(
    std::function<void(ControlAxis, PIDType, gains_t)> pidConstantsUpdateHandler)
{
  _pidConstantsUpdateHandler = pidConstantsUpdateHandler;
}

void BLEController::setTimeUpdateHandler(std::function<void(double)> timeUpdateHandler)
{
  _timeUpdateHandler = timeUpdateHandler;
}
#endif  // MATLAB_SIM