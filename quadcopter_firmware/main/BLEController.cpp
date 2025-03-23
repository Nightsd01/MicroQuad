#include "BLEController.h"

#include <AsyncController.h>
#include <DebugHelper.h>
#include <Logger.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gattc_api.h>
#include <mbedtls/md5.h>
#include <stdio.h>   // For sprintf, if you want a hex string output
#include <string.h>  // For memset, if needed

#include "BLE2902.h"
#include "BLEUtils.h"

#define MAX_CALIBRATION_PACKET_SIZE_BYTES 10
#define TERMINATE_UPLOAD_STREAM_SIGNAL "==TERMINATE=="

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

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
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(150);
  _server = BLEDevice::createServer();
  _server->setCallbacks(this);

  _service = _server->createService(BLEUUID(SERVICE_UUID), 30, 0);

  _controlCharacteristic = _service->createCharacteristic(
      CONTROL_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _controlCharacteristic->setCallbacks(this);
  _controlCharacteristic->addDescriptor(new BLE2902());

  _telemetryCharacteristic = _service->createCharacteristic(
      TELEM_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _telemetryCharacteristic->setCallbacks(this);
  _telemetryCharacteristic->addDescriptor(new BLE2902());
  _telemetryCharacteristic->setWriteNoResponseProperty(true);

  _armCharacteristic = _service->createCharacteristic(
      ARM_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _armCharacteristic->setCallbacks(this);
  _armCharacteristic->addDescriptor(new BLE2902());

  _resetCharacteristic = _service->createCharacteristic(
      RESET_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _resetCharacteristic->setCallbacks(this);
  _resetCharacteristic->addDescriptor(new BLE2902());

  _motorDebugCharacteristic = _service->createCharacteristic(
      MOTOR_DEBUG_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _motorDebugCharacteristic->setCallbacks(this);
  _motorDebugCharacteristic->addDescriptor(new BLE2902());

  _calibrationCharacteristic = _service->createCharacteristic(
      CALIBRATION_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _calibrationCharacteristic->setCallbacks(this);
  _calibrationCharacteristic->addDescriptor(new BLE2902());
  _calibrationCharacteristic->setWriteNoResponseProperty(true);

  _debugCharacteristic = _service->createCharacteristic(
      DEBUG_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _debugCharacteristic->setCallbacks(this);
  _debugCharacteristic->addDescriptor(new BLE2902());

  _pidConstantsCharacteristic = _service->createCharacteristic(
      PID_CONSTANTS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR);
  _pidConstantsCharacteristic->setCallbacks(this);
  _pidConstantsCharacteristic->addDescriptor(new BLE2902());
  _pidConstantsCharacteristic->setWriteNoResponseProperty(true);

  // Set the MTU size
  esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
  if (status != ESP_OK) {
    LOG_ERROR("set local MTU failed, error code = %x", status);
  }

  // Register device info service, that contains the device's UUID, manufacturer
  // and name. _deviceInfoService = _server->createService(DEVINFO_UUID);
  BLECharacteristic *manufacturerCharacteristic =
      _service->createCharacteristic(DEVINFO_MANUFACTURER_UUID, BLECharacteristic::PROPERTY_READ);
  manufacturerCharacteristic->setValue(DEVICE_MANUFACTURER);
  BLECharacteristic *deviceNameCharacteristic =
      _service->createCharacteristic(DEVINFO_NAME_UUID, BLECharacteristic::PROPERTY_READ);
  deviceNameCharacteristic->setValue(DEVICE_NAME);
  BLECharacteristic *deviceMacAddressCharacteristic =
      _service->createCharacteristic(DEVINFO_SERIAL_UUID, BLECharacteristic::PROPERTY_READ);
  String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
  deviceMacAddressCharacteristic->setValue(chipId.c_str());

  // Advertise services
  BLEAdvertising *advertisement = _server->getAdvertising();
  BLEAdvertisementData adv;
  adv.setName(DEVICE_NAME);
  adv.setCompleteServices(BLEUUID(SERVICE_UUID));
  advertisement->setAdvertisementData(adv);
  advertisement->start();

  LOG_INFO_ASYNC_ON_MAIN("BLE setup complete");

  _service->start();
}
void BLEController::onConnect(BLEServer *server)
{
  LOG_INFO_ASYNC_ON_MAIN("Connected");
  isConnected = true;
  // Set the MTU size
  esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
  if (status != ESP_OK) {
    LOG_ERROR("set local MTU failed, error code = %x", status);
  }
};

void BLEController::onDisconnect(BLEServer *server)
{
  LOG_INFO_ASYNC_ON_MAIN("Disconnected");
  isConnected = false;
  // Set the MTU size
  esp_err_t status = esp_ble_gatt_set_local_mtu(180);  // Replace 500 with the MTU size you want
  if (status != ESP_OK) {
    LOG_ERROR("set local MTU failed, error code = %x", status);
  }
  _server->startAdvertising();
  if (_armStatusUpdateHandler) {
    _armStatusUpdateHandler(false);
  }
}

void BLEController::onMtuChanged(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
{
  LOG_INFO_ASYNC_ON_MAIN("MTU changed to: %i", param->mtu);
}

void BLEController::onWrite(BLECharacteristic *characteristic)
{
  // TODO: switch this to a mutex
  isProcessingBluetoothTransaction = true;
  if (characteristic->getUUID().equals(_controlCharacteristic->getUUID())) {
    std::string data = std::string(characteristic->getValue().c_str());
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
  } else if (characteristic->getUUID().equals(_armCharacteristic->getUUID())) {
    uint8_t *armData = characteristic->getData();
    bool armed = armData[0];
    if (_armStatusUpdateHandler) {
      _armStatusUpdateHandler(armed);
    }
  } else if (characteristic->getUUID().equals(_resetCharacteristic->getUUID())) {
    LOG_INFO_ASYNC_ON_MAIN("SET RESET FLAG");
    if (_resetStatusUpdateHandler) {
      _resetStatusUpdateHandler();
    }
  } else if (characteristic->getUUID().equals(_motorDebugCharacteristic->getUUID())) {
    std::string value = std::string(characteristic->getValue().c_str());
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
  } else if (characteristic->getUUID().equals(_calibrationCharacteristic->getUUID())) {
    if (_calibrationUpdateHandler) {
      std::string value = std::string(characteristic->getValue().c_str());
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
  } else if (characteristic->getUUID().equals(_debugCharacteristic->getUUID())) {
    std::string val = std::string(characteristic->getValue().c_str());
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
  } else if (characteristic->getUUID().equals(_pidConstantsCharacteristic->getUUID())) {
    LOG_INFO_ASYNC_ON_MAIN("Received PID constants update");
    uint8_t *pidConstants = characteristic->getData();
    if (characteristic->getLength() != 14) {
      LOG_ERROR_ASYNC_ON_MAIN("Incorrect PID constants packet size %i", characteristic->getLength());
      return;
    }
    // Assumes the following packet structure:
    // byte 0 - axis enum
    // byte 1 - PID type enum
    // bytes 2 to 14 - angle gains (kp, ki, kd)
    ControlAxis axis = (ControlAxis)pidConstants[0];
    PIDType type = (PIDType)pidConstants[1];
    gains_t gains = {
        .kP = *((float *)&pidConstants[2]),
        .kI = *((float *)&pidConstants[6]),
        .kD = *((float *)&pidConstants[10]),
    };
    if (_pidConstantsUpdateHandler) {
      // _pidConstantsUpdateHandler(axis, type, gains);
      LOG_INFO_ASYNC_ON_MAIN(
          "Updating PID constants for axis %d, type %d, p = %f, i = %f, d = %f",
          axis,
          type,
          gains.kP,
          gains.kI,
          gains.kD);
    }
  }
  // TODO: switch this to a mutex
  isProcessingBluetoothTransaction = false;
}

void BLEController::onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code)
{
  switch (s) {
    case BLECharacteristicCallbacks::Status::SUCCESS_INDICATE:
      LOG_VERBOSE_ASYNC_ON_MAIN(
          "received BLE status update = SUCCESS_INDICATE, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY:
      LOG_VERBOSE_ASYNC_ON_MAIN(
          "received BLE status update = SUCCESS_NOTIFY, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      if (!pCharacteristic->getUUID().equals(_telemetryCharacteristic->getUUID())) {
        return;
      }
      AsyncController::main.executePossiblySync([this, pCharacteristic]() {
        if (_telemetryTransmissionCompleteHandler != nullptr) {
          _telemetryTransmissionCompleteHandler();
          _telemetryTransmissionCompleteHandler = nullptr;
        }
      });
      break;
    case BLECharacteristicCallbacks::Status::ERROR_INDICATE_DISABLED:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_INDICATE_DISABLED, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::ERROR_NOTIFY_DISABLED:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_NOTIFY_DISABLED, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::ERROR_GATT:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_GATT, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_NO_CLIENT, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::ERROR_INDICATE_TIMEOUT:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_INDICATE_TIMEOUT, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
    case BLECharacteristicCallbacks::Status::ERROR_INDICATE_FAILURE:
      LOG_ERROR_ASYNC_ON_MAIN(
          "received BLE status update = ERROR_INDICATE_FAILURE, code = %d, characteristic UUID = %s",
          code,
          pCharacteristic->getUUID().toString().c_str());
      break;
  }
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