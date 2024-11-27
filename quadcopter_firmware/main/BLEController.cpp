#include "BLEController.h"

#include <AsyncController.h>
#include <DebugHelper.h>
#include <Logger.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gattc_api.h>

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

void BLEController::beginBluetooth(void)
{
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(150);
  _server = BLEDevice::createServer();
  _server->setCallbacks(this);

  _service = _server->createService(BLEUUID(std::string(SERVICE_UUID)), 30, 0);

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
  } else if (characteristic->getUUID().equals(_calibrationCharacteristic->getUUID())) {
    if (_calibrationUpdateHandler) {
      _calibrationUpdateHandler();
    }
  } else if (characteristic->getUUID().equals(_debugCharacteristic->getUUID())) {
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
      if (pCharacteristic->getUUID().equals(_debugCharacteristic->getUUID())) {
        _waitingForDebugPacketResponse = false;
        return;
      } else if (!pCharacteristic->getUUID().equals(_telemetryCharacteristic->getUUID())) {
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
  LOG_INFO("Begin debug data upload");
  int currentByteIndex = 0;
  const int packetSize = 160;
  String firstInfo = String("debug:") + String((int)length);

  _waitingForDebugPacketResponse = true;
  _debugCharacteristic->setValue(firstInfo.c_str());
  _debugCharacteristic->notify();
  delay(30);
  while (currentByteIndex < length) {
    _waitingForDebugPacketResponse = true;
    if (currentByteIndex + packetSize < length) {
      _debugCharacteristic->setValue(&data[currentByteIndex], packetSize);
    } else {
      _debugCharacteristic->setValue(&data[currentByteIndex], length - currentByteIndex);
    }
    _debugCharacteristic->notify();
    delay(30);
    currentByteIndex += packetSize;
  }

  _debugCharacteristic->setValue(TERMINATE_UPLOAD_STREAM_SIGNAL);
  _debugCharacteristic->notify();
  LOG_INFO("End debug data upload");
}

void BLEController::sendTelemetryUpdate(uint8_t *data, size_t size)
{
  _telemetryCharacteristic->setValue(data, size);
  _telemetryCharacteristic->notify();
}

void BLEController::sendCalibrationData(calibration_data_t calibrationData)
{
  uint8_t *data = (uint8_t *)&calibrationData;
  const size_t size = sizeof(calibrationData);

  // Send data chunks
  for (size_t byteIndex = 0; byteIndex < size; byteIndex += MAX_CALIBRATION_PACKET_SIZE_BYTES) {
    const size_t packetSize = MIN(MAX_CALIBRATION_PACKET_SIZE_BYTES, size - byteIndex);
    _calibrationCharacteristic->setValue(&data[byteIndex], packetSize);
    _calibrationCharacteristic->notify();
    delay(30);
  }

  // Send termination signal
  _calibrationCharacteristic->setValue(TERMINATE_UPLOAD_STREAM_SIGNAL);
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

void BLEController::setCalibrationUpdateHandler(std::function<void()> calibrationUpdateHandler)
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