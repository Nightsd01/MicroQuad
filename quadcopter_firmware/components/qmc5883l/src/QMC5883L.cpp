#include "QMC5883L.h"

#include <math.h>

#include "esp_log.h"

static const char *TAG = "QMC5883L";

#define QMC5883L_ADDRESS 0x0D
#define QMC5883L_REG_X_LSB 0x00
#define QMC5883L_REG_X_MSB 0x01
#define QMC5883L_REG_Y_LSB 0x02
#define QMC5883L_REG_Y_MSB 0x03
#define QMC5883L_REG_Z_LSB 0x04
#define QMC5883L_REG_Z_MSB 0x05
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CONTROL 0x09
#define QMC5883L_REG_RESET 0x0A

class QMC5883L::QMC5883LPrivate
{
 public:
  TwoWire *wire = nullptr;
  std::function<void(mag_update_t)> updateHandler;
  bool initialized = false;

  unsigned long lastReadTime = 0;
  unsigned long readInterval = 100;  // read every 100ms

  uint8_t controlReg = 0x1D;
  float declinationAngle = 0.0f;

  // Offsets determined by calibration
  float xOffset = 0.0f;
  float yOffset = 0.0f;
  float zOffset = 0.0f;

  bool writeRegister(uint8_t reg, uint8_t value)
  {
    wire->beginTransmission(QMC5883L_ADDRESS);
    wire->write(reg);
    wire->write(value);
    return (wire->endTransmission() == 0);
  }

  bool readRegisters(uint8_t startReg, uint8_t *buffer, size_t length)
  {
    wire->beginTransmission(QMC5883L_ADDRESS);
    wire->write(startReg);
    if (wire->endTransmission(false) != 0) {
      return false;
    }

    if (wire->requestFrom((int)QMC5883L_ADDRESS, (int)length) != (int)length) {
      return false;
    }

    for (size_t i = 0; i < length; i++) {
      buffer[i] = wire->read();
    }
    return true;
  }

  void updateControlRegister() { writeRegister(QMC5883L_REG_CONTROL, controlReg); }

  bool readRaw(int16_t &x, int16_t &y, int16_t &z)
  {
    uint8_t data[6];
    if (!readRegisters(QMC5883L_REG_X_LSB, data, 6)) {
      return false;
    }
    x = (int16_t)(data[1] << 8 | data[0]);
    y = (int16_t)(data[3] << 8 | data[2]);
    z = (int16_t)(data[5] << 8 | data[4]);
    return true;
  }

  void readData()
  {
    int16_t xi, yi, zi;
    if (!readRaw(xi, yi, zi)) {
      ESP_LOGW(TAG, "Failed to read magnetometer data.");
      return;
    }

    // Apply offsets
    float fx = (float)xi - xOffset;
    float fy = (float)yi - yOffset;
    float fz = (float)zi - zOffset;

    float heading = atan2(fy, fx);
    heading += declinationAngle;
    heading *= 180.0f / (float)M_PI;
    if (heading < 0) {
      heading += 360.0f;
    }

    mag_update_t update = {fx, fy, fz, heading};
    if (updateHandler) {
      updateHandler(update);
    }
  }

  void setOSR(QMC5883L::Samples samples)
  {
    controlReg &= ~(0b11 << 6);
    uint8_t val = 0;
    switch (samples) {
      case QMC5883L::Samples::OSR512:
        val = 0b00;
        break;
      case QMC5883L::Samples::OSR256:
        val = 0b01;
        break;
      case QMC5883L::Samples::OSR128:
        val = 0b10;
        break;
      case QMC5883L::Samples::OSR64:
        val = 0b11;
        break;
    }
    controlReg |= (val << 6);
  }

  void setRNG(QMC5883L::Range range)
  {
    controlReg &= ~(0b11 << 4);
    uint8_t val = 0;
    switch (range) {
      case QMC5883L::Range::GAUSS_2:
        val = 0b00;
        break;
      case QMC5883L::Range::GAUSS_8:
        val = 0b01;
        break;
    }
    controlReg |= (val << 4);
  }

  void setODR(QMC5883L::DataRate dataRate)
  {
    controlReg &= ~(0b11 << 2);
    uint8_t val = 0;
    switch (dataRate) {
      case QMC5883L::DataRate::HZ10:
        val = 0b00;
        break;
      case QMC5883L::DataRate::HZ50:
        val = 0b01;
        break;
      case QMC5883L::DataRate::HZ100:
        val = 0b10;
        break;
      case QMC5883L::DataRate::HZ200:
        val = 0b11;
        break;
    }
    controlReg |= (val << 2);
  }

  void setMode(QMC5883L::MeasurementMode mode)
  {
    controlReg &= ~(0b11);
    uint8_t val = 0;
    switch (mode) {
      case QMC5883L::MeasurementMode::STANDBY:
        val = 0b00;
        break;
      case QMC5883L::MeasurementMode::CONTINUOUS:
        val = 0b01;
        break;
    }
    controlReg |= val;
  }
};

QMC5883L::QMC5883L(void) { _p = new QMC5883LPrivate(); }

QMC5883L::~QMC5883L() { delete _p; }

esp_err_t QMC5883L::begin(TwoWire &wire, std::function<void(mag_update_t)> updateHandler)
{
  _p->wire = &wire;
  _p->updateHandler = updateHandler;

  // Check communication
  uint8_t dummy;
  if (!_p->readRegisters(QMC5883L_REG_STATUS, &dummy, 1)) {
    ESP_LOGE(TAG, "QMC5883L not responding");
    return ESP_FAIL;
  }

  // Reset
  if (!_p->writeRegister(QMC5883L_REG_RESET, 0x01)) {
    ESP_LOGE(TAG, "Failed to reset QMC5883L");
    return ESP_FAIL;
  }
  delay(100);

  if (!_p->writeRegister(QMC5883L_REG_CONTROL, _p->controlReg)) {
    ESP_LOGE(TAG, "Failed to write control register");
    return ESP_FAIL;
  }

  _p->initialized = true;
  _p->lastReadTime = millis();

  return ESP_OK;
}

void QMC5883L::loopHandler(void)
{
  if (!_p->initialized) return;

  unsigned long currentTime = millis();
  if (currentTime - _p->lastReadTime >= _p->readInterval) {
    _p->lastReadTime = currentTime;
    _p->readData();
  }
}

void QMC5883L::setRange(Range range)
{
  _p->setRNG(range);
  _p->updateControlRegister();
}

void QMC5883L::setMeasurementMode(MeasurementMode mode)
{
  _p->setMode(mode);
  _p->updateControlRegister();
}

void QMC5883L::setDataRate(DataRate dataRate)
{
  _p->setODR(dataRate);
  _p->updateControlRegister();
}

void QMC5883L::setSamples(Samples samples)
{
  _p->setOSR(samples);
  _p->updateControlRegister();
}

void QMC5883L::setDeclinationAngle(float declinationAngle) { _p->declinationAngle = declinationAngle; }

mag_calibration_offsets_t QMC5883L::calibrate(size_t sampleCount, uint32_t sampleDelayMs)
{
  if (!_p->initialized) {
    ESP_LOGE(TAG, "QMC5883L not initialized before calibration");
    return {.error = ESP_FAIL};
  }

  int16_t x, y, z;
  int16_t xMin = 32767, yMin = 32767, zMin = 32767;
  int16_t xMax = -32768, yMax = -32768, zMax = -32768;

  ESP_LOGI(TAG, "Starting calibration - rotate the sensor in all directions...");

  for (size_t i = 0; i < sampleCount; i++) {
    if (!_p->readRaw(x, y, z)) {
      ESP_LOGW(TAG, "Failed to read sample during calibration");
      delay(sampleDelayMs);
      continue;
    }

    if (x < xMin) xMin = x;
    if (y < yMin) yMin = y;
    if (z < zMin) zMin = z;

    if (x > xMax) xMax = x;
    if (y > yMax) yMax = y;
    if (z > zMax) zMax = z;

    delay(sampleDelayMs);
  }

  // Compute offsets
  _p->xOffset = (xMax + xMin) / 2.0f;
  _p->yOffset = (yMax + yMin) / 2.0f;
  _p->zOffset = (zMax + zMin) / 2.0f;

  return {
      .error = ESP_OK,
      .x_offset = _p->xOffset,
      .y_offset = _p->yOffset,
      .z_offset = _p->zOffset,
  };
}

void QMC5883L::setXOffset(float xOffset) { _p->xOffset = xOffset; }
void QMC5883L::setYOffset(float yOffset) { _p->yOffset = yOffset; }
void QMC5883L::setZOffset(float zOffset) { _p->zOffset = zOffset; }
