/*!
 * @file QMC5883L.cpp
 * @brief Compatible with QMC5883 HMC5883 and QMC5883
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      PengKaixing(kaixing.peng@dfrobot.com)
 * @version  V1.0.0
 * @date  2022-2-23
 * @url https://github.com/DFRobot/QMC5883L
 */
#include "QMC5883L.h"

#include "Logger.h"

QMC5883L::QMC5883L(TwoWire* pWire, uint8_t I2C_addr)
{
  isHMC_ = false;
  isQMC_ = false;
  minX = 0;
  maxX = 0;
  minY = 0;
  maxY = 0;
  minZ = 0;
  maxZ = 0;
  _offsets = {0, 0, 0};
  firstRun = true;
  this->_pWire = pWire;
  this->_I2C_addr = I2C_addr;
}

QMC5883LObserverId QMC5883L::addObserver(std::function<void(mag_update_t)> updateHandler)
{
  static QMC5883LObserverId observerId = 0;
  LOG_INFO("Adding mag observer with ID = %i", observerId + 1);
  _updateHandlers[++observerId] = updateHandler;
  return observerId;
}

bool QMC5883L::removeObserver(QMC5883LObserverId observerId)
{
  if (_updateHandlers.find(observerId) == _updateHandlers.end()) {
    LOG_ERROR("Failed to remove magnetometer observer with ID = %i", observerId);
    return false;
  }
  _observersToRemove.push_back(observerId);
  return true;
}

void QMC5883L::setCalibrationOffsets(xyz_vector_t offsets) { _offsets = offsets; }

struct calibration_data_t
{
  xyz_vector_t mins;
  xyz_vector_t maxs;
  uint64_t startTimeMillis;
  QMC5883LObserverId observerId;
};

void QMC5883L::calibrate(float durationSeconds, std::function<void(bool, xyz_vector_t)> completionHandler)
{
  calibration_data_t temp = {
      .mins = {1e6f,  1e6f,  1e6f },
      .maxs = {-1e6f, -1e6f, -1e6f},
      .startTimeMillis = millis(),
      .observerId = -1,
  };
  calibration_data_t* data = (calibration_data_t*)malloc(sizeof(calibration_data_t));
  if (!data) {
    LOG_ERROR("Failed to allocate memory for calibration data");
    completionHandler(false, {0, 0, 0});
    return;
  }
  memcpy(data, &temp, sizeof(calibration_data_t));

  data->observerId = this->addObserver([=](mag_update_t update) mutable {
    // 1) Update min/max
    data->mins.x = std::min(data->mins.x, update.x);
    data->maxs.x = std::max(data->maxs.x, update.x);

    data->mins.y = std::min(data->mins.y, update.y);
    data->maxs.y = std::max(data->maxs.y, update.y);

    data->mins.z = std::min(data->mins.z, update.z);
    data->maxs.z = std::max(data->maxs.z, update.z);

    // 2) Check if we've reached calibration time
    uint64_t elapsed = millis() - data->startTimeMillis;
    if ((float)(elapsed) / 1000.0f >= durationSeconds) {
      LOG_INFO("Calibration complete with observer ID = %i", data->observerId);
      // We've collected enough samples. Compute offsets.
      float xOffset = (data->maxs.x + data->mins.x) * 0.5f;
      float yOffset = (data->maxs.y + data->mins.y) * 0.5f;
      float zOffset = (data->maxs.z + data->mins.z) * 0.5f;

      // 3) Store these offsets however you like.
      // For example, if your _compass object has xOffset, yOffset, zOffset:
      this->setCalibrationOffsets({xOffset, yOffset, zOffset});

      completionHandler(true, {xOffset, yOffset, zOffset});

      // 4) Remove the observer so we stop collecting data.
      this->removeObserver(data->observerId);
    }
  });
}

void QMC5883L::loopHandler(void)
{
  uint64_t updateInterval = 5;  // default to 200 hz
  if (_dataRate == QMC5883_DATARATE_10HZ) {
    updateInterval = 100;
  } else if (_dataRate == QMC5883_DATARATE_50HZ) {
    updateInterval = 20;
  } else if (_dataRate == QMC5883_DATARATE_100HZ) {
    updateInterval = 10;
  } else if (_dataRate == QMC5883_DATARATE_200HZ) {
    updateInterval = 5;
  }

  if (millis() - _lastUpdateTimestampMillis < updateInterval) {
    return;
  }

  if (_observersToRemove.size() > 0) {
    for (const auto& observerId : _observersToRemove) {
      _updateHandlers.erase(observerId);
    }
    _observersToRemove.clear();
  }

  _lastUpdateTimestampMillis = millis();
  getHeadingDegrees();
  sVector_t mag = readRaw();
  if (_updateHandlers.size() == 0) {
    return;
  }

  for (const auto& [key, handler] : _updateHandlers) {
    handler({
        .x = ((float)mag.XAxis - _offsets.x) / _gainFactor,
        .y = ((float)mag.YAxis - _offsets.y) / _gainFactor,
        .z = ((float)mag.ZAxis - _offsets.z) / _gainFactor,
        .heading = mag.HeadingDegress,
    });
  }
}

bool QMC5883L::begin(void)
{
  ICType = IC_QMC5883;
  bool ret = false;
  switch (ICType) {
    case IC_NONE:
      ret = false;
      break;
    case IC_HMC5883L:
      if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48) || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) ||
          (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33)) {
        return false;
      }
      setRange(HMC5883L_RANGE_1_3GA);
      setMeasurementMode(HMC5883L_CONTINOUS);
      setDataRate(HMC5883L_DATARATE_15HZ);
      setSamples(HMC5883L_SAMPLES_1);
      mgPerDigit = 0.92f;
      ret = true;
      break;
    case IC_QMC5883:
      writeRegister8(QMC5883_REG_IDENT_B, 0X01);
      writeRegister8(QMC5883_REG_IDENT_C, 0X40);
      writeRegister8(QMC5883_REG_IDENT_D, 0X01);
      writeRegister8(QMC5883_REG_CONFIG_1, 0X1D);
      if ((fastRegister8(QMC5883_REG_IDENT_B) != 0x01) || (fastRegister8(QMC5883_REG_IDENT_C) != 0x40) ||
          (fastRegister8(QMC5883_REG_IDENT_D) != 0x01)) {
        return false;
      }
      setRange(QMC5883_RANGE_8GA);
      setMeasurementMode(QMC5883_CONTINOUS);
      setDataRate(QMC5883_DATARATE_50HZ);
      setSamples(QMC5883_SAMPLES_8);
      mgPerDigit = 4.35f;
      ret = true;
      break;
    case IC_VCM5883L:
      writeRegister8(VCM5883L_CTR_REG1, 0X00);
      writeRegister8(VCM5883L_CTR_REG2, 0X4D);
      ret = true;
      break;
    default:
      ret = false;
      break;
  }
  return ret;
}

sVector_t QMC5883L::readRaw(void)
{
  if (ICType == IC_HMC5883L) {
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M);
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M);
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);
  } else if (ICType == IC_QMC5883) {
    v.XAxis = readRegister16(QMC5883_REG_OUT_X_L);
    v.YAxis = readRegister16(QMC5883_REG_OUT_Y_L);
    v.ZAxis = readRegister16(QMC5883_REG_OUT_Z_L);
  } else if (ICType == IC_VCM5883L) {
    v.XAxis = -readRegister16(VCM5883L_REG_OUT_X_L);
    v.YAxis = -readRegister16(VCM5883L_REG_OUT_Y_L);
    v.ZAxis = -readRegister16(VCM5883L_REG_OUT_Z_L);
  }
  v.AngleXY = (atan2((double)v.YAxis, (double)v.XAxis) * (180 / 3.14159265) + 180);
  v.AngleXZ = (atan2((double)v.ZAxis, (double)v.XAxis) * (180 / 3.14159265) + 180);
  v.AngleYZ = (atan2((double)v.ZAxis, (double)v.YAxis) * (180 / 3.14159265) + 180);
  return v;
}

void QMC5883L::setRange(eRange_t range)
{
  if (ICType == IC_HMC5883L) {
    switch (range) {
      case HMC5883L_RANGE_0_88GA:
        Gauss_LSB_XY = 1370.0;
        break;
      case HMC5883L_RANGE_1_3GA:
        Gauss_LSB_XY = 1090.0;
        break;
      case HMC5883L_RANGE_1_9GA:
        Gauss_LSB_XY = 820.0;
        break;
      case HMC5883L_RANGE_2_5GA:
        Gauss_LSB_XY = 660.0;
        break;
      case HMC5883L_RANGE_4GA:
        Gauss_LSB_XY = 440.0;
        break;
      case HMC5883L_RANGE_4_7GA:
        Gauss_LSB_XY = 390.0;
        break;
      case HMC5883L_RANGE_5_6GA:
        Gauss_LSB_XY = 330.0;
        break;
      case HMC5883L_RANGE_8_1GA:
        Gauss_LSB_XY = 230.0;
        break;
      default:
        break;
    }
    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
  } else if (ICType == IC_QMC5883) {
    switch (range) {
      case QMC5883_RANGE_2GA:
        mgPerDigit = 1.22f;
        _gainFactor = 12000.0f;
        break;
      case QMC5883_RANGE_8GA:
        mgPerDigit = 4.35f;
        _gainFactor = 3000.0f;
        break;
      default:
        break;
    }
    writeRegister8(QMC5883_REG_CONFIG_2, range << 4);
  } else if (ICType == IC_VCM5883L) {
    // default 8G
  }
}

eRange_t QMC5883L::getRange(void)
{
  eRange_t ret;
  switch (ICType) {
    case IC_HMC5883L:
      ret = (eRange_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
      break;
    case IC_QMC5883:
      ret = (eRange_t)((readRegister8(QMC5883_REG_CONFIG_2) >> 4));
      break;
    case IC_VCM5883L:
      ret = QMC5883_RANGE_8GA;
      break;
    default:
      ret = QMC5883_RANGE_8GA;
      break;
  }
  return ret;
}

void QMC5883L::setMeasurementMode(eMode_t mode)
{
  uint8_t value;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_MODE);
      value &= 0b11111100;
      value |= mode;
      writeRegister8(HMC5883L_REG_MODE, value);
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0xfc;
      value |= mode;
      writeRegister8(QMC5883_REG_CONFIG_1, value);
      break;
    case IC_VCM5883L:
      value = readRegister8(VCM5883L_CTR_REG2);
      value &= 0xFE;
      value |= mode;
      writeRegister8(VCM5883L_CTR_REG2, value);
      break;
    default:
      break;
  }
}

eMode_t QMC5883L::getMeasurementMode(void)
{
  uint8_t value = 0;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_MODE);
      value &= 0b00000011;
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0b00000011;
      break;
    case IC_VCM5883L:
      value = readRegister8(VCM5883L_CTR_REG2);
      value &= 0b00000001;
      break;
    default:
      break;
  }
  return (eMode_t)value;
}

void QMC5883L::setDataRate(eDataRate_t dataRate)
{
  uint8_t value;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b11100011;
      value |= (dataRate << 2);
      writeRegister8(HMC5883L_REG_CONFIG_A, value);
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0xf3;
      value |= (dataRate << 2);
      writeRegister8(QMC5883_REG_CONFIG_1, value);
      break;
    case IC_VCM5883L:
      value = readRegister8(VCM5883L_CTR_REG2);
      value &= 0xf3;
      value |= (dataRate << 2);
      writeRegister8(VCM5883L_CTR_REG2, value);
      break;
    default:
      break;
  }
  _dataRate = dataRate;
}

eDataRate_t QMC5883L::getDataRate(void)
{
  uint8_t value = 0;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b00011100;
      value >>= 2;
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0b00001100;
      value >>= 2;
      break;
    case IC_VCM5883L:
      value = readRegister8(VCM5883L_CTR_REG2);
      value &= 0b00001100;
      value >>= 2;
      break;
    default:
      break;
  }
  return (eDataRate_t)value;
}

void QMC5883L::setSamples(eSamples_t samples)
{
  uint8_t value;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b10011111;
      value |= (samples << 5);
      writeRegister8(HMC5883L_REG_CONFIG_A, value);
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0x3f;
      value |= (samples << 6);
      writeRegister8(QMC5883_REG_CONFIG_1, value);
      break;
    case IC_VCM5883L:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0x3f;
      value |= (samples << 6);
      writeRegister8(QMC5883_REG_CONFIG_1, value);
      break;
    default:
      break;
  }
}

eSamples_t QMC5883L::getSamples(void)
{
  uint8_t value = 0;
  switch (ICType) {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b01100000;
      value >>= 5;
      break;
    case IC_QMC5883:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0x3f;
      value >>= 6;
      break;
    case IC_VCM5883L:
      value = readRegister8(QMC5883_REG_CONFIG_1);
      value &= 0x3f;
      value >>= 6;
      break;
    default:
      break;
  }
  return (eSamples_t)value;
}

void QMC5883L::setDeclinationAngle(float declinationAngle) { this->ICdeclinationAngle = declinationAngle; }

void QMC5883L::getHeadingDegrees(void)
{
  float heading = atan2(v.YAxis, v.XAxis);
  heading += this->ICdeclinationAngle;
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  v.HeadingDegress = heading * 180 / PI;
}

int QMC5883L::getICType(void) { return ICType; }

void QMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
  _pWire->beginTransmission(this->_I2C_addr);
#if ARDUINO >= 100
  _pWire->write(reg);
  _pWire->write(value);
#else
  _pWire->send(reg);
  _pWire->send(value);
#endif
  _pWire->endTransmission();
}

uint8_t QMC5883L::fastRegister8(uint8_t reg)
{
  uint8_t value = 0;
  _pWire->beginTransmission(this->_I2C_addr);
#if ARDUINO >= 100
  _pWire->write(reg);
#else
  _pWire->send(reg);
#endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
#if ARDUINO >= 100
  value = _pWire->read();
#else
  value = _pWire->receive();
#endif
  _pWire->endTransmission();
  return value;
}

uint8_t QMC5883L::readRegister8(uint8_t reg)
{
  uint8_t value = 0;
  _pWire->beginTransmission(this->_I2C_addr);
#if ARDUINO >= 100
  _pWire->write(reg);
#else
  _pWire->send(reg);
#endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
  while (!_pWire->available()) {
  };
#if ARDUINO >= 100
  value = _pWire->read();
#else
  value = _pWire->receive();
#endif
  return value;
}

int16_t QMC5883L::readRegister16(uint8_t reg)
{
  int16_t value = 0;
  uint8_t vha, vla;
  _pWire->beginTransmission(this->_I2C_addr);
#if ARDUINO >= 100
  _pWire->write(reg);
#else
  _pWire->send(reg);
#endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)2);
  while (!_pWire->available()) {
  };
  if (ICType == IC_HMC5883L) {
#if ARDUINO >= 100
    vha = _pWire->read();
    vla = _pWire->read();
#else
    vha = _pWire->receive();
    vla = _pWire->receive();
#endif
  } else {
#if ARDUINO >= 100
    vla = _pWire->read();
    vha = _pWire->read();
#else
    vla = _pWire->receive();
    vha = _pWire->receive();
#endif
  }
  value = vha << 8 | vla;
  return value;
}
