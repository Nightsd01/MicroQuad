/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#pragma once

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

class Mpu6x00
{
 public:
  typedef enum
  {

    GYRO_250DPS,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS

  } gyroFsr_e;

  typedef enum
  {

    ACCEL_2G,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G

  } accelFsr_e;

  /**
   * Returns true on success, false on failure.
   */
  bool begin(bool setupAuxDevice)
  {
    pinMode(m_csPin, OUTPUT);

    writeRegister(REG_PWR_MGMT_1, BIT_RESET);
    delay(100);

    writeRegister(REG_PWR_MGMT_1, BIT_CLK_SEL_PLLGYROZ);
    delayMicroseconds(7);

    writeRegister(REG_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    writeRegister(REG_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    writeRegister(REG_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    if (whoAmI() != m_deviceId) {
      return false;
    }

    writeRegister(REG_GYRO_CONFIG, (uint8_t)(m_gyroFsr << 3));
    delayMicroseconds(15);

    writeRegister(REG_ACCEL_CONFIG, (uint8_t)(m_accelFsr << 3));
    delayMicroseconds(15);

    writeRegister(REG_INT_PIN_CFG, 0x10);
    delayMicroseconds(15);

    writeRegister(REG_INT_ENABLE, BIT_RAW_RDY_EN);
    delayMicroseconds(15);

    writeRegister(REG_CONFIG, 0);
    delayMicroseconds(1);

    if (!setupAuxDevice) {
      return true;
    }

    _magnetometer = true;

    // configure the auxillary i2c bus to use the external sensor
    writeRegister(REG_I2C_MST_CTRL, 29);

    // Write configuration to the QMC5883L
    writeRegister(REG_I2C_SLV0_ADDR, EXT_SENS_QMC5883L_I2C_ADDR);
    delayMicroseconds(1);
    writeRegister(REG_I2C_SLV0_REG, EXT_SENS_CTRL1_ADDR);
    delayMicroseconds(1);
    writeRegister(
        REG_I2C_SLV0_DO,
        BIT_QMC5883L_CTRL_MODE_CONTINUOUS | BIT_QMC5883L_CTRL_RATE_200HZ | BIT_QMC5883L_CTRL_SCALE_8G);
    delayMicroseconds(1);
    writeRegister(REG_I2C_SLV0_CTRL, 0x80 | 0x01);
    delayMicroseconds(1);

    // Set FIFO enable for everything except SLV1 and SLV2
    writeRegister(REG_FIFO_EN, 255);
    delayMicroseconds(1);

    writeRegister(REG_I2C_SLV0_CTRL, 0x80);
    delayMicroseconds(1);

    // configure the auxillary i2c bus to use the external sensor
    writeRegister(REG_I2C_MST_CTRL, 29);

    // Bits 0-6 are the address, bit 7 is to enable the external sensor bus
    writeRegister(REG_I2C_SLV0_ADDR, EXT_SENS_QMC5883L_I2C_ADDR | 0x80);
    delayMicroseconds(1);

    return true;
  }

  void readSensor(void)
  {
    readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14, SPI_FULL_CLK_HZ);
    if (_magnetometer) {
      readRegisters(REG_EXT_SENS_DATA_BEGIN, ext_sensor_buffer, EXT_SENS_DATA_LEN, SPI_FULL_CLK_HZ);
    }
  }

  void getGyro(float &gx, float &gy, float &gz)
  {
    gx = getRawValue(9) * m_gyroScale;
    gy = getRawValue(11) * m_gyroScale;
    gz = getRawValue(13) * m_gyroScale;
  }

  void getAccel(float &ax, float &ay, float &az)
  {
    ax = getRawValue(1) * m_accelScale;
    ay = getRawValue(3) * m_accelScale;
    az = getRawValue(5) * m_accelScale;
  }

  void getMag(float &mx, float &my, float &mz)
  {
    mx = getExtSensX();
    my = getExtSensY();
    mz = getExtSensZ();
  }

  float getAccelX(void) { return getAccelValue(1); }

  float getAccelY(void) { return getAccelValue(3); }

  float getAccelZ(void) { return getAccelValue(5); }

  float getGyroX(void) { return getGyroValue(9); }

  float getGyroY(void) { return getGyroValue(11); }

  float getGyroZ(void) { return getGyroValue(13); }

  int16_t getRawAccelX(void) { return getRawValue(1); }

  int16_t getRawAccelY(void) { return getRawValue(3); }

  int16_t getRawAccelZ(void) { return getRawValue(5); }

  int16_t getRawGyroX(void) { return getRawValue(9); }

  int16_t getRawGyroY(void) { return getRawValue(11); }

  int16_t getRawGyroZ(void) { return getRawValue(13); }

  int16_t getExtSensX(void) { return (uint16_t)((ext_sensor_buffer[0] << 8) | ext_sensor_buffer[1]); }

  int16_t getExtSensY(void) { return (uint16_t)((ext_sensor_buffer[2] << 8) | ext_sensor_buffer[3]); }

  int16_t getExtSensZ(void) { return (uint16_t)((ext_sensor_buffer[4] << 8) | ext_sensor_buffer[5]); }

  void setGyroOffsetX(int16_t x)
  {
    writeRegister(REG_XG_OFFS_USRH, (uint8_t)((x >> 8) & 0xFF));
    delayMicroseconds(1);
    writeRegister(REG_XG_OFFS_USRL, (uint8_t)(x & 0xFF));
    delayMicroseconds(1);
  }

  void setGyroOffsetY(int16_t y)
  {
    writeRegister(REG_YG_OFFS_USRH, (uint8_t)((y >> 8) & 0xFF));
    delayMicroseconds(1);
    writeRegister(REG_YG_OFFS_USRL, (uint8_t)(y & 0xFF));
    delayMicroseconds(1);
  }

  void setGyroOffsetZ(int16_t z)
  {
    writeRegister(REG_ZG_OFFS_USRH, (uint8_t)((z >> 8) & 0xFF));
    delayMicroseconds(1);
    writeRegister(REG_ZG_OFFS_USRL, (uint8_t)(z & 0xFF));
    delayMicroseconds(1);
  }

  void setGyroOffsets(int16_t x, int16_t y, int16_t z)
  {
    // Write X
    setGyroOffsetX(x);
    // Write Y
    setGyroOffsetY(y);
    // Write Z
    setGyroOffsetZ(z);
  }

 protected:
  Mpu6x00(
      const uint8_t deviceId, SPIClass &spi, const uint8_t csPin, const gyroFsr_e gyroFsr, const accelFsr_e accelFsr)
  {
    init(deviceId, &spi, csPin, gyroFsr, accelFsr);
  }

  Mpu6x00(const uint8_t deviceId, const uint8_t csPin, const gyroFsr_e gyroFsr, const accelFsr_e accelFsr = ACCEL_16G)
  {
    init(deviceId, &SPI, csPin, gyroFsr, accelFsr);
  }

 private:
  bool _magnetometer;

  // Configuration bits
  static const uint8_t BIT_RAW_RDY_EN = 0x01;
  static const uint8_t BIT_CLK_SEL_PLLGYROZ = 0x01;
  static const uint8_t BIT_I2C_IF_DIS = 0x10;
  static const uint8_t BIT_RESET = 0x80;

  // Slave i2c sensor configuration bits
  static const uint8_t BIT_I2C_SLV0_EN = 0x80;
  static const uint8_t BIT_QMC5883L_CTRL_MODE_CONTINUOUS = 0x01;
  static const uint8_t BIT_QMC5883L_CTRL_RATE_200HZ = 0xC;
  static const uint8_t BIT_QMC5883L_CTRL_SCALE_8G = 0x10;

  // Registers
  static const uint8_t REG_SMPLRT_DIV = 0x19;
  static const uint8_t REG_CONFIG = 0x1A;
  static const uint8_t REG_GYRO_CONFIG = 0x1B;
  static const uint8_t REG_ACCEL_CONFIG = 0x1C;
  static const uint8_t REG_INT_PIN_CFG = 0x37;
  static const uint8_t REG_INT_ENABLE = 0x38;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
  static const uint8_t REG_USER_CTRL = 0x6A;
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_PWR_MGMT_2 = 0x6C;
  static const uint8_t REG_WHO_AM_I = 0x75;
  static const uint8_t REG_EXT_SENS_DATA_BEGIN = 0x49;
  static const uint8_t REG_EXT_SENS_DATA_END = 0x60;
  static const uint8_t REG_I2C_SLV0_ADDR = 0x25;
  static const uint8_t REG_I2C_SLV0_REG = 0x26;
  static const uint8_t REG_I2C_SLV0_CTRL = 0x27;
  static const uint8_t REG_I2C_MST_CTRL = 0x24;
  static const uint8_t REG_FIFO_EN = 0x23;
  static const uint8_t EXT_SENS_CTRL1_ADDR = 0x09;
  static const uint8_t REG_I2C_SLV0_DO = 0x63;
  static const uint8_t REG_XG_OFFS_USRH = 0x13;
  static const uint8_t REG_XG_OFFS_USRL = 0x14;
  static const uint8_t REG_YG_OFFS_USRH = 0x15;
  static const uint8_t REG_YG_OFFS_USRL = 0x16;
  static const uint8_t REG_ZG_OFFS_USRH = 0x17;
  static const uint8_t REG_ZG_OFFS_USRL = 0x18;

  static const uint8_t EXT_SENS_QMC5883L_I2C_ADDR = 0x0D;
  static const uint8_t EXT_SENS_DATA_LEN = 24;
  static const uint8_t EXT_SENS_I2C_400KHZ_CLK_VAL = 31;
  static const uint32_t SPI_FULL_CLK_HZ = 20000000;
  static const uint32_t SPI_INIT_CLK_HZ = 1000000;

  SPIClass *m_spi;

  uint8_t m_csPin;

  uint8_t m_deviceId;

  gyroFsr_e m_gyroFsr;
  accelFsr_e m_accelFsr;

  float m_gyroScale;
  float m_accelScale;

  uint8_t m_buffer[15];
  uint8_t ext_sensor_buffer[24];

  void init(
      const uint8_t deviceId, SPIClass *spi, const uint8_t csPin, const gyroFsr_e gyroFsr, const accelFsr_e accelFsr)
  {
    m_deviceId = deviceId;
    m_spi = spi;
    m_csPin = csPin;

    m_gyroFsr = gyroFsr;
    m_accelFsr = accelFsr;

    // float gscale[] = {250., 500., 1000., 2000.};
    m_gyroScale = 1.0;  // gscale[gyroFsr] / 32768.;

    float ascale[] = {2., 4., 8., 16.};
    m_accelScale = ascale[accelFsr] / 32768.;
  }

  void writeRegister(const uint8_t reg, const uint8_t val)
  {
    m_spi->beginTransaction(SPISettings(SPI_INIT_CLK_HZ, MSBFIRST, SPI_MODE3));

    digitalWrite(m_csPin, LOW);
    m_spi->transfer(reg);
    m_spi->transfer(val);
    digitalWrite(m_csPin, HIGH);

    m_spi->endTransaction();
  }

  void readRegisters(const uint8_t addr, uint8_t *buffer, const uint8_t count, const uint32_t spiClkHz)
  {
    m_spi->beginTransaction(SPISettings(spiClkHz, MSBFIRST, SPI_MODE3));

    digitalWrite(m_csPin, LOW);

    buffer[0] = addr | 0x80;
    m_spi->transfer(buffer, count + 1);

    digitalWrite(m_csPin, HIGH);

    m_spi->endTransaction();
  }

  uint8_t whoAmI(void)
  {
    uint8_t buf[2] = {};
    readRegisters(REG_WHO_AM_I, buf, 1, SPI_INIT_CLK_HZ);
    return buf[1];
  }

  int16_t getRawValue(const uint8_t offset) { return (((int16_t)m_buffer[offset]) << 8) | m_buffer[offset + 1]; }

  float getAccelValue(const uint8_t k) { return getFloatValue(k, m_accelScale); }

  float getGyroValue(const uint8_t k) { return getFloatValue(k, m_gyroScale); }

  float getFloatValue(const uint8_t k, const float scale) { return getRawValue(k) * scale; }

};  // class Mpu6x00

class Mpu6000 : public Mpu6x00
{
 public:
  Mpu6000(
      SPIClass &spi, const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_500DPS, const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x68, spi, csPin, gyroFsr, accelFsr)
  {
  }

  Mpu6000(const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_500DPS, const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x68, SPI, csPin, gyroFsr, accelFsr)
  {
  }
};

class Mpu6500 : public Mpu6x00
{
 public:
  Mpu6500(
      SPIClass &spi, const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_500DPS, const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x70, spi, csPin, gyroFsr, accelFsr)
  {
  }

  Mpu6500(const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_500DPS, const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x70, SPI, csPin, gyroFsr, accelFsr)
  {
  }
};

#endif  // MATLAB_SIM