#include "IMU.h"

#include <Logger.h>

static bool _gotInterrupt;
static void handleInterrupt(void) { _gotInterrupt = true; }

/**
 * We need to account for the full scale range when using the gyroscope values
 * For example if we select GYRO_1000DPS, we need to divide the gyro readings by
 * the corresponding sensitivity of 32.8 to get the "degrees per second"
 reading. ±250 dps: 131 LSB/dps ±500 dps: 65.5 LSB/dps ±1000 dps: 32.8 LSB/dps
    ±2000 dps: 16.4 LSB/dps
*/
#define GYRO_FSR ICM42688::GyroFS::dps2000
#define GYRO_SENSITIVITY 16.4
#define UPDATE_RATE_GYRO_HZ ICM42688::ODR::odr4k
#define UPDATE_RATE_ACCEL_HZ ICM42688::ODR::odr4k

#define ACCEL_FSR ICM42688::AccelFS::gpm16

/**
 * These calibration values were determined by sitting the device on a flat
 * surface and averaging the gyroscope x, y, and z readings for a full minute
 */
#define GYRO_OFFSET_X -36.87218045
#define GYRO_OFFSET_Y 16.21804511
#define GYRO_OFFSET_Z 15.26691729

#define ACCEL_OFFSET_X 45.55639098
#define ACCEL_OFFSET_Y -3.597744361
#define ACCEL_OFFSET_Z 65.79

IMU::IMU(uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sclkPin,
         uint8_t interruptPin, std::function<void(imu_update_t)> updateHandler,
         bool *success) {
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  _spi.begin(sclkPin, misoPin, mosiPin, csPin);

  _imu = new ICM42688(_spi, csPin);
  _imu->setAccelFS(ACCEL_FSR);
  _imu->setGyroFS(GYRO_FSR);
  _updateHandler = updateHandler;

  const float fsrVals[4] = {2.0f, 4.0f, 8.0f, 16.0f};
  _accelScale = fsrVals[(int)(ACCEL_FSR)] / 32768.0f;

  if (_imu->begin() < 0) {
    LOG_ERROR("Failed to initialize ICM42688P IMU");
    return;
  }

  _imu->setGyroODR(UPDATE_RATE_GYRO_HZ);
  _imu->setAccelODR(UPDATE_RATE_ACCEL_HZ);

  *success = true;

  pinMode(interruptPin, INPUT);
  attachInterrupt(interruptPin, handleInterrupt, RISING);
  _imu->enableDataReadyInterrupt();
}

void IMU::loopHandler(void) {
  if (_gotInterrupt) {
    _imu->getAGT();
    _imu->getRawAGT();
    imu_update_t update;
    update.accel_raw_x = _imu->rawAccX() - ACCEL_OFFSET_X;
    update.accel_raw_y = _imu->rawAccY() - ACCEL_OFFSET_Y;
    update.accel_raw_z = _imu->rawAccZ() - ACCEL_OFFSET_Z;
    update.accel_x = _imu->accX();
    update.accel_y = _imu->accY();
    update.accel_z = _imu->accZ();
    update.gyro_raw_x = _imu->rawGyrX();
    update.gyro_raw_y = _imu->rawGyrY();
    update.gyro_raw_z = _imu->rawGyrZ();
    update.gyro_x = _imu->gyrX();
    update.gyro_y = _imu->gyrY();
    update.gyro_z = _imu->gyrZ();

    _updateHandler(update);

    _mostRecentUpdate = update;

    // reset the interrupt pin flag
    _gotInterrupt = false;
  }
}

void IMU::calibrate(CalibrationAxis axis, int val) {
  _calibrated = true;
  switch (axis) {
    case CalibrationAxis::x:
      _offsets.gyro_x = (float)val;
      break;
    case CalibrationAxis::y:
      _offsets.gyro_y = (float)val;
      break;
    case CalibrationAxis::z:
      _offsets.gyro_z = (float)val;
      break;
  }
}