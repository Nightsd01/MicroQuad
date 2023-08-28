#include "IMU.h"

#include <Logger.h>

static bool _gotInterrupt;
static void handleInterrupt(void)
{
    _gotInterrupt = true;
}

/**
 * We need to account for the full scale range when using the gyroscope values
 * For example if we select GYRO_1000DPS, we need to divide the gyro readings by 
 * the corresponding sensitivity of 32.8 to get the "degrees per second" reading.
    ±250 dps: 131 LSB/dps
    ±500 dps: 65.5 LSB/dps
    ±1000 dps: 32.8 LSB/dps
    ±2000 dps: 16.4 LSB/dps
*/
#define GYRO_FSR Mpu6x00::GYRO_1000DPS
#define GYRO_SENSITIVITY 32.8f

/**
 * These calibration values were determined by sitting the device on a flat surface
 * and averaging the gyroscope x, y, and z readings for a full minute
*/
#define GYRO_OFFSET_X -69.675
#define GYRO_OFFSET_Y 31.70454545
#define GYRO_OFFSET_Z 28.325

IMU::IMU(uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sclkPin, uint8_t interruptPin, std::function<void(imu_update_t)> updateHandler, bool *success)
{
    _spi.begin(sclkPin, misoPin, mosiPin, csPin);

    _imu = new Mpu6000(_spi, csPin, GYRO_FSR, Mpu6x00::ACCEL_16G);
    _updateHandler = updateHandler;

    if (!_imu->begin(false /* setupAuxDevice */)) {
        LOG_ERROR("Failed to initialize MPU6000 IMU");
        return;
    }

    calibrate(CalibrationAxis::x, GYRO_OFFSET_X);
    calibrate(CalibrationAxis::y, GYRO_OFFSET_Y);
    calibrate(CalibrationAxis::z, GYRO_OFFSET_Z);


    *success = true;

    pinMode(interruptPin, INPUT);
    attachInterrupt(interruptPin, handleInterrupt, RISING);
}

void IMU::loopHandler(void)
{
    if (_gotInterrupt) {
        _imu->readSensor();

        imu_update_t update;
        _imu->getAccel(update.accel_x, update.accel_y, update.accel_z);
        _imu->getGyro(update.gyro_x, update.gyro_y, update.gyro_z);
        update.gyro_raw_x = _imu->getRawGyroX();
        update.gyro_raw_y = _imu->getRawGyroY();
        update.gyro_raw_z = _imu->getRawGyroZ();
        update.accel_raw_x = _imu->getRawAccelX();
        update.accel_raw_y = _imu->getRawAccelY();
        update.accel_raw_z = _imu->getRawAccelZ();

        if (_calibrated) {
            // TODO: Handle accelerometer calibration as well
            update.gyro_x -= _offsets.gyro_x;
            update.gyro_y -= _offsets.gyro_y;
            update.gyro_z -= _offsets.gyro_z;
        }

        update.gyro_x /= GYRO_SENSITIVITY;
        update.gyro_y /= GYRO_SENSITIVITY;
        update.gyro_z /= GYRO_SENSITIVITY;

        _updateHandler(update);

        _mostRecentUpdate = update;

        // reset the interrupt pin flag
        _gotInterrupt = false;
    }
}

void IMU::calibrate(CalibrationAxis axis, int val)
{
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