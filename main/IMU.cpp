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
#define GYRO_FSR Mpu6x00::GYRO_2000DPS
#define GYRO_SENSITIVITY 16.4

#define ACCEL_FSR Mpu6x00::ACCEL_16G

/**
 * These calibration values were determined by sitting the device on a flat surface
 * and averaging the gyroscope x, y, and z readings for a full minute
*/
#define GYRO_OFFSET_X -36.87218045
#define GYRO_OFFSET_Y 16.21804511
#define GYRO_OFFSET_Z 15.26691729

#define ACCEL_OFFSET_X 45.55639098
#define ACCEL_OFFSET_Y -3.597744361
#define ACCEL_OFFSET_Z 65.79

IMU::IMU(uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sclkPin, uint8_t interruptPin, std::function<void(imu_update_t)> updateHandler, bool *success)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    _spi.begin(sclkPin, misoPin, mosiPin, csPin);

    _imu = new Mpu6000(_spi, csPin, GYRO_FSR, ACCEL_FSR);
    _updateHandler = updateHandler;

    const float fsrVals[4] = {2.0f, 4.0f, 8.0f, 16.0f};
    _accelScale = fsrVals[(int)(ACCEL_FSR)] / 32768.0f;

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
        update.accel_raw_x = _imu->getRawAccelX() - ACCEL_OFFSET_X;
        update.accel_raw_y = _imu->getRawAccelY() - ACCEL_OFFSET_Y;
        update.accel_raw_z = _imu->getRawAccelZ() - ACCEL_OFFSET_Z;
        update.accel_x = update.accel_raw_x * _accelScale;
        update.accel_y = update.accel_raw_y * _accelScale;
        update.accel_z = update.accel_raw_z * _accelScale;
        _imu->getGyro(update.gyro_x, update.gyro_y, update.gyro_z);
        update.gyro_raw_x = _imu->getRawGyroX();
        update.gyro_raw_y = _imu->getRawGyroY();
        update.gyro_raw_z = _imu->getRawGyroZ();

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