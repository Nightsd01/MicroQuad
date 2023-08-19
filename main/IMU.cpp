#include "IMU.h"

#include <Logger.h>

static bool _gotInterrupt;
static void handleInterrupt(void)
{
    _gotInterrupt = true;
}

IMU::IMU(uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sclkPin, uint8_t interruptPin, std::function<void(imu_update_t)> updateHandler, bool *success)
{
    _spi.begin(sclkPin, misoPin, mosiPin, csPin);

    _imu = new Mpu6000(_spi, csPin);
    _updateHandler = updateHandler;

    if (!_imu->begin(false /* setupAuxDevice */)) {
        LOG_ERROR("Failed to initialize MPU6000 IMU");
        return;
    }

    *success = true;

    pinMode(interruptPin, INPUT);
    attachInterrupt(interruptPin, handleInterrupt, RISING);
}

void IMU::loopHandler(void)
{
    if (_gotInterrupt) {
        static uint64_t updates = 0;
        static uint64_t lastPrintMillis = 0;
        updates++;
        if (millis() - lastPrintMillis > 1000) {
            lastPrintMillis = millis();
            // LOG_INFO("IMU running at %lld hz", updates);
            updates = 0;
        }

        _imu->readSensor();

        imu_update_t update;
        _imu->getAccel(update.accel_x, update.accel_y, update.accel_z);
        _imu->getGyro(update.gyro_x, update.gyro_y, update.gyro_z);
        _imu->getMag(update.mag_x, update.mag_y, update.mag_z);
        _updateHandler(update);

        // reset the interrupt pin flag
        _gotInterrupt = false;
    }
}