#ifndef IMU_h
#define IMU_h

#include <SPI.h>
#include <functional>

#include "mpu6x00.h"

struct imu_update_t {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
};

class IMU {
    public:
        IMU(uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sclkPin, uint8_t interruptPin, std::function<void(imu_update_t)> updateHandler, bool *success);
        void loopHandler(void);
    private:
        SPIClass _spi;
        Mpu6000 *_imu;
        std::function<void(imu_update_t)> _updateHandler;
};

#endif 