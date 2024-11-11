#ifndef IMU_h
#define IMU_h

#include <SPI.h>

#include <functional>

#include "ICM42688.h"

struct imu_update_t
{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
  int16_t accel_raw_x, accel_raw_y, accel_raw_z;
};

// NOTE: Keep this in sync with BLEController.swift's struct CalibrationData
struct calibration_data_t
{
  float gyro_biases[3];
  float accel_biases[3];
  float accel_scales[3];
  int16_t gyro_offsets[3];
  int16_t accel_offsets[3];

  // must be the last value/s - the client app doesn't care about data after this point
  bool success;

  static calibration_data_t createError()
  {
    calibration_data_t data = {};  // Zero-initialize all fields
    data.success = false;
    return data;
  }
};

class IMU
{
 public:
  IMU(uint8_t csPin,
      uint8_t misoPin,
      uint8_t mosiPin,
      uint8_t sclkPin,
      uint8_t interruptPin,
      std::function<void(imu_update_t)> updateHandler,
      bool *success);
  void loopHandler(void);
  calibration_data_t calibrate(void);

 private:
  SPIClass _spi;
  ICM42688 *_imu;
  std::function<void(imu_update_t)> _updateHandler;
  imu_update_t _offsets;
  imu_update_t _mostRecentUpdate;
  bool _calibrated;
  float _accelScale;
};

#endif