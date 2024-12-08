#ifndef IMU_h
#define IMU_h

#include <SPI.h>

#include <array>
#include <functional>
#include <map>
#include <vector>

#include "CalibrationEvent.h"
#include "Filters/MedianFilter.h"
#include "ICM42688.h"
#include "PersistentKeyValueStore.h"

#define NUM_ACCEL_CALIBRATION_SAMPLES_PER_AXIS 2000
#define ACCEL_CALIB_MEDIAN_FILTER_WINDOW 20

struct accel_calibration_data_t
{
  std::function<void(CalibrationRequest)> requestHandler;
  CalibrationRequest stage;
  bool calibrationFailed;
  bool awaitingResponse;
  std::array<int64_t, 3> currentSums;
  int16_t currentStageSamples;

  std::array<MedianFilter<int16_t>, 3> medianFilters = {
      MedianFilter<int16_t>(ACCEL_CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(ACCEL_CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(ACCEL_CALIB_MEDIAN_FILTER_WINDOW)};

  // Offsets
  std::map<CalibrationRequest, std::array<int64_t, 3>> offsets;
};

struct imu_update_t
{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
  int16_t accel_raw_x, accel_raw_y, accel_raw_z;
  float gyro_dps_x, gyro_dps_y, gyro_dps_z;
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
      PersistentKeyValueStore *persistentKvStore,
      bool *success);
  void loopHandler(void);
  calibration_data_t calibrate__deprecated(void);
  std::map<CalibrationResponse, std::function<void(void)>> calibrationHandlers(
      std::function<void(CalibrationRequest)> requestHandler);

 private:
  SPIClass _spi;
  ICM42688 *_imu;
  std::function<void(imu_update_t)> _updateHandler;
  imu_update_t _offsets;
  imu_update_t _mostRecentUpdate;
  bool _calibrated = false;
  bool _accelerometerCalibrationInProgress = false;
  accel_calibration_data_t _accelCalibrationData;
  void _continueCalibration(imu_update_t update);
  PersistentKeyValueStore *_persistentKvStore;
};

#endif