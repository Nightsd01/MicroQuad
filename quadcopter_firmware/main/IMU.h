#pragma once

#ifndef MATLAB_SIM

#include <SPI.h>

#include <array>
#include <functional>
#include <map>
#include <vector>

#include "CalibrationEvent.h"
#include "Filters/MedianFilter.h"
#include "ICM42688.h"
#include "PersistentKeyValueStore.h"

#define NUM_CALIBRATION_SAMPLES_PER_AXIS 1000
#define CALIB_MEDIAN_FILTER_WINDOW 20
#define QUICK_GYRO_CALIBRATION_DURATION_MILLIS 2000

struct calib_data_t
{
  std::function<void(CalibrationRequest)> requestHandler;
  CalibrationRequest stage;
  bool calibrationFailed;
  bool awaitingResponse;
  std::array<int64_t, 3> accelCurrentSums;
  std::array<int64_t, 3> gyroCurrentSums;
  int16_t currentStageSamples;

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
  std::map<CalibrationResponse, std::function<void(void)>> calibrationHandlers(
      std::function<void(CalibrationRequest)> requestHandler);
  bool completedQuickCalibration = false;

  // Called right after arming the quadcopter, and will do a short gyro calibration
  // Assumes the quadcopter is sitting stationary when called
  void beginQuickGyroCalibration(void);
  void cancelQuickGyroCalibration(void);
  void completeQuickGyroCalibration(void);
  bool isQuickGyroCalibrationInProgress = false;

 private:
  SPIClass _spi;
  ICM42688 *_imu;
  std::function<void(imu_update_t)> _updateHandler;
  imu_update_t _offsets;
  imu_update_t _mostRecentUpdate;
  bool _calibrated = false;
  bool _accelerometerCalibrationInProgress = false;
  calib_data_t _calibrationData;
  void _continueCalibration(imu_update_t update);
  PersistentKeyValueStore *_persistentKvStore;
  bool _quickGyroCalibration = false;
  int64_t _startedQuickGyroCalibrationTimeMillis = 0;
  std::array<int64_t, 3> _quickCalibrationGyroSums;  // x, y, z
  int64_t _numQuickGyroCalibrationSamples = 0;
};

#endif  // MATLAB_SIM