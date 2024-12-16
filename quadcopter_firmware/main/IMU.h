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

#define NUM_CALIBRATION_SAMPLES_PER_AXIS 10000
#define CALIB_MEDIAN_FILTER_WINDOW 20

struct calib_data_t
{
  std::function<void(CalibrationRequest)> requestHandler;
  CalibrationRequest stage;
  bool calibrationFailed;
  bool awaitingResponse;
  std::array<int64_t, 3> accelCurrentSums;
  std::array<int64_t, 3> gyroCurrentSums;
  int16_t currentStageSamples;

  std::array<MedianFilter<int16_t>, 3> accelMedianFilters = {
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW)};

  std::array<MedianFilter<int16_t>, 3> gyroMedianFilters = {
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW),
      MedianFilter<int16_t>(CALIB_MEDIAN_FILTER_WINDOW)};

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

 private:
  SPIClass _spi;
  ICM42688 *_imu;
  std::function<void(imu_update_t)> _updateHandler;
  imu_update_t _offsets;
  imu_update_t _mostRecentUpdate;
  bool _calibrated = false;
  bool _accelerometerCalibrationInProgress = false;
  calib_data_t _accelCalibrationData;
  void _continueCalibration(imu_update_t update);
  PersistentKeyValueStore *_persistentKvStore;
};

#endif