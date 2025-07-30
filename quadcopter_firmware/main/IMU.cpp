#include "IMU.h"

#ifndef MATLAB_SIM

#include <Logger.h>

#include "PersistentKeysCommon.h"

static volatile bool _gotInterrupt;
static void handleInterrupt(void) { _gotInterrupt = true; }

/**
 * We need to account for the full scale range when using the gyroscope values
 * For example if we select GYRO_1000DPS, we need to divide the gyro readings by
 * the corresponding sensitivity of 32.8 to get the "degrees per second"
 reading. ±250 dps: 131 LSB/dps ±500 dps: 65.5 LSB/dps ±1000 dps: 32.8 LSB/dps
    ±2000 dps: 16.4 LSB/dps
*/
#define GYRO_FSR ICM42688::GyroFS::dps500
#define UPDATE_RATE_GYRO_HZ ICM42688::ODR::odr1k
#define UPDATE_RATE_ACCEL_HZ ICM42688::ODR::odr1k

#define ACCEL_FSR ICM42688::AccelFS::gpm4

#define INT16T_MAX 32768.0f

// Macro to load and set IMU offsets from persistent storage
#define ATTEMPT_LOAD_IMU_OFFSET(sensorPrefix, axis, persistentKey, sensorName, offsetFlag, type) \
  do {                                                                                           \
    if (persistentKvStore->hasValueForKey(persistentKey)) {                                      \
      const type offset = persistentKvStore->getValue<type>(persistentKey);                      \
      LOG_INFO("Setting %c " sensorName " offset: %d", #axis[0], offset);                        \
      _imu->set##sensorPrefix##axis##Offset(offset);                                             \
      offsetFlag = true;                                                                         \
    }                                                                                            \
  } while (0)

static float gyroDPS(int16_t rawValue)
{
  switch (GYRO_FSR) {
    case ICM42688::GyroFS::dps15_625:
      return (15.625f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps31_25:
      return (31.25f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps62_5:
      return (62.5f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps125:
      return (125.0f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps250:
      return (250.0f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps500:
      return (500.0f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps1000:
      return (1000.0f / INT16T_MAX) * rawValue;
    case ICM42688::GyroFS::dps2000:
      return (2000.0f / INT16T_MAX) * rawValue;
  }
}

IMU::IMU(
    uint8_t csPin,
    uint8_t misoPin,
    uint8_t mosiPin,
    uint8_t sclkPin,
    uint8_t interruptPin,
    std::function<void(imu_update_t)> updateHandler,
    PersistentKeyValueStore *persistentKvStore,
    bool *success)
{
  _persistentKvStore = persistentKvStore;
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  _spi.begin(sclkPin, misoPin, mosiPin, csPin);

  _imu = new ICM42688(_spi, csPin);

  bool setAnyOffset = false;

  // Load accelerometer offsets
  ATTEMPT_LOAD_IMU_OFFSET(Acc, X, PersistentKeysCommon::ACCEL_OFFSET_X, "accel", setAnyOffset, int64_t);
  ATTEMPT_LOAD_IMU_OFFSET(Acc, Y, PersistentKeysCommon::ACCEL_OFFSET_Y, "accel", setAnyOffset, int64_t);
  ATTEMPT_LOAD_IMU_OFFSET(Acc, Z, PersistentKeysCommon::ACCEL_OFFSET_Z, "accel", setAnyOffset, int64_t);

  // Load gyroscope offsets
  ATTEMPT_LOAD_IMU_OFFSET(Gyr, X, PersistentKeysCommon::GYRO_OFFSET_X, "gyro", setAnyOffset, int16_t);
  ATTEMPT_LOAD_IMU_OFFSET(Gyr, Y, PersistentKeysCommon::GYRO_OFFSET_Y, "gyro", setAnyOffset, int16_t);
  ATTEMPT_LOAD_IMU_OFFSET(Gyr, Z, PersistentKeysCommon::GYRO_OFFSET_Z, "gyro", setAnyOffset, int16_t);

  if (!setAnyOffset) {
    LOG_WARN("No IMU offsets found in persistent storage, please calibrate the device");
  }

  _imu->setAccelFS(ACCEL_FSR);
  _imu->setGyroFS(GYRO_FSR);
  _updateHandler = updateHandler;

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

void IMU::loopHandler(void)
{
  if (_gotInterrupt) {
    _imu->getAGT();
    _imu->getRawAGT();
    imu_update_t update;
    update.accel_raw_x = _imu->rawAccX();
    update.accel_raw_y = _imu->rawAccY();
    update.accel_raw_z = _imu->rawAccZ();
    update.accel_x = _imu->accX();
    update.accel_y = _imu->accY();
    update.accel_z = _imu->accZ();
    update.gyro_raw_x = _imu->rawGyrX();
    update.gyro_raw_y = _imu->rawGyrY();
    update.gyro_raw_z = _imu->rawGyrZ();
    update.gyro_x = _imu->gyrX();
    update.gyro_y = _imu->gyrY();
    update.gyro_z = _imu->gyrZ();
    update.gyro_dps_x = gyroDPS(update.gyro_x);
    update.gyro_dps_y = gyroDPS(update.gyro_y);
    update.gyro_dps_z = gyroDPS(update.gyro_z);

    if (isQuickGyroCalibrationInProgress) {
      _quickCalibrationGyroSums[0] += update.gyro_raw_x;
      _quickCalibrationGyroSums[1] += update.gyro_raw_y;
      _quickCalibrationGyroSums[2] += update.gyro_raw_z;
      _numQuickGyroCalibrationSamples++;

      if (millis() - _startedQuickGyroCalibrationTimeMillis >= QUICK_GYRO_CALIBRATION_DURATION_MILLIS) {
        completeQuickGyroCalibration();
      }
    }

    _updateHandler(update);

    _mostRecentUpdate = update;

    // reset the interrupt pin flag
    _gotInterrupt = false;

    if (_accelerometerCalibrationInProgress && !_calibrationData.awaitingResponse) {
      _continueCalibration(update);
    }
  }
}

void IMU::beginQuickGyroCalibration(void)
{
  // Need to reset the existing offsets before starting the calibration
  _imu->setGyrXOffset(0.0f);
  _imu->setGyrYOffset(0.0f);
  _imu->setGyrZOffset(0.0f);
  isQuickGyroCalibrationInProgress = true;
  _startedQuickGyroCalibrationTimeMillis = millis();
  _quickCalibrationGyroSums = {0, 0, 0};
}

void IMU::cancelQuickGyroCalibration(void)
{
  isQuickGyroCalibrationInProgress = false;
  _numQuickGyroCalibrationSamples = 0;
  _quickCalibrationGyroSums = {0, 0, 0};
  LOG_INFO("Quick gyro calibration cancelled");
}

void IMU::completeQuickGyroCalibration(void)
{
  isQuickGyroCalibrationInProgress = false;
  if (_numQuickGyroCalibrationSamples == 0) {
    LOG_ERROR("No samples taken for quick gyro calibration");
    return;
  }
  float xOffset = (float)_quickCalibrationGyroSums[0] / (float)_numQuickGyroCalibrationSamples;
  float yOffset = (float)_quickCalibrationGyroSums[1] / (float)_numQuickGyroCalibrationSamples;
  float zOffset = (float)_quickCalibrationGyroSums[2] / (float)_numQuickGyroCalibrationSamples;

  LOG_INFO(
      "Quick gyro calibration complete! %i samples, %.2f, %.2f, %.2f",
      (int)_numQuickGyroCalibrationSamples,
      xOffset,
      yOffset,
      zOffset);
  int16_t averages[3] = {
      (int16_t)(roundf(xOffset)),
      (int16_t)(roundf(yOffset)),
      (int16_t)(roundf(zOffset)),
  };
  _imu->setGyrXOffset(averages[0]);
  _imu->setGyrYOffset(averages[1]);
  _imu->setGyrZOffset(averages[2]);
  completedQuickCalibration = true;
}

void IMU::_continueCalibration(imu_update_t update)
{
  const int16_t vals[3] = {update.accel_raw_x, update.accel_raw_y, update.accel_raw_z};

  // Calculate running median sum for X, Y, and Z axes
  for (int i = 0; i < 3; i++) {
    _calibrationData.accelCurrentSums[i] += vals[i];
  }
  if (_calibrationData.stage == CalibrationRequest::PlaceFlat) {
    const int16_t gyroVals[3] = {update.gyro_raw_x, update.gyro_raw_y, update.gyro_raw_z};
    for (int i = 0; i < 3; i++) {
      _calibrationData.gyroCurrentSums[i] += gyroVals[i];
    }
  }

  if (++_calibrationData.currentStageSamples >= NUM_CALIBRATION_SAMPLES_PER_AXIS) {
    // We're done with this stage
    LOG_INFO("Completed stage %d", (int)_calibrationData.stage);
    _accelerometerCalibrationInProgress = false;

    if (_calibrationData.stage == CalibrationRequest::PlaceFlat) {
      int16_t averages[3];
      for (int i = 0; i < 3; i++) {
        averages[i] = (int16_t)(_calibrationData.gyroCurrentSums[i] / NUM_CALIBRATION_SAMPLES_PER_AXIS);
      }

      _persistentKvStore->setValue<int16_t>(PersistentKeysCommon::GYRO_OFFSET_X, averages[0]);
      _persistentKvStore->setValue<int16_t>(PersistentKeysCommon::GYRO_OFFSET_Y, averages[1]);
      _persistentKvStore->setValue<int16_t>(PersistentKeysCommon::GYRO_OFFSET_Z, averages[2]);

      _imu->setGyrXOffset(averages[0]);
      _imu->setGyrYOffset(averages[1]);
      _imu->setGyrZOffset(averages[2]);

      LOG_INFO("Gyro calibration complete: %d, %d, %d", averages[0], averages[1], averages[2]);
    }

    _calibrationData.offsets[_calibrationData.stage] = {
        (int64_t)(_calibrationData.accelCurrentSums[0] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
        (int64_t)(_calibrationData.accelCurrentSums[1] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
        (int64_t)(_calibrationData.accelCurrentSums[2] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
    };

    if (_calibrationData.stage == CalibrationRequest::RollLeft) {
      // Final Stage Completed
      if (_calibrationData.offsets.size() != NUM_ACCELGYRO_CALIBRATION_STAGES) {
        LOG_ERROR("Accelerometer calibration failed: Not all stages completed.");
        _calibrationData.requestHandler(CalibrationRequest::Failed);
        return;
      }

      int64_t offsets[3] = {0, 0, 0};
      for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < NUM_ACCELGYRO_CALIBRATION_STAGES; i++) {
          offsets[axis] += _calibrationData.offsets[(CalibrationRequest)i][axis];
        }
        offsets[axis] /= NUM_ACCELGYRO_CALIBRATION_STAGES;
      }

      LOG_INFO("Accelerometer calibration complete:");
      LOG_INFO("X Offset: %lld", offsets[0]);
      LOG_INFO("Y Offset: %lld", offsets[1]);
      LOG_INFO("Z Offset: %lld", offsets[2]);

      _imu->setAccXOffset(offsets[0]);
      _imu->setAccYOffset(offsets[1]);
      _imu->setAccZOffset(offsets[2]);

      _persistentKvStore->setValue<int64_t>(PersistentKeysCommon::ACCEL_OFFSET_X, offsets[0]);
      _persistentKvStore->setValue<int64_t>(PersistentKeysCommon::ACCEL_OFFSET_Y, offsets[1]);
      _persistentKvStore->setValue<int64_t>(PersistentKeysCommon::ACCEL_OFFSET_Z, offsets[2]);

      _calibrationData.requestHandler(CalibrationRequest::Complete);
    } else {
      // continue to the next stage
      const int nextStage = (int)(_calibrationData.stage) + 1;
      _calibrationData.stage = (CalibrationRequest)nextStage;
      _calibrationData.currentStageSamples = 0;
      _calibrationData.accelCurrentSums = {0, 0, 0};
      // We will wait for the user to send CalibrationResponse::Continue before continuing
      _calibrationData.awaitingResponse = true;
      _calibrationData.requestHandler(_calibrationData.stage);
    }
  }
}

std::map<CalibrationResponse, std::function<void(void)>> IMU::calibrationHandlers(
    std::function<void(CalibrationRequest)> requestHandler)
{
  std::map<CalibrationResponse, std::function<void(void)>> responseHandlers;

  responseHandlers[CalibrationResponse::Start] = [this, requestHandler]() {
    // The IMU loop handler will imminently start collecting data for the first stage
    _accelerometerCalibrationInProgress = true;
    _calibrationData = {
        .requestHandler = requestHandler,
        .stage = CalibrationRequest::PlaceFlat,
        .calibrationFailed = false,
        .awaitingResponse = true,
        .accelCurrentSums = {0, 0, 0},
        .currentStageSamples = 0,
    };
    requestHandler(CalibrationRequest::PlaceFlat);
    LOG_INFO("Starting accelerometer calibration.");
  };

  responseHandlers[CalibrationResponse::Continue] = [this]() {
    _calibrationData.awaitingResponse = false;
    _accelerometerCalibrationInProgress = true;
    LOG_INFO("Continuing accelerometer calibration.");
  };

  responseHandlers[CalibrationResponse::Cancel] = [this]() {
    _calibrationData = {};
    _accelerometerCalibrationInProgress = false;
    LOG_INFO("Accelerometer calibration canceled.");
  };

  return responseHandlers;
}

#endif MATLAB_SIM