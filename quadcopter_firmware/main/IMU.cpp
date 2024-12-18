#include "IMU.h"

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

#define INT16T_MAX 32767.0f

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
  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::ACCEL_OFFSET_X)) {
    const int16_t xOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::ACCEL_OFFSET_X);
    LOG_INFO("Setting X accel offset: %d", xOffset);
    _imu->setAccXOffset(xOffset);
    setAnyOffset = true;
  }

  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::ACCEL_OFFSET_Y)) {
    const int16_t yOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::ACCEL_OFFSET_Y);
    LOG_INFO("Setting Y accel offset: %d", yOffset);
    _imu->setAccYOffset(yOffset);
    setAnyOffset = true;
  }

  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::ACCEL_OFFSET_Z)) {
    const int16_t zOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::ACCEL_OFFSET_Z);
    LOG_INFO("Setting Z accel offset: %d", zOffset);
    _imu->setAccZOffset(zOffset);
    setAnyOffset = true;
  }

  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::GYRO_OFFSET_X)) {
    const int16_t xOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::GYRO_OFFSET_X);
    LOG_INFO("Setting X gyro offset: %d", xOffset);
    _imu->setGyrXOffset(xOffset);
    setAnyOffset = true;
  }

  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::GYRO_OFFSET_Y)) {
    const int16_t yOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::GYRO_OFFSET_Y);
    LOG_INFO("Setting Y gyro offset: %d", yOffset);
    _imu->setGyrYOffset(yOffset);
    setAnyOffset = true;
  }

  if (persistentKvStore->hasValueForKey(PersistentKeysCommon::GYRO_OFFSET_Z)) {
    const int16_t zOffset = (int16_t)persistentKvStore->getIntForKey(PersistentKeysCommon::GYRO_OFFSET_Z);
    LOG_INFO("Setting Z gyro offset: %d", zOffset);
    _imu->setGyrZOffset(zOffset);
    setAnyOffset = true;
  }

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

    _updateHandler(update);

    _mostRecentUpdate = update;

    // reset the interrupt pin flag
    _gotInterrupt = false;

    if (_accelerometerCalibrationInProgress && !_accelCalibrationData.awaitingResponse) {
      _continueCalibration(update);
    }
  }
}

void IMU::_continueCalibration(imu_update_t update)
{
  const int16_t vals[3] = {update.accel_raw_x, update.accel_raw_y, update.accel_raw_z};

  // Calculate running median sum for X, Y, and Z axes
  for (int i = 0; i < 3; i++) {
    _accelCalibrationData.accelMedianFilters[i].addValue(vals[i]);
    _accelCalibrationData.accelCurrentSums[i] += _accelCalibrationData.accelMedianFilters[i].getMedian();
  }
  if (_accelCalibrationData.stage == CalibrationRequest::PlaceFlat) {
    const int16_t gyroVals[3] = {update.gyro_raw_x, update.gyro_raw_y, update.gyro_raw_z};
    for (int i = 0; i < 3; i++) {
      _accelCalibrationData.gyroMedianFilters[i].addValue(gyroVals[i]);
      _accelCalibrationData.gyroCurrentSums[i] += _accelCalibrationData.gyroMedianFilters[i].getMedian();
    }
  }

  if (++_accelCalibrationData.currentStageSamples >= NUM_CALIBRATION_SAMPLES_PER_AXIS) {
    // We're done with this stage
    LOG_INFO("Completed stage %d", (int)_accelCalibrationData.stage);
    _accelerometerCalibrationInProgress = false;

    if (_accelCalibrationData.stage == CalibrationRequest::PlaceFlat) {
      int16_t averages[3];
      for (int i = 0; i < 3; i++) {
        averages[i] = (int16_t)(_accelCalibrationData.gyroCurrentSums[i] / NUM_CALIBRATION_SAMPLES_PER_AXIS);
      }

      _persistentKvStore->setIntForKey(PersistentKeysCommon::GYRO_OFFSET_X, averages[0]);
      _persistentKvStore->setIntForKey(PersistentKeysCommon::GYRO_OFFSET_Y, averages[1]);
      _persistentKvStore->setIntForKey(PersistentKeysCommon::GYRO_OFFSET_Z, averages[2]);

      _imu->setGyrXOffset(averages[0]);
      _imu->setGyrYOffset(averages[1]);
      _imu->setGyrZOffset(averages[2]);

      LOG_INFO("Gyro calibration complete: %d, %d, %d", averages[0], averages[1], averages[2]);
    }

    _accelCalibrationData.offsets[_accelCalibrationData.stage] = {
        (int64_t)(_accelCalibrationData.accelCurrentSums[0] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
        (int64_t)(_accelCalibrationData.accelCurrentSums[1] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
        (int64_t)(_accelCalibrationData.accelCurrentSums[2] / NUM_CALIBRATION_SAMPLES_PER_AXIS),
    };

    if (_accelCalibrationData.stage == CalibrationRequest::RollLeft) {
      // Final Stage Completed
      if (_accelCalibrationData.offsets.size() != NUM_ACCELGYRO_CALIBRATION_STAGES) {
        LOG_ERROR("Accelerometer calibration failed: Not all stages completed.");
        _accelCalibrationData.requestHandler(CalibrationRequest::Failed);
        return;
      }

      int64_t offsets[3] = {0, 0, 0};
      for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < NUM_ACCELGYRO_CALIBRATION_STAGES; i++) {
          offsets[axis] += _accelCalibrationData.offsets[(CalibrationRequest)i][axis];
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

      _persistentKvStore->setIntForKey(PersistentKeysCommon::ACCEL_OFFSET_X, offsets[0]);
      _persistentKvStore->setIntForKey(PersistentKeysCommon::ACCEL_OFFSET_Y, offsets[1]);
      _persistentKvStore->setIntForKey(PersistentKeysCommon::ACCEL_OFFSET_Z, offsets[2]);

      _accelCalibrationData.requestHandler(CalibrationRequest::Complete);
    } else {
      // continue to the next stage
      const int nextStage = (int)(_accelCalibrationData.stage) + 1;
      _accelCalibrationData.stage = (CalibrationRequest)nextStage;
      _accelCalibrationData.currentStageSamples = 0;
      _accelCalibrationData.accelCurrentSums = {0, 0, 0};
      // We will wait for the user to send CalibrationResponse::Continue before continuing
      _accelCalibrationData.awaitingResponse = true;
      _accelCalibrationData.requestHandler(_accelCalibrationData.stage);
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
    _accelCalibrationData = {
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
    _accelCalibrationData.awaitingResponse = false;
    _accelerometerCalibrationInProgress = true;
    LOG_INFO("Continuing accelerometer calibration.");
  };

  responseHandlers[CalibrationResponse::Cancel] = [this]() {
    _accelCalibrationData = {};
    _accelerometerCalibrationInProgress = false;
    LOG_INFO("Accelerometer calibration canceled.");
  };

  return responseHandlers;
}
