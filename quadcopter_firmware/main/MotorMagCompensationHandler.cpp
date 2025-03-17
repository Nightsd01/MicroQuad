#include "MotorMagCompensationHandler.h"

#include <Logger.h>

#include "MotorController.h"
#include "PersistentKeyValueStore.h"
#include "PersistentKeysCommon.h"

std::string _getKey(int motor, int step)
{
  return PersistentKeysCommon::MAG_COMPENSATION_OFFSET_PREFIX + "m" + std::to_string(motor) + "s" +
         std::to_string(step);
}

static float mapRange(float value, float inMin, float inMax, float outMin, float outMax)
{
  // Optionally clamp 'value' to [inMin, inMax] if desired:
  if (value < inMin) value = inMin;
  if (value > inMax) value = inMax;

  return outMin + (outMax - outMin) * ((value - inMin) / (inMax - inMin));
}

/**
 * @brief Interpolate linearly between two mag_update_t data points.
 * @param low   The data at step indexLow
 * @param high  The data at step indexHigh
 * @param ratio The fraction in [0..1] between them
 */
static mag_update_t interpolateMag(const mag_update_t &low, const mag_update_t &high, float ratio)
{
  return {
      .x = low.x + (high.x - low.x) * ratio,
      .y = low.y + (high.y - low.y) * ratio,
      .z = low.z + (high.z - low.z) * ratio,
  };
}

MotorMagCompensationHandler::MotorMagCompensationHandler(QMC5883L *magSensor, PersistentKeyValueStore *kvStore)
{
  _magSensor = magSensor;
  _kvStore = kvStore;
  isCalibrating = false;
  isCalibrated = false;
  _currentlyCalibratingMotor = 0;
  _currentMotorCalibrationStep = 0;
  _lastCalibrationStepTimeMillis = 0;

  for (int motor = 0; motor < NUM_MOTORS; motor++) {
    for (int step = 0; step < CALIBRATION_MOTOR_STEPS; step++) {
      std::string key = _getKey(motor, step);
      if (!_kvStore->hasValueForKey(key)) {
        LOG_WARN(
            "Magnetometer motor compensation calibration data not found in KV store - please run this calibration");
        return;
      }
      std::vector<float> xyzOffsets = _kvStore->getVectorForKey<float>(key, 3 /* length */);
      if (xyzOffsets.size() != 3) {
        LOG_WARN(
            "Invalid calibration data stored for motor %d, step %d, expected 3 XYZ elements but found %i",
            motor,
            step,
            xyzOffsets.size());
        return;
      }
      mag_update_t offsets = {
          .x = xyzOffsets[0],
          .y = xyzOffsets[1],
          .z = xyzOffsets[2],
          .heading = 0.0f  // we can ignore heading here as we don't use it
      };
      _perMotorCalibrationData[motor][step] = offsets;
    }
  }

  LOG_INFO("Completed retrieving mag/motor compensation offsets from persistent KV store");
  isCalibrated = true;
}

void MotorMagCompensationHandler::beginCalibration(
    std::function<void(motor_outputs_t)> motorOutputCallback, std::function<void(bool)> calibrationCompleteCallback)
{
  isCalibrating = true;
  _motorOutputCallback = motorOutputCallback;
  _calibrationCompleteCallback = calibrationCompleteCallback;
  _currentlyCalibratingMotor = 0;
  _currentMotorCalibrationStep = 0;
  _lastCalibrationStepTimeMillis = millis();

  motor_outputs_t motorValues;
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorValues[i] = THROTTLE_MIN;
  }
  _motorOutputCallback(motorValues);
}

void MotorMagCompensationHandler::updateMagValue(mag_update_t magValues)
{
  if (!isCalibrating) {
    return;
  }

  if (millis() - _lastCalibrationStepTimeMillis < CALIBRATION_STEP_TIME_MS) {
    _currentStepMagValues.push_back(magValues);
    return;
  }

  // Compute the average x/y/z values and store
  mag_update_t total = magValues;
  for (auto &value : _currentStepMagValues) {
    total.x += value.x;
    total.y += value.y;
    total.z += value.z;
  }
  total.x /= _currentStepMagValues.size() + 1;
  total.y /= _currentStepMagValues.size() + 1;
  total.z /= _currentStepMagValues.size() + 1;
  _perMotorCalibrationData[_currentlyCalibratingMotor][_currentMotorCalibrationStep] = total;

  if (_currentMotorCalibrationStep >= CALIBRATION_MOTOR_STEPS) {
    _currentlyCalibratingMotor++;
    _currentMotorCalibrationStep = 0;
    if (_currentlyCalibratingMotor >= NUM_MOTORS) {
      _completeCalibration();
      return;
    }
  }

  _lastCalibrationStepTimeMillis = millis();

  motor_outputs_t motorValues;
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorValues[i] = THROTTLE_MIN;
  }
  motorValues[_currentlyCalibratingMotor] = mapRange(
      (float)(_currentMotorCalibrationStep),
      0.0f,
      (float)(CALIBRATION_MOTOR_STEPS),
      THROTTLE_MIN,
      THROTTLE_MAX);

  _motorOutputCallback(motorValues);
  _currentMotorCalibrationStep++;
}

mag_update_t MotorMagCompensationHandler::applyMagneticMotorCompensation(
    const mag_update_t &rawMagData, const motor_outputs_t &motorThrottles)
{
  // This will hold the sum of offsets across all motors
  mag_update_t totalOffset = {0, 0, 0, 0};

  // For each motor, figure out the throttle-based offset
  for (int motorIndex = 0; motorIndex < NUM_MOTORS; motorIndex++) {
    const float throttle = map(motorThrottles[motorIndex], THROTTLE_MIN, THROTTLE_MAX, 0.0f, 1.0f);

    // We have 5 discrete steps: 0..4.
    // Convert throttle [0..1] to a float index in [0..4].
    const float scaledStep = throttle * (CALIBRATION_MOTOR_STEPS - 1);
    int stepLow = static_cast<int>(std::floor(scaledStep));
    int stepHigh = static_cast<int>(std::ceil(scaledStep));

    // Clamp to valid range just in case
    if (stepLow < 0) stepLow = 0;
    if (stepHigh >= CALIBRATION_MOTOR_STEPS) stepHigh = CALIBRATION_MOTOR_STEPS - 1;

    // If stepLow == stepHigh, no interpolation needed
    float ratio = 0.0f;
    if (stepHigh != stepLow) {
      ratio = (scaledStep - stepLow) / (stepHigh - stepLow);
    }

    // Interpolate the offset from _perMotorCalibrationData
    mag_update_t offsetLow = _perMotorCalibrationData[motorIndex][stepLow];
    mag_update_t offsetHigh = _perMotorCalibrationData[motorIndex][stepHigh];
    mag_update_t motorOffset = interpolateMag(offsetLow, offsetHigh, ratio);

    // Accumulate into total offset
    totalOffset.x += motorOffset.x;
    totalOffset.y += motorOffset.y;
    totalOffset.z += motorOffset.z;
    totalOffset.heading += motorOffset.heading;
  }

  // Now subtract totalOffset from the raw reading
  mag_update_t corrected;
  corrected.x = rawMagData.x - totalOffset.x;
  corrected.y = rawMagData.y - totalOffset.y;
  corrected.z = rawMagData.z - totalOffset.z;

  // If you also want to adjust heading, you'd do something like:
  corrected.heading = rawMagData.heading - totalOffset.heading;

  return corrected;
}

void MotorMagCompensationHandler::_completeCalibration(void)
{
  LOG_INFO(
      "Successfully completed motor magnetometer compensation calibration! Persisting calibration values to KV store");
  isCalibrating = false;
  isCalibrated = true;
  for (int motor = 0; motor < NUM_MOTORS; motor++) {
    for (int step = 0; step < CALIBRATION_MOTOR_STEPS; step++) {
      std::string key = _getKey(motor, step);
      std::vector<float> xyzOffsets = {
          _perMotorCalibrationData[motor][step].x,
          _perMotorCalibrationData[motor][step].y,
          _perMotorCalibrationData[motor][step].z,
      };
      _kvStore->setVectorForKey(key, xyzOffsets);
    }
  }
  _calibrationCompleteCallback(true);
}