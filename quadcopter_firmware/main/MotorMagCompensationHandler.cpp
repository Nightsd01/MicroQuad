#include "MotorMagCompensationHandler.h"

#ifndef MATLAB_SIM

#include <Logger.h>

#include "Constants.h"
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

static mag_update_t lerpMag(const mag_update_t &a, const mag_update_t &b,
                            float t)  // 0 … 1
{
  mag_update_t m;
  m.x = a.x + t * (b.x - a.x);
  m.y = a.y + t * (b.y - a.y);
  m.z = a.z + t * (b.z - a.z);
  m.heading = 0;  // placeholder
  return m;
}

static float _headingFromXY(float x, float y)
{
  // declination for Foster City, CA: 12° 55′ E  (2025)
  constexpr float declinationRad = DEG_TO_RADS(DECLINATION_ANGLE_DEG);

  float hdg = atan2(-y, x);  // NED frame  (‑Y because of right‑hand rule)
  hdg += declinationRad;

  if (hdg < 0)
    hdg += 2 * PI;
  else if (hdg > 2 * PI)
    hdg -= 2 * PI;

  return hdg * 180.0f / PI;  // degrees 0‑360
}

MotorMagCompensationHandler::MotorMagCompensationHandler(PersistentKeyValueStore *kvStore)
{
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
      std::vector<float> xyzOffsets = _kvStore->getVectorForKey<float>(key, 4 /* length */);
      if (xyzOffsets.size() != 4) {
        LOG_WARN(
            "Invalid calibration data stored for motor %d, step %d, expected 4 XYZ + voltage elements but found %i",
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
      _perMotorCalibrationVoltage[motor][step] = xyzOffsets[3];  // voltage
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

void MotorMagCompensationHandler::updateMagValue(const mag_update_t &mag,
                                                 float voltage)  // ★
{
  if (!isCalibrating) return;

  const uint32_t now = millis();

  //------------------------------------
  // 0.  spin‑up period – ignore samples
  //------------------------------------
  if (_inSpinUp) {                                 // ★
    if (now - _spinUpStartTs < SPINUP_MS) return;  // ★
    _inSpinUp = false;                             // ★
    _lastStepTs = now;                             // start data window AFTER spin‑up  // ★
    _stepAcc.reset();                              // fresh accumulation               // ★
  }

  //------------------------------------
  // 1.  accumulate this sample
  //------------------------------------
  _stepAcc.accumulate(mag, voltage);  // ★

  //------------------------------------
  // 2.  wait until 1‑s window completes
  //------------------------------------
  if (now - _lastStepTs < CALIBRATION_STEP_TIME_MS) return;

  //------------------------------------
  // 3.  store mean mag + mean voltage
  //------------------------------------
  _perMotorCalibrationData[_curMotor][_curStep] = _stepAcc.meanMag();
  _perMotorCalibrationVoltage[_curMotor][_curStep] = _stepAcc.meanVolt();  // ★

  //------------------------------------
  // 4.  reset accumulators & advance state machine
  //------------------------------------
  _stepAcc.reset();
  _lastStepTs = now;

  ++_curStep;
  if (_curStep == CALIBRATION_MOTOR_STEPS) {
    _curStep = 0;
    ++_curMotor;
    if (_curMotor == NUM_MOTORS) {
      _completeCalibration();
      return;
    }
  }

  //------------------------------------
  // 5.  set next throttle & start spin‑up
  //------------------------------------
  motor_outputs_t throttles{};
  std::fill(std::begin(throttles), std::end(throttles), THROTTLE_MIN);

  const float duty = mapRange(
      static_cast<float>(_curStep),
      0.f,
      static_cast<float>(CALIBRATION_MOTOR_STEPS - 1),
      THROTTLE_MIN,
      THROTTLE_MAX);

  throttles[_curMotor] = duty;
  _motorOutputCallback(throttles);

  _inSpinUp = true;      // ★
  _spinUpStartTs = now;  // ★
}

mag_update_t MotorMagCompensationHandler::applyMagneticMotorCompensation(
    const mag_update_t &rawMag,
    const motor_outputs_t &motorThrottles,
    float liveVoltage)  // ★ pass in latest pack V
{
  mag_update_t totalBias{0, 0, 0, 0};

  for (int motor = 0; motor < NUM_MOTORS; ++motor) {
    // --- 1. normalise throttle to 0…1 ---------------------------------
    const float tNorm = map(motorThrottles[motor], THROTTLE_MIN, THROTTLE_MAX, 0.0f, 1.0f);

    // --- 2. find surrounding calibration steps ------------------------
    const float idx = tNorm * (CALIBRATION_MOTOR_STEPS - 1);
    const int k0 = static_cast<int>(std::floor(idx));
    const int k1 = std::min(k0 + 1, CALIBRATION_MOTOR_STEPS - 1);
    const float alpha = idx - k0;

    // --- 3. interpolate bias & voltage --------------------------------
    const auto &m0 = _perMotorCalibrationData[motor][k0];
    const auto &m1 = _perMotorCalibrationData[motor][k1];
    const auto bias = lerpMag(m0, m1, alpha);

    const float v0 = _perMotorCalibrationVoltage[motor][k0];
    const float v1 = _perMotorCalibrationVoltage[motor][k1];
    const float vCal = v0 + alpha * (v1 - v0);  // volts at calib

    // --- 4. rescale for battery sag -----------------------------------
    const float scale = vCal / liveVoltage;  // e.g. 3.8 / 3.5
    totalBias.x += bias.x * scale;
    totalBias.y += bias.y * scale;
    totalBias.z += bias.z * scale;
  }

  // --- 5. subtract bias from raw reading --------------------------------
  return {
      .x = rawMag.x - totalBias.x,
      .y = rawMag.y - totalBias.y,
      .z = rawMag.z - totalBias.z,
      .heading = _headingFromXY(rawMag.x - totalBias.x, rawMag.y - totalBias.y),
  };
}

void MotorMagCompensationHandler::_completeCalibration()
{
  LOG_INFO(
      "Successfully completed motor magnetometer‑compensation calibration! "
      "Persisting calibration values + voltages to KV store");

  isCalibrating = false;
  isCalibrated = true;

  for (int motor = 0; motor < NUM_MOTORS; ++motor) {
    for (int step = 0; step < CALIBRATION_MOTOR_STEPS; ++step) {
      const auto &mag = _perMotorCalibrationData[motor][step];
      const float v = _perMotorCalibrationVoltage[motor][step];

      // [ x, y, z, heading, voltage ]
      std::vector<float> record = {mag.x, mag.y, mag.z, v};

      std::string key = _getKey(motor, step);
      _kvStore->setVectorForKey(key, record);
    }
  }

  // Allow caller to know we finished successfully
  _calibrationCompleteCallback(true);

  // --- optional: reset state so a new calibration can start cleanly ---
  _curMotor = 0;
  _curStep = 0;
  _inSpinUp = false;
  _stepAcc.reset();
}
#endif  // MATLAB_SIM