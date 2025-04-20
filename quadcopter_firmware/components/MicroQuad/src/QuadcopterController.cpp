#include "QuadcopterController.h"

#ifndef MATLAB_SIM
#include <Arduino.h>
#endif // MATLAB_SIM

#include <stdio.h>
#include <stdlib.h>

#include "Logger.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define DEG_TO_RAD(x) (float)(x * (M_PI / 180.0))
#define RAD_TO_DEG(x) x * 57.295779513082320876798154814105f
/*

    2       4

    3       1

*/
static double mapf(double x, double in_min, double in_max, double out_min,
                   double out_max) {
  if (x > in_max)
    return out_max;
  if (x < in_min)
    return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float wrapPi(float a) {
  // returns angle in (-π, π]
  if (a > M_PI)
    a -= 2.0f * M_PI;
  if (a <= -M_PI)
    a += 2.0f * M_PI;
  return a;
}

// Public functions
QuadcopterController::QuadcopterController(DebugHelper *debugHelper,
                                           unsigned long timeMicros) {
  _debugHelper = debugHelper;
  for (int i = 0; i < 3; i++) {
    _angleControllers[i] = std::make_unique<PIDController>(debugHelper);
    _rateControllers[i] = std::make_unique<PIDController>(debugHelper);
  }
  _verticalVelocityController = new PIDController(debugHelper);
}

void QuadcopterController::_yawUpdate(unsigned long now,
                                      const controller_values_t &sticks,
                                      const imu_output_t &imu) {
  if (_lastYawUpdateTimeMicros == 0) {
    _lastYawUpdateTimeMicros = now;
    return;
  }

  const double dt = (now - _lastYawUpdateTimeMicros) * 1e-6;
  _lastYawUpdateTimeMicros = now;

  // 1. stick → desired yaw *rate*  (rad/s)
  const double yawStickNorm = (sticks.leftStickInput.x - 127.5) / 127.5;
  const double yawRateRad = yawStickNorm * DEG_TO_RAD(MAX_YAW_RATE_DEG_PER_SEC);

  // 2. integrate → yaw *angle* set‑point, then wrap to ±π
  _yawSetPointRad = wrapPi(_yawSetPointRad + yawRateRad * dt);

  // 3. keep it close to current yaw so the PID takes the short path
  const double yawNowRad = DEG_TO_RAD(imu.yawPitchRollDegrees[0]);
  const double diff = wrapPi(_yawSetPointRad - yawNowRad);
  _yawSetPointRad =
      yawNowRad + diff; // same numeric value but guaranteed shortest arc
}

// TODO: {bradhesse} Convert whole codebase purely to use radians and not
// degrees It is really poor practice to use one unit of measurement (degrees)
// in some parts of the codebase and another (radians) in other parts, this
// should be consistent throughout the codebase to avoid unit errors. However
// for debugging I understand degrees more intuitively, so I will leave this for
// now
motor_outputs_t QuadcopterController::calculateOutputs(
    const quadcopter_config_t &cfg,
    const imu_output_t &imu, // attitude in degrees, gyro in DPS
    const controller_values_t &sticks, unsigned long nowUs, bool recordData) {
  // ------------------------------------------------------------------ 0.  time
  // base
  const double nowSec = nowUs * 1e-6;

  // ------------------------------------------------------------------ 1.
  // altitude loop (unchanged)
  const double throttleIn = (double)sticks.leftStickInput.y; // 0 … 255
  const double velSet_mps =
      (throttleIn - 127.5) * MAX_VERTICAL_VELOCITY_METERS_PER_SECOND;

  const float velPidOut = _verticalVelocityController->computeOutput(
      cfg.verticalVelocityGains, imu.verticalVelocityMetersPerSec, velSet_mps,
      nowSec);

  const float baseThrottle = STEADY_STATE_HOVER_THROTTLE +
                             mapf(velPidOut, -1000.f, 1000.f, -127.5f, 127.5f);

  // ------------------------------------------------------------------ 2.  yaw
  // set‑point initialisation
  if (!_initializedYawSetPointDegrees) {
    _initializedYawSetPointDegrees = true;
    _yawSetPointRad = DEG_TO_RAD(imu.yawPitchRollDegrees[0]);
    LOG_INFO("Init yaw SP %.1f°", imu.yawPitchRollDegrees[0]);
  }

  // updates _yawSetPointRad using sticks and dt, keeps it wrapped
  _yawUpdate(nowUs, sticks, imu);

  // ------------------------------------------------------------------ 3.
  // desired roll / pitch angles
  const double stickNormRoll =
      (sticks.rightStickInput.x - INPUT_MAX_CONTROLLER_INPUT / 2.0) /
      (INPUT_MAX_CONTROLLER_INPUT / 2.0);
  const double stickNormPitch =
      (sticks.rightStickInput.y - INPUT_MAX_CONTROLLER_INPUT / 2.0) /
      (INPUT_MAX_CONTROLLER_INPUT / 2.0);

  const double rollSet_deg = MAX_PITCH_ROLL_ANGLE_DEGREES * stickNormRoll;
  const double pitchSet_deg = MAX_PITCH_ROLL_ANGLE_DEGREES * stickNormPitch;

  // ------------------------------------------------------------------ 4. outer
  // angle PIDs  (rad in / rad out)
  const float angleMeas[3] = {DEG_TO_RAD((float)imu.yawPitchRollDegrees[0]),
                              DEG_TO_RAD((float)imu.yawPitchRollDegrees[1]),
                              DEG_TO_RAD((float)imu.yawPitchRollDegrees[2])};
  float angleSet[3] = {(float)_yawSetPointRad, DEG_TO_RAD((float)pitchSet_deg),
                       DEG_TO_RAD((float)rollSet_deg)};

  // wrap yaw error to shortest path
  angleSet[0] = angleMeas[0] + wrapPi(angleSet[0] - angleMeas[0]);

  float angleOut[3], rateOut[3];
  for (int i = 0; i < 3; ++i) {
    angleOut[i] = _angleControllers[i]->computeOutput(
        cfg.angleGains[i], angleMeas[i], angleSet[i], nowSec);

    rateOut[i] = _rateControllers[i]->computeOutput(
        cfg.rateGains[i],
        DEG_TO_RAD(imu.gyroOutput[i]), // gyro DPS → rad/s
        angleOut[i],                   // desired rad/s
        nowSec);
  }

  // ------------------------------------------------------------------ 5. motor
  // mix
  motor_outputs_t m = {
      (float)(baseThrottle + rateOut[1] - rateOut[2] - rateOut[0]), // M1
      (float)(baseThrottle - rateOut[1] + rateOut[2] - rateOut[0]), // M2
      (float)(baseThrottle - rateOut[1] - rateOut[2] + rateOut[0]), // M3
      (float)(baseThrottle + rateOut[1] + rateOut[2] + rateOut[0])  // M4
  };

  // ------------------------------------------------------------------ 6. scale
  // & clamp to ESC range
  for (int i = 0; i < NUM_MOTORS; ++i) {
    m[i] = MIN(MAX(m[i], 0), 255); // stick byte range
    m[i] = (m[i] / 255.0f) * (THROTTLE_MAX - THROTTLE_MIN) + THROTTLE_MIN;
    m[i] = MIN(MAX(m[i], THROTTLE_MIN), THROTTLE_MAX);
  }

#ifndef MATLAB_SIM
  if (recordData) {
    _debugHelper->angleOutputs[0] = angleOut[0];
    _debugHelper->angleOutputs[1] = angleOut[1];
    _debugHelper->angleOutputs[2] = angleOut[2];
    _debugHelper->rateOutputs[0] = rateOut[0];
    _debugHelper->rateOutputs[1] = rateOut[1];
    _debugHelper->rateOutputs[2] = rateOut[2];
    _debugHelper->motorValues[0] = m[0];
    _debugHelper->motorValues[1] = m[1];
    _debugHelper->motorValues[2] = m[2];
    _debugHelper->motorValues[3] = m[3];
    _debugHelper->throttle = baseThrottle;
    _debugHelper->setPoints[0] = RAD_TO_DEG(angleSet[0]);
    _debugHelper->setPoints[1] = RAD_TO_DEG(angleSet[1]);
    _debugHelper->setPoints[2] = RAD_TO_DEG(angleSet[2]);
  }
#endif // MATLAB_SIM

  _previousUpdateMicros = nowUs;
  return m;
}
