#include "QuadcopterController.h"

#ifndef MATLAB_SIM
#include <Arduino.h>
#endif // MATLAB_SIM

#include <stdio.h>
#include <stdlib.h>

#include "Logger.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define DEG_TO_RAD(x) (x * (M_PI / 180.0))
/*

    2       4

    3       1

*/
static double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  if (x > in_max) return out_max;
  if (x < in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Public functions
QuadcopterController::QuadcopterController(DebugHelper* debugHelper, unsigned long timeMicros)
{
  _debugHelper = debugHelper;
  for (int i = 0; i < 3; i++) {
    _angleControllers[i] = std::make_unique<PIDController>(debugHelper);
    _rateControllers[i] = std::make_unique<PIDController>(debugHelper);
  }
  _verticalVelocityController = new PIDController(debugHelper);
}

void QuadcopterController::_yawUpdate(
    unsigned long currentTimeMicros, const controller_values_t& controllerValues, const imu_output_t& imuValues)
{
  // On the first call, just initialize the timestamp and return.
  if (_lastYawUpdateTimeMicros == 0) {
    _lastYawUpdateTimeMicros = currentTimeMicros;
    return;
  }

  // Calculate elapsed time in seconds.
  double dt = static_cast<double>(currentTimeMicros - _lastYawUpdateTimeMicros) / 1e6;
  _lastYawUpdateTimeMicros = currentTimeMicros;

  // Convert the left stick X range (0–255) to -1.0 to +1.0 (center around 127.5).
  // leftStickInput.x == 127.5 -> 0.0 yaw input
  // leftStickInput.x == 0     -> approx -1.0 yaw input
  // leftStickInput.x == 255   -> approx +1.0 yaw input
  double yawStickNormalized = (controllerValues.leftStickInput.x - 127.5) / 127.5;

  // Determine the desired yaw rate (degrees/second).
  double yawRateDegPerSec = yawStickNormalized * MAX_YAW_RATE_DEG_PER_SEC;

  // Update the yaw setpoint based on the rate and elapsed time.
  _yawSetPointDegrees += yawRateDegPerSec * dt;

  // update yaw setpoint to the shortest path from current yaw
  if (fabsf(_yawSetPointDegrees - imuValues.yawPitchRollDegrees[0]) > 180.0) {
    if (_yawSetPointDegrees > imuValues.yawPitchRollDegrees[0]) {
      _yawSetPointDegrees -= 360.0;
    } else {
      _yawSetPointDegrees += 360.0;
    }
  }
}

// TODO: {bradhesse} Convert whole codebase purely to use radians and not
// degrees It is really poor practice to use one unit of measurement (degrees)
// in some parts of the codebase and another (radians) in other parts, this
// should be consistent throughout the codebase to avoid unit errors. However
// for debugging I understand degrees more intuitively, so I will leave this for
// now
motor_outputs_t QuadcopterController::calculateOutputs(
    const quadcopter_config_t& config,
    const imu_output_t& imuValues,
    const controller_values_t& controllerValues,
    unsigned long timeMicros,
    bool recordData)
{
  const double throttle = (double)controllerValues.leftStickInput.y;  // 0.0 to 255.0

  const double desiredMetersPerSec = (throttle - (255.0f / 2.0f)) * MAX_VERTICAL_VELOCITY_METERS_PER_SECOND;

  if (!_initializedYawSetPointDegrees) {
    _initializedYawSetPointDegrees = true;
    _yawSetPointDegrees = imuValues.yawPitchRollDegrees[0];
    LOG_INFO("Initialized yaw setpoint to %f", _yawSetPointDegrees);
  }

  _yawUpdate(timeMicros, controllerValues, imuValues);

  const double desiredPitchDelta =
      (controllerValues.rightStickInput.y - (INPUT_MAX_CONTROLLER_INPUT / 2.0f)) / (INPUT_MAX_CONTROLLER_INPUT / 2.0f);
  const double desiredRollDelta =
      (controllerValues.rightStickInput.x - (INPUT_MAX_CONTROLLER_INPUT / 2.0f)) / (INPUT_MAX_CONTROLLER_INPUT / 2.0f);
  const double desiredPitchAngleDegrees = MAX_PITCH_ROLL_ANGLE_DEGREES * desiredPitchDelta;
  const double desiredRollAngleDegrees = MAX_PITCH_ROLL_ANGLE_DEGREES * desiredRollDelta;

  // Desired yaw/pitch/roll angles in degrees
  const double desiredAnglesDegrees[3] = {_yawSetPointDegrees, desiredPitchAngleDegrees, desiredRollAngleDegrees};

  // x, y, and z
  // for the accelerometer, at rest, Z = 1 because of gravity vector
  float angleControllerOutputs[3];
  float rateControllerOutputs[3];
  const double timeSeconds = (double)timeMicros / 1000000.0f;
  for (int i = 0; i < 3; i++) {
    // Calculate the angle controller result, which is the desired
    // yaw/pitch/roll rate We then feed this into the rate controller to get the
    // desired motor output
    angleControllerOutputs[i] = _angleControllers[i]->computeOutput(
        config.angleGains[i],
        DEG_TO_RAD(imuValues.yawPitchRollDegrees[i]),
        DEG_TO_RAD(desiredAnglesDegrees[i]),
        timeSeconds);

    // Calculate the rate controller result
    rateControllerOutputs[i] = _rateControllers[i]->computeOutput(
        config.rateGains[i],
        DEG_TO_RAD(imuValues.gyroOutput[i]),
        angleControllerOutputs[i],
        timeSeconds);
  }

  // ranges from -1000 to 1000
  const float verticalVelocityOutput = _verticalVelocityController->computeOutput(
      config.verticalVelocityGains,
      imuValues.verticalVelocityMetersPerSec,
      desiredMetersPerSec,
      timeSeconds);

  const float adjustedThrottle =
      STEADY_STATE_HOVER_THROTTLE + mapf(verticalVelocityOutput, -1000.0f, 1000.0f, -127.5f, 127.5f);
  LOG_INFO_PERIODIC_MILLIS(
      100,
      "Vertical velocity = %.2f, adjusted throttle: %.2f, vertical velocity PID output: %.2f",
      imuValues.verticalVelocityMetersPerSec,
      adjustedThrottle,
      verticalVelocityOutput);

  // Axes 1
  motor_outputs_t motors = {
      (float)(throttle + rateControllerOutputs[1] - rateControllerOutputs[2] - rateControllerOutputs[0]),
      (float)(throttle - rateControllerOutputs[1] + rateControllerOutputs[2] - rateControllerOutputs[0]),
      (float)(throttle - rateControllerOutputs[1] - rateControllerOutputs[2] + rateControllerOutputs[0]),
      (float)(throttle + rateControllerOutputs[1] + rateControllerOutputs[2] + rateControllerOutputs[0])};

  for (int i = 0; i < NUM_MOTORS; i++) {
    // Clamp to 0 to 255
    motors[i] = MIN(MAX(motors[i], 0), 255);

    // Scale to between THROTTLE_MAX and THROTTLE_MIN
    motors[i] = (motors[i] / 255.0) * (THROTTLE_MAX - THROTTLE_MIN) + THROTTLE_MIN;
  }

  _previousUpdateMicros = timeMicros;

  for (int i = 0; i < 4; i++) {
    motors[i] = MIN(MAX(motors[i], THROTTLE_MIN), THROTTLE_MAX);
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

  return motors;
}
