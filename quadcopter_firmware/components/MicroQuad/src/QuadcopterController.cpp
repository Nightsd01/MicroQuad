#include "QuadcopterController.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "DebugHelper.h"
#include "Logger.h"
#include "PIDController.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define DEG_TO_RAD(x) (x * (M_PI / 180.0))
/*

    2       4

    3       1

*/

// Public functions
QuadcopterController::QuadcopterController(DebugHelper* debugHelper, unsigned long timeMicros)
{
  _debugHelper = debugHelper;
  for (int i = 0; i < 3; i++) {
    _angleControllers[i] = std::make_unique<PIDController>(debugHelper);
    _rateControllers[i] = std::make_unique<PIDController>(debugHelper);
  }
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

  // LOG_INFO_PERIODIC_MILLIS(
  //     100,  // log at most up to every 100 millis
  //     "%8.2f, %8.2f, %8.2f, %8.2f",
  //     (float)throttle,
  //     (float)rateControllerOutputs[1],
  //     (float)rateControllerOutputs[2],
  //     (float)rateControllerOutputs[0]);

  if (recordData) {
    _debugHelper->angleOutputs[0] = angleControllerOutputs[0];
    _debugHelper->angleOutputs[1] = angleControllerOutputs[1];
    _debugHelper->angleOutputs[2] = angleControllerOutputs[2];
    _debugHelper->rateOutputs[0] = rateControllerOutputs[0];
    _debugHelper->rateOutputs[1] = rateControllerOutputs[1];
    _debugHelper->rateOutputs[2] = rateControllerOutputs[2];
    _debugHelper->motorValues[0] = motors[0];
    _debugHelper->motorValues[1] = motors[1];
    _debugHelper->motorValues[2] = motors[2];
    _debugHelper->motorValues[3] = motors[3];
    _debugHelper->throttle = throttle;
    _debugHelper->setPoints[0] = desiredAnglesDegrees[0];
    _debugHelper->setPoints[1] = desiredAnglesDegrees[1];
    _debugHelper->setPoints[2] = desiredAnglesDegrees[2];
  }

  return motors;
}
