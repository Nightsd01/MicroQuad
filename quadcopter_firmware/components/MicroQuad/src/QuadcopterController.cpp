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
QuadcopterController::QuadcopterController(
    quadcopter_config_t config, DebugHelper *debugHelper, unsigned long timeMicros)
{
  _debugHelper = debugHelper;
  _config = config;
  for (int i = 0; i < 3; i++) {
    _angleControllers[i] = std::make_unique<PIDController>(config.angleGains[i], debugHelper);
    _rateControllers[i] = std::make_unique<PIDController>(config.rateGains[i], debugHelper);
  }
}

// TODO: {bradhesse} Convert whole codebase purely to use radians and not
// degrees It is really poor practice to use one unit of measurement (degrees)
// in some parts of the codebase and another (radians) in other parts, this
// should be consistent throughout the codebase to avoid unit errors. However
// for debugging I understand degrees more intuitively, so I will leave this for
// now
motor_outputs_t QuadcopterController::calculateOutputs(
    imu_output_t imuValues, controller_values_t controllerValues, unsigned long timeMicros, bool recordData)
{
  const double throttle = (double)controllerValues.leftStickInput.y;  // 0.0 to 255.0

  // Gives us a value between -(INPUT_MAX_CONTROLLER_INPUT/2) and
  // (INPUT_MAX_CONTROLLER_INPUT/2)
  const double desiredYawDegreesDelta = controllerValues.leftStickInput.x - (INPUT_MAX_CONTROLLER_INPUT / 2.0f);
  const double desiredYawDegrees = imuValues.yawPitchRollDegrees[0] + desiredYawDegreesDelta;

  const double desiredPitchDelta =
      (controllerValues.rightStickInput.y - (INPUT_MAX_CONTROLLER_INPUT / 2.0f)) / (INPUT_MAX_CONTROLLER_INPUT / 2.0f);
  const double desiredRollDelta =
      (controllerValues.rightStickInput.x - (INPUT_MAX_CONTROLLER_INPUT / 2.0f)) / (INPUT_MAX_CONTROLLER_INPUT / 2.0f);
  const double desiredPitchAngleDegrees = MAX_PITCH_ROLL_ANGLE_DEGREES * desiredPitchDelta;
  const double desiredRollAngleDegrees = MAX_PITCH_ROLL_ANGLE_DEGREES * desiredRollDelta;

  // Desired yaw/pitch/roll angles in degrees
  const double desiredAnglesDegrees[3] = {desiredYawDegrees, desiredPitchAngleDegrees, desiredRollAngleDegrees};

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
        DEG_TO_RAD(imuValues.yawPitchRollDegrees[i]),
        DEG_TO_RAD(desiredAnglesDegrees[i]),
        timeSeconds);

    // Calculate the rate controller result
    rateControllerOutputs[i] =
        _rateControllers[i]->computeOutput(DEG_TO_RAD(imuValues.gyroOutput[i]), angleControllerOutputs[i], timeSeconds);
  }

  // Axes 1
  motor_outputs_t motors = {
      (float)(throttle - rateControllerOutputs[1] + rateControllerOutputs[2] - rateControllerOutputs[0]),
      (float)(throttle + rateControllerOutputs[1] - rateControllerOutputs[2] - rateControllerOutputs[0]),
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
  }

  return motors;
}
