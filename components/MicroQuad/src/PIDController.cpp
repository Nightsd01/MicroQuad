#include "PIDController.h"

#include <math.h>
#include <cmath>

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

PIDController::PIDController(
    gains_t gains,
    DebugHelper *helper
)
{
  _gains = gains;
  _helper = helper;
  _firstRun = true;
}

double PIDController::computeOutput(double current, double set, double timeSeconds)
{
  const double deltaTimeSeconds = _firstRun ? 0.00001f : timeSeconds - _previousTimeSeconds;
  _firstRun = false;
  const double error = set - current;

  _integral += error * deltaTimeSeconds;
  _integral = MIN(_integral, INTEGRAL_MAX);
  _integral = MAX(_integral, -INTEGRAL_MAX);

  const double derivative = (error - _previousError) / deltaTimeSeconds;
  _previousError = error;

  return (_gains.kP * error) + (_gains.kI * _integral) + (_gains.kD * derivative);
}