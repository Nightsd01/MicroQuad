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
}

float PIDController::computeOutput(float current, float set)
{
  _error = current - set;

  _integral += _error;
  _integral = MIN(_integral, INTEGRAL_MAX);
  _integral = MAX(_integral, -INTEGRAL_MAX);

  _derivative = _error - _previousError;
  _previousError = _error;

  return (_gains.kP * _error) + (_gains.kI * _integral) + (_gains.kD * _derivative);
}