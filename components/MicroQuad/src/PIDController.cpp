#include "PIDController.h"

#include <math.h>
#include <cmath>

#define WINDUP_GAIN 500.0

PIDController::PIDController(quadcopter_tuning_parameters_t params)
{
  _params = params;
}

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

double PIDController::compute(
  double setPoint,
  double imuValue,
  unsigned long timestamp
)
{
  float error = setPoint - imuValue;
  float dTime = timestamp - _previousTimestamp;
  _previousTimestamp = timestamp;

  _terms[0] = _params.gains.proportionalGain * error;
  _iState += error * dTime;
  float windupGuard = WINDUP_GAIN / _params.gains.integralGain;
  if (_iState > windupGuard) {
    _iState = windupGuard;
  } else if (_iState < -windupGuard) {
    _iState = -windupGuard;
  }
  _terms[1] = _params.gains.integralGain * _iState;
  _terms[2] = (_params.gains.derivativeGain * (imuValue - _previousImuValue)) / dTime;
  _previousImuValue = imuValue;
  float vals = _terms[0] + _terms[1] - _terms[2];
  if (std::isfinite(vals)) {
    return MAX(MIN(vals, _params.outputMax), _params.outputMin);
  } else {
    return 0.0f;
  }
}

void PIDController::reset(unsigned long timestamp)
{
  _terms[0] = 0.0f;
  _terms[1] = 0.0f;
  _terms[2] = 0.0f;
  _iState = 0.0f;
  _previousImuValue = 0.0f;
  _previousTimestamp = timestamp;
}
