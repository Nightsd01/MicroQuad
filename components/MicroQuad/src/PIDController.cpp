#include "PIDController.h"

#include <math.h>
#include <cmath>

#define WINDUP_GAIN 200.0

PIDController::PIDController(DebugHelper *helper, quadcopter_tuning_parameters_t params)
{
  _params = params;
  _helper = helper;
}

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

double PIDController::compute(
  double setPoint,
  double imuValue,
  unsigned long timestamp,
  bool enableIntegral,
  PIDAxis axis
)
{
  float error = setPoint - imuValue;
  float dTime = timestamp - _previousTimestamp;
  _previousTimestamp = timestamp;

  _terms[0] = _params.gains.proportionalGain * error;
  _iState += error * dTime;
  float windupGuard = WINDUP_GAIN / _params.gains.integralGain;
  if (!enableIntegral) {
    _iState = 0.0f;
  } else if (_iState > windupGuard) {
    _iState = windupGuard;
  } else if (_iState < -windupGuard) {
    _iState = -windupGuard;
  }
  _terms[1] = _params.gains.integralGain * _iState;
  _terms[2] = (_params.gains.derivativeGain * _previousError) / (dTime / 1000.0f);
  _previousError = error;
  float vals = _terms[0] + _terms[1] + _terms[2];
  const double output = std::isfinite(vals) ? MAX(MIN(vals, _params.outputMax), _params.outputMin) : 0.0f;
  switch (axis) {
    case PIDAxis::yaw:
      _helper->yawPidValues[0] = _terms[0];
      _helper->yawPidValues[1] = _terms[1];
      _helper->yawPidValues[2] = _terms[2];
      break;
    case PIDAxis::pitch:
      _helper->pitchPidValues[0] = _terms[0];
      _helper->pitchPidValues[1] = _terms[1];
      _helper->pitchPidValues[2] = _terms[2];
      break;
    case PIDAxis::roll:
      _helper->rollPidValues[0] = _terms[0];
      _helper->rollPidValues[1] = _terms[1];
      _helper->rollPidValues[2] = _terms[2];
      break;
  }
  return output;
}

void PIDController::reset(unsigned long timestamp)
{
  _terms[0] = 0.0f;
  _terms[1] = 0.0f;
  _terms[2] = 0.0f;
  _iState = 0.0f;
  _previousError = 0.0f;
  _previousTimestamp = timestamp;
}
