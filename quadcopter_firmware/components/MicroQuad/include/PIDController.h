#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "DebugHelper.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define ERROR_LEN 11
#define INTEGRAL_MAX 10.0f

enum PIDAxis
{
  yaw,
  pitch,
  roll
};

typedef struct gains_t
{
  double kP;  // Kp
  double kI;  // Ki
  double kD;  // Kd
} gains_t;

class PIDController
{
 public:
  PIDController(DebugHelper *helper);

  double computeOutput(const gains_t &gains, double current, double set, double timeSeconds);

 private:
  DebugHelper *_helper;
  double _previousError;
  double _integral;
  double _previousTimeSeconds;
  bool _firstRun;
};

#endif
