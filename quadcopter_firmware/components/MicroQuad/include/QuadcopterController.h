#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include <stdint.h>

#include <array>
#include <memory>

#include "DebugHelper.h"
#include "PIDController.h"

#define THROTTLE_MIN 1000.0f
#define THROTTLE_MAX 2000.0f
#define MAX_YAW_RATE_DEG_PER_SEC 75.0f
#define MAX_PITCH_ROLL_ANGLE_DEGREES 60.0f
#define MAX_VERTICAL_VELOCITY_METERS_PER_SECOND 1.0f
#define STEADY_STATE_HOVER_THROTTLE 93.0f  // determined via testing on a 0-255 range

// Defines the range of the controller inputs starting from 0.0f
#define INPUT_MAX_CONTROLLER_INPUT 255.0f
#define NUM_MOTORS 4

typedef double imu_values_t[3];

typedef struct
{
  imu_values_t gyroOutput;
  imu_values_t yawPitchRollDegrees;
  double altitudeMeters;
  double verticalVelocityMetersPerSec;
} imu_output_t;

typedef struct
{
  double x;
  double y;
} stick_input_values_t;

typedef struct
{
  stick_input_values_t leftStickInput;
  stick_input_values_t rightStickInput;
} controller_values_t;

typedef std::array<float, NUM_MOTORS> motor_outputs_t;

typedef struct
{  // yaw, pitch, and roll gains
  gains_t angleGains[3];
  gains_t rateGains[3];
  gains_t verticalVelocityGains;
} quadcopter_config_t;

class QuadcopterController
{
 public:
  // Call this in arduino setup()
  QuadcopterController(DebugHelper *debugHelper, unsigned long timeMicros);

  // Should be called at a relatively constant frequency with
  // new accel/gyro/compass readings.
  motor_outputs_t calculateOutputs(
      const quadcopter_config_t &config,
      const imu_output_t &imuValues,
      const controller_values_t &controllerValues,
      unsigned long timeMicros,
      bool recordData);

 private:
  std::array<std::unique_ptr<PIDController>, 3> _angleControllers;
  std::array<std::unique_ptr<PIDController>, 3> _rateControllers;
  PIDController *_verticalVelocityController;
  double _throttle;
  double _previousUpdateMicros;
  DebugHelper *_debugHelper;
  unsigned long _startTime;
  double _yawSetPointDegrees;
  bool _initializedYawSetPointDegrees = false;
  unsigned long _lastYawUpdateTimeMicros = 0;
  void _yawUpdate(
      unsigned long currentTimeMicros, const controller_values_t &controllerValues, const imu_output_t &imuValues);
};

#endif
