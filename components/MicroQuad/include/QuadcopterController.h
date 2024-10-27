#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include "PIDController.h"
#include "DebugHelper.h"

#include <stdint.h>
#include <array>
#include <memory>

#define THROTTLE_MIN 1000.0f
#define THROTTLE_MAX 2000.0f
#define MAX_YAW_RATE_DEG_PER_SEC 50.0f
#define MAX_PITCH_ROLL_ANGLE_DEGREES 40.0f

// Defines the range of the controller inputs starting from 0.0f
#define INPUT_MAX_CONTROLLER_INPUT 255.0f
#define NUM_MOTORS 4

typedef double imu_values_t[3];

typedef struct
{
    imu_values_t gyroOutput;
    imu_values_t accelOutput;
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

typedef std::array<double, 4> motor_outputs_t;

typedef struct
{ // yaw, pitch, and roll gains
    gains_t angleGains[3];
    gains_t rateGains[3];
} quadcopter_config_t;

class QuadcopterController
{
public:
    // Call this in arduino setup()
    QuadcopterController(
        quadcopter_config_t config,
        DebugHelper *debugHelper,
        unsigned long timeMicros);

    // Should be called at a relatively constant frequency with
    // new accel/gyro/compass readings.
    motor_outputs_t calculateOutputs(
        imu_output_t imuValues,
        controller_values_t controllerValues,
        unsigned long timeMicros,
        bool recordData);

private:
    std::array<std::unique_ptr<PIDController>, 3> _angleControllers;
    std::array<std::unique_ptr<PIDController>, 3> _rateControllers;
    double _throttle;
    double _previousUpdateMicros;
    quadcopter_config_t _config;
    DebugHelper *_debugHelper;
    unsigned long _startTime;
};

#endif
