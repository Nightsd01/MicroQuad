#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include "PIDController.h"
#include "DebugHelper.h"

#include <stdint.h>

#define THROTTLE_MIN 1000.0f
#define THROTTLE_MAX 2000.0f

typedef double imu_values_t[3];

typedef struct {
    imu_values_t gyroOutput;
    imu_values_t accelOutput;
} imu_output_t;

typedef struct {
    double x;
    double y;
} stick_input_values_t;

typedef struct {
    stick_input_values_t leftStickInput;
    stick_input_values_t rightStickInput;
} controller_values_t;

typedef double motor_outputs_t[4];

typedef struct { // yaw, pitch, and roll gains
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
            unsigned long timeMicros
        );

        // Should be called at a relatively constant frequency with
        // new accel/gyro/compass readings.
        void calculateOutputs(
            imu_output_t imuValues,
            controller_values_t controllerValues,
            unsigned long timeMicros,
            bool recordData,
            motor_outputs_t *result
        );

    private:
        PIDController _angleControllers[3];
        PIDController _rateControllers[3];
        double _throttle;
        double _previousUpdateMicros;
        quadcopter_config_t _config;
        DebugHelper *_debugHelper;
        unsigned long _startTime;
};

#endif
