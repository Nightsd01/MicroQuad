#include "QuadcopterController.h"

#include <stdlib.h>
#include <stdio.h>
#include <Arduino.h>
#include "DebugHelper.h"
#include "PIDController.h"
#include "Logger.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/*

    3       1

    2       4

*/


// Public functions
QuadcopterController::QuadcopterController(
    quadcopter_config_t config,
    DebugHelper *debugHelper,
    unsigned long timeMicros
)
{
    _debugHelper = debugHelper;
    _config = config;
    for (int i = 0; i < 3; i++) {
        _angleControllers[i] = std::make_unique<PIDController>(config.angleGains[i], debugHelper);
        _rateControllers[i] = std::make_unique<PIDController>(config.rateGains[i], debugHelper);
    }
}

static unsigned long lastUpdateMillis = 0;

static float _initialYaw = 0.0f;

motor_outputs_t QuadcopterController::calculateOutputs(
    imu_output_t imuValues,
    controller_values_t controllerValues,
    unsigned long timeMicros,
    bool recordData
)
{
    const double throttle = (double)controllerValues.leftStickInput.y; // 0.0 to 255.0

    if (_initialYaw == 0.0f) {
        _initialYaw = imuValues.accelOutput[0];
    }

    // x, y, and z
    // for the accelerometer, at rest, Z = 1 because of gravity vector
    float angleControllerOutputs[3];
    float rateControllerOutputs[3];
    for (int i = 0; i < 3; i++) {
        // Calculate the angle controller result, which is the desired yaw/pitch/roll rate
        // We then feed this into the rate controller to get the desired motor output
        // TODO: For now, use initialYaw for desired yaw and 0.0f for pitch and roll
        angleControllerOutputs[i] = _angleControllers[i]->computeOutput(imuValues.accelOutput[i], i == 0 ? _initialYaw : 0.0f);

        // Calculate the rate controller result, which is the desired motor output
        rateControllerOutputs[i] = _rateControllers[i]->computeOutput(imuValues.gyroOutput[i], angleControllerOutputs[i]);
    }
    
    // Axes 1
    motor_outputs_t motors = {
        throttle + rateControllerOutputs[1] + rateControllerOutputs[2] + angleControllerOutputs[0],
        throttle + rateControllerOutputs[1] - rateControllerOutputs[2] - angleControllerOutputs[0],
        throttle - rateControllerOutputs[1] - rateControllerOutputs[2] + angleControllerOutputs[0],
        throttle - rateControllerOutputs[1] + rateControllerOutputs[2] - angleControllerOutputs[0]
    };

    _previousUpdateMicros = timeMicros;

    for (int i = 0; i < 4; i++) {
        motors[i] = MIN(MAX(motors[i], THROTTLE_MIN), THROTTLE_MAX);
    }

    if (recordData) {
        _debugHelper->updates[0] = 0.0f; // TODO
        _debugHelper->updates[1] = 0.0f; // TODO
        _debugHelper->updates[2] = 0.0f; // TODO
        _debugHelper->motorValues[0] = motors[0];
        _debugHelper->motorValues[1] = motors[1];
        _debugHelper->motorValues[2] = motors[2];
        _debugHelper->motorValues[3] = motors[3];
        _debugHelper->ypr[0] = 0.0f; // TODO
        _debugHelper->ypr[1] = 0.0f; // TODO
        _debugHelper->ypr[2] = 0.0f; // TODO
        _debugHelper->throttle = throttle;
        _debugHelper->desiredValues[0] = 0.0f; // TODO
        _debugHelper->desiredValues[1] = 0.0f; // TODO
        _debugHelper->desiredValues[2] = 0.0f; // TODO
    }

    return motors;
}
