#include "QuadcopterController.h"

#include <stdlib.h>
#include <stdio.h>
#include <Arduino.h>
#include "DebugHelper.h"
#include "PIDController.h"
#include "Logger.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MAX_PID_OUTPUT 500.0f

/*

    3       1

    2       4

*/

// Private functions
static position_values_t _desiredAttitudeValues(controller_values_t controllerValues,
                                                position_values_t scalingValues,
                                                DebugHelper *helper,
                                                double currentHeading,
                                                double updateIntervalMicros)
{
    // printf("Current heading = %f\n", currentHeading);
    const double timescale = updateIntervalMicros / 1000000.0f;

    // this produces a value between -1.0 to 1.0
    const double scaledYawStickOutput = (((double)controllerValues.leftStickInput.x / 255.0f) * 2.0f) - 1.0f;

    // this gives us a new "correct" heading for the PID controller to work with
    const double desiredHeading = currentHeading + ((scaledYawStickOutput * timescale) * scalingValues.yaw);
    // printf("Desired heading = %f\n", desiredHeading);

    const double desiredPitch = (((double)controllerValues.rightStickInput.y / 255.0f) * 2.0f) - 1.0f;
    const double desiredPitchAngle = (desiredPitch * scalingValues.pitch);
    // printf("Desired pitch = %.4f, right stick input = %.4f\n", desiredPitch, controllerValues.rightStickInput.y);
    // printf("Right stick input: %f, desired pitch: %f\n", (double)controllerValues.rightStickInput.y, desiredPitchAngle);
    const double desiredRoll = (((double)controllerValues.rightStickInput.x / 255.0f) * 2.0f) - 1.0f;
    const double desiredRollAngle = (desiredRoll * scalingValues.roll);
    // printf("Desired roll = %f\n", desiredRoll);

    return {
        .yaw = desiredHeading,
        .pitch = desiredPitchAngle,
        .roll = desiredRollAngle
    };
}

// Public functions
QuadcopterController::QuadcopterController(quadcopter_config_t config,
                                           DebugHelper *debugHelper,
                                           unsigned long timeMicros,
                                           double minThrottle,
                                           double maxThrottle)
{
    _debugHelper = debugHelper;
    _config = config;
    _minThrottle = minThrottle;
    _maxThrottle = maxThrottle;
    _previousUpdateMicros = timeMicros;
    _controlScalingValues = {
        .yaw = DEFAULT_YAW_ANGLE_CHANGE_PER_SECOND,
        .pitch = MAX_PITCH_CONTROL_ANGLE,
        .roll = MAX_ROLL_CONTROL_ANGLE
    };
    _throttleScaling = DEFAULT_THROTTLE_SCALING;
    _yawController = new PIDController(
        _debugHelper,
        {
            .gains = _config.yawGains,
            .outputMax = MAX_PID_OUTPUT,
            .outputMin = -MAX_PID_OUTPUT
        }
    );
    _pitchController = new PIDController(
        _debugHelper,
        {
            .gains = _config.pitchGains,
            .outputMax = MAX_PID_OUTPUT,
            .outputMin = -MAX_PID_OUTPUT
        }
    );
    _rollController = new PIDController(
        _debugHelper,
        {
            .gains = _config.rollGains,
            .outputMax = MAX_PID_OUTPUT,
            .outputMin = -MAX_PID_OUTPUT
        }
    );
}

void QuadcopterController::setControlScalingValues(position_values_t controlScalingValues, float throttleScaling)
{
    _controlScalingValues = controlScalingValues;
    _throttleScaling = throttleScaling;
}

void QuadcopterController::startMonitoringPID(void)
{
    _monitoringPid = true;
    _startTime = micros();
}

static unsigned long lastUpdateMillis = 0;

motor_outputs_t QuadcopterController::calculateOutputs(position_values_t imuValues,
                                                       controller_values_t controllerValues,
                                                       unsigned long timeMicros,
                                                       bool recordData)
{
    position_values_t desiredValues = _desiredAttitudeValues(
        controllerValues,
        _controlScalingValues,
        _debugHelper,
        imuValues.yaw,
        timeMicros - _previousUpdateMicros
    );

    const double throttle = (((double)controllerValues.leftStickInput.y / 255.0f) * _minThrottle * _throttleScaling) + _minThrottle;

    double yawUpdate = 0.0f;
    double pitchUpdate = 0.0f;
    double rollUpdate = 0.0f;

    if (_monitoringPid) {
        yawUpdate = _yawController->compute(0.0f, imuValues.yaw, timeMicros, throttle > 1300, PIDAxis::yaw);
        pitchUpdate = _pitchController->compute(desiredValues.pitch, imuValues.pitch, timeMicros, throttle > 1300, PIDAxis::pitch);
        rollUpdate = _rollController->compute(desiredValues.roll, imuValues.roll, timeMicros, throttle > 1300, PIDAxis::roll);
    }

    _previousUpdateMicros = timeMicros;
    motor_outputs_t outputs = {
        .values = {
            MIN(MAX(throttle + pitchUpdate + rollUpdate - yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle - pitchUpdate - rollUpdate - yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle + pitchUpdate - rollUpdate + yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle - pitchUpdate + rollUpdate + yawUpdate, _minThrottle), _maxThrottle)
        }
    };

    if (recordData) {
        _debugHelper->updates[0] = yawUpdate;
        _debugHelper->updates[1] = pitchUpdate;
        _debugHelper->updates[2] = rollUpdate;
        _debugHelper->motorValues[0] = outputs.values[0];
        _debugHelper->motorValues[1] = outputs.values[1];
        _debugHelper->motorValues[2] = outputs.values[2];
        _debugHelper->motorValues[3] = outputs.values[3];
        _debugHelper->ypr[0] = imuValues.yaw;
        _debugHelper->ypr[1] = imuValues.pitch;
        _debugHelper->ypr[2] = imuValues.roll;
        _debugHelper->throttle = throttle;
        _debugHelper->desiredValues[0] = desiredValues.yaw;
        _debugHelper->desiredValues[1] = desiredValues.pitch;
        _debugHelper->desiredValues[2] = desiredValues.roll;
    }

    static unsigned long lastLogTime = 0;
    if (millis() - lastLogTime > 1000) {
        LOG_INFO("Yaw = %f, pitch = %f, roll = %f, throttle = %f, pitch update = %f, roll update = %f, yaw update = %f, desired yaw: %f, desired roll: %f, desired pitch: %f, right controller X: %f, right controller Y: %f", imuValues.yaw, imuValues.pitch, imuValues.roll, throttle, pitchUpdate, rollUpdate, yawUpdate, desiredValues.yaw, desiredValues.pitch, desiredValues.roll, controllerValues.rightStickInput.x, controllerValues.rightStickInput.y);
        lastLogTime = millis();
    }

    return outputs;
}

void QuadcopterController::reset(unsigned long timestamp)
{
    _yawController->reset(timestamp);
    _pitchController->reset(timestamp);
    _rollController->reset(timestamp);
    _monitoringPid = false;
    _startTime = 0;
}
