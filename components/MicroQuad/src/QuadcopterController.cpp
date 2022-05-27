#include "QuadcopterController.h"

#include <stdlib.h>
#include <stdio.h>
#include <Arduino.h>

#include "PIDController.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// Private functions
static position_values_t _desiredAttitudeValues(controller_values_t controllerValues,
                                                position_values_t scalingValues,
                                                double currentHeading,
                                                double updateIntervalMillis)
{
    // printf("Current heading = %f\n", currentHeading);
    const double timescale = updateIntervalMillis / 1000.0f;

    // this produces a value between -1.0 to 1.0
    const double scaledYawStickOutput = (((double)controllerValues.leftStickInput.x / 255.0f) * 2.0f) - 1.0f;

    // this gives us a new "correct" heading for the PID controller to work with
    const double desiredHeading = currentHeading + ((scaledYawStickOutput * timescale) * scalingValues.yaw);
    // printf("Desired heading = %f\n", desiredHeading);

    const double desiredPitch = (((double)controllerValues.rightStickInput.y / 255.0f) * 2.0f) - 1.0f;
    const double desiredPitchAngle = (desiredPitch * scalingValues.pitch) + 90.0f;
    // printf("Desired pitch = %.4f, right stick input = %.4f\n", desiredPitch, controllerValues.rightStickInput.y);

    const double desiredRoll = (((double)controllerValues.rightStickInput.x / 255.0f) * 2.0f) - 1.0f;
    const double desiredRollAngle = (desiredRoll * scalingValues.roll) + 90.0f;
    // printf("Desired roll = %f\n", desiredRoll);

    return {
        .yaw = desiredHeading,
        .pitch = desiredPitchAngle,
        .roll = desiredRollAngle
    };
}

// Public functions
QuadcopterController::QuadcopterController(quadcopter_config_t config,
                                           unsigned long timeMillis,
                                           double minThrottle,
                                           double maxThrottle)
{
    _config = config;
    _yawController.set_Kpid(config.yawGains.proportionalGain, config.yawGains.integralGain, config.yawGains.derivativeGain);
    _pitchController.set_Kpid(config.pitchGains.proportionalGain, config.pitchGains.integralGain, config.pitchGains.derivativeGain);
    _rollController.set_Kpid(config.rollGains.proportionalGain, config.rollGains.integralGain, config.rollGains.derivativeGain);
    // _yawController.setParameters(timeMillis, {
    //     .gains = config.yawGains,
    //     .outputMax = minThrottle,
    //     .outputMin = minThrottle,
    // });
    // _pitchController.setParameters(timeMillis, {
    //     .gains = config.pitchGains,
    //     .outputMax = minThrottle,
    //     .outputMin = minThrottle,
    // });
    // _rollController.setParameters(timeMillis, {
    //     .gains = config.rollGains,
    //     .outputMax = minThrottle,
    //     .outputMin = minThrottle,
    // });
    _minThrottle = minThrottle;
    _maxThrottle = maxThrottle;
    _previousUpdateMillis = timeMillis;
    _controlScalingValues = {
        .yaw = DEFAULT_YAW_ANGLE_CHANGE_PER_SECOND,
        .pitch = MAX_PITCH_CONTROL_ANGLE,
        .roll = MAX_ROLL_CONTROL_ANGLE
    };
    _throttleScaling = DEFAULT_THROTTLE_SCALING;
}

void QuadcopterController::setControlScalingValues(position_values_t controlScalingValues, float throttleScaling)
{
    _controlScalingValues = controlScalingValues;
    _throttleScaling = throttleScaling;
}

static uint64_t lastUpdateMillis = 0;

motor_outputs_t QuadcopterController::calculateOutputs(position_values_t imuValues,
                                                       controller_values_t controllerValues,
                                                       unsigned long timeMillis)
{
    position_values_t desiredValues = _desiredAttitudeValues(
        controllerValues,
        _controlScalingValues,
        imuValues.yaw,
        timeMillis - _previousUpdateMillis
    );

    const double throttle = (((double)controllerValues.leftStickInput.y / 255.0f) * _minThrottle * _throttleScaling) + _minThrottle;

    const double yawUpdate = _yawController.update_pid_std(desiredValues.yaw, imuValues.yaw, timeMillis);
    const double pitchUpdate = _pitchController.update_pid_std(desiredValues.pitch, imuValues.pitch, timeMillis);
    const double rollUpdate = _rollController.update_pid_std(desiredValues.roll, imuValues.roll, timeMillis);

    if (millis() - lastUpdateMillis > 500) {
        lastUpdateMillis = (uint64_t)millis();
        // printf("Desired pitch: %.2f, current pitch: %.2f\n", desiredValues.pitch, imuValues.pitch);
        // printf("Throttle: %f, throttle stick value: %i\n", throttle, controllerValues.leftStickInput.y);
        // printf("Desired values. Yaw = %f, pitch = %f, roll = %f\n", desiredValues.yaw, desiredValues.pitch, desiredValues.roll);
        // printf("throttle = %f, yaw = %f, pitch = %f, roll = %f\n", throttle, yawUpdate, pitchUpdate, rollUpdate);
    }
    

    _previousUpdateMillis = timeMillis;
    return {
        .values = {
            MIN(MAX(throttle + rollUpdate + yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle - rollUpdate + yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle + pitchUpdate - yawUpdate, _minThrottle), _maxThrottle),
            MIN(MAX(throttle - pitchUpdate - yawUpdate, _minThrottle), _maxThrottle)
        }
    };
}
