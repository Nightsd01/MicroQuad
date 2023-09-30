#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include "PIDController.h"
#include "DebugHelper.h"

#include <stdint.h>

// Currently assumes a quadcopter with an X configuration

typedef struct position_values_t {
    double yaw;
    double pitch;
    double roll;
} position_values_t;

struct motor_outputs_t {
    double values[4];
};

struct stick_input_values_t {
    double x;
    double y;
};

struct controller_values_t {
    stick_input_values_t leftStickInput;
    stick_input_values_t rightStickInput;
};

struct quadcopter_config_t {
    gains_t yawGains;
    gains_t pitchGains;
    gains_t rollGains;
};

// Definitions

// allows the vehicle to rotate up to 180 degrees per second
#define DEFAULT_YAW_ANGLE_CHANGE_PER_SECOND 180.0

// to move forward for example, the vehicle will tilt up to -20/20 degrees
#define MAX_PITCH_CONTROL_ANGLE 32.0

// to move side-to-side, the vehicle will tilt up to -20/20 degrees
#define MAX_ROLL_CONTROL_ANGLE 32.0

// How much does the throttle contribute to motor output vs. PID/IMU?
#define DEFAULT_THROTTLE_SCALING 1.0f

/**
 * This class assumes a traditional X configuration quadcopter
 *
 *             (motor 1)
 *                |  |
 *                |  |
 *    (motor 2)===    ===(motor 3)
 *                |  |
 *                |  |
 *             (motor 4)
 *
 * TODO: Add support for other motor layouts, such as hexacopter,
 * X copter, tricopter, etc.
*/

class PIDController;

class QuadcopterController
{
    public:
        // Call this in arduino setup()
        QuadcopterController(quadcopter_config_t config,
                             DebugHelper *debugHelper,
                             unsigned long timeMicros,
                             double minThrottle = 1000.0f,
                             double maxThrottle = 2000.0f);

        /**
         * This is used to essentially tell the quadcopter how far to tilt
         * itself to move in any given direction/axis.
         * If you move the stick all the way forwards, this will indicate
         * how far the craft should tilt forward from the "default" hover
         *
         * controlScalingValues:
         *  yaw: indicates max angle change allowed per second in yaw (default = 180 degrees per second)
         *  pitch and roll: max allowed angle set point. So if you move the stick all the way forward,
         *    this means the drone will tilt forward at this angle.
         * throttleScaling: Indicates what max percent of motor activity is determined by throttle.
        */
        void setControlScalingValues(position_values_t controlScalingValues, float throttleScaling);

        // Should be called at a relatively constant frequency with
        // new accel/gyro/compass readings.
        // motor 1 = index 0, motor 2 = index 1, etc.
        motor_outputs_t calculateOutputs(position_values_t imuValues,
                                         controller_values_t controllerValues,
                                         unsigned long timeMicros,
                                         bool recordData);

        void startMonitoringPID(void);

        void reset(unsigned long timestamp);

    private:
        PIDController *_yawController;
        PIDController *_pitchController;
        PIDController *_rollController;
        double _throttle;
        float _throttleScaling;
        double _minThrottle;
        double _maxThrottle;
        double _previousUpdateMicros;
        quadcopter_config_t _config;
        position_values_t _controlScalingValues;
        bool _monitoringPid;
        DebugHelper *_debugHelper;
        unsigned long _startTime;
};

#endif
