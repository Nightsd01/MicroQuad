#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "DebugHelper.h"

enum PIDAxis {
    yaw, pitch, roll
};

struct gains_t {
    double proportionalGain; // Kp
    double integralGain; // Ki
    double derivativeGain; // Kd
};

struct quadcopter_tuning_parameters_t {
    gains_t gains;
    double outputMax;
    double outputMin;
};

class PIDController
{
    public:
        PIDController(DebugHelper *helper, quadcopter_tuning_parameters_t params);

        double compute(
            double setPoint,
            double imuValue,
            unsigned long timestamp,
            bool enableIntegral,
            PIDAxis axis
        );

        void reset(unsigned long timestamp);

    private:
        quadcopter_tuning_parameters_t _params;
        unsigned long _previousTimestamp;
        double _terms[3];
        double _iState;
        double _previousError;
        DebugHelper *_helper;
};

#endif
