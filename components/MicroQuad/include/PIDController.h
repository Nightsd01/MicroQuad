#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

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
        // void setParameters(unsigned long initialTimeMillis,
        //                    quadcopter_tuning_parameters_t parameters);
        // double calculate(double setPoint, double input, unsigned long currentTimeMillis);
        PIDController();
        PIDController(float,float,float);
        float update_pid_std(float setpt, float input, float dt);
        void  updateKpKi(float setpt, float input);
        void  set_Kpid(float, float, float);
        void  set_windup_bounds(float, float);
        void  reset();
        float setpoint;

    private:
        quadcopter_tuning_parameters_t _params;
        //PID constants
        float m_Kp;
        float m_Ki;
        float m_Kd;

        //PID constants
        float m_err;
        float m_last_err;
        float m_sum_err;
        float m_ddt_err;
        float m_lastInput;
        float m_outmax;
        float m_outmin;
        float m_output;
};

#endif
