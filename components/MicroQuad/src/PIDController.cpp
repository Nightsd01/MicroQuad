#include "PIDController.h"

#include <math.h>


PIDController yprSTAB[3];
PIDController yprRATE[3];

//default constructor
PIDController::PIDController()
{
  //PID constants
  m_Kd = 0;
  m_Ki = 0;
  m_Kp = 0;

  //PID variables
  m_err = 0;
  m_last_err=0;
  m_sum_err = 0;
  m_ddt_err = 0;
  m_lastInput= 0;
  m_outmax =  350;
  m_outmin = -350;
}


PIDController::PIDController(float kp_,float ki_,float kd_)
{
  //PIDController constants
  m_Kp = kp_;
  m_Ki = ki_;
  m_Kd = kd_;

  //PIDController variables
  m_err = 0;
  m_last_err=0;
  m_sum_err = 0;
  m_ddt_err = 0;
  m_lastInput= 0;
  m_outmax =  350;
  m_outmin = -350;
}

float PIDController::update_pid_std(float setpoint, float input, float dt)
{

  //Computes error
  m_err = setpoint-input;

  //Integrating errors
  m_sum_err += m_err * m_Ki * dt;

  //calculating error derivative
  //Input derivative is used to avoid derivative kick
  m_ddt_err = -m_Kd / dt * (input - m_lastInput);

  //Calculation of the output
  //winds up boundaries
  m_output = m_Kp*m_err + m_sum_err + m_ddt_err;
  if (m_output > m_outmax) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmax;
  }else if (m_output < m_outmin) {
    //winds up boundaries
    m_sum_err  = 0.0;
    m_output   = m_outmin;
  }

  m_lastInput= input;

  // printf("kp %f ki %f kd %f\n", m_Kp, m_Ki, m_Kd);
  // printf("setpt %7.2f input   %7.2f output   %f\n", setpoint, input, m_output);
  // printf("err   %7.2f ddt_err %7.2f sum_err  %7.2f\n", m_err, m_ddt_err, m_sum_err);

  return m_output;
}

void PIDController::reset()
{
  m_sum_err   = 0;
  m_ddt_err   = 0;
  m_lastInput = 0;
}

void PIDController::set_Kpid(float Kp,float Ki, float Kd)
{
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;

  // printf("%f %f %f \n", m_Kp, m_Ki, m_Kd);
  // printf("%f %f %f \n", Kp, Ki, Kd);

}

void PIDController::set_windup_bounds(float Min,float Max)
{
  m_outmax = Max;
  m_outmin = Min;
}

void PIDController::updateKpKi(float setpoint, float input)
{
  //err calculation
  float err = setpoint-input;
  float derr=0;
  float err_abs = fabs(err);

  if (err - m_last_err == 0.0){
    return;
  }

  derr = (err - m_last_err)/fabs(err - m_last_err);

  if (err_abs >= 7.0){
    m_Kp += derr*0.005*err_abs;
  }

  if (err_abs < 1.0){
    m_Kp -= 0.01*err_abs;
  }

  if (m_Kp < 1.5) {
    m_Kp = 1.5;
  }

  if (m_Kp > 5.0) {
    m_Kp = 5.0;
  }

  m_last_err = err;

  // if (err_abs > 4.0 && err_abs < 6.0){
  //   m_Ki += 0.05*err;
  //   return;
  // }
}