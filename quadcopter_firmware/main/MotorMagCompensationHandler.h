#pragma once

#ifndef MATLAB_SIM

#include <QMC5883L.h>
#include <QuadcopterController.h>

#include <array>
#include <functional>

class PersistentKeyValueStore;

#define CALIBRATION_MOTOR_STEPS 10
#define CALIBRATION_STEP_TIME_MS 1000
#define NOMINAL_VOLTAGE 3.85f
#define SPINUP_MS 200  // ★ warm‑up time

struct Sum
{
  float x{0}, y{0}, z{0}, v{0};  // ★ add voltage
  uint32_t n{0};
  void accumulate(const mag_update_t &m, float volt)
  {  // ★
    x += m.x;
    y += m.y;
    z += m.z;
    v += volt;
    ++n;
  }
  void reset()
  {
    x = y = z = v = 0;
    n = 0;
  }
  mag_update_t meanMag() const
  {
    const float inv = (n ? 1.0f / n : 1.0f);
    mag_update_t m{x * inv, y * inv, z * inv, 0};
    m.heading = std::atan2(-m.y, m.x);
    return m;
  }
  float meanVolt() const { return (n ? v / n : NOMINAL_VOLTAGE); }  // ★
};

class MotorMagCompensationHandler
{
 public:
  MotorMagCompensationHandler(PersistentKeyValueStore *kvStore);

  bool isCalibrating;
  bool isCalibrated;

  // callback returns a boolean indicating if calibration succeeded/failed
  void beginCalibration(
      std::function<void(motor_outputs_t)> motorOutputCallback, std::function<void(bool)> calibrationCompleteCallback);

  void updateMagValue(const mag_update_t &mag, float voltage);

  mag_update_t applyMagneticMotorCompensation(
      const mag_update_t &rawMagData, const motor_outputs_t &motorThrottles, const float liveVoltage);

 private:
  QMC5883L *_magSensor;
  PersistentKeyValueStore *_kvStore;
  std::function<void(motor_outputs_t)> _motorOutputCallback;
  std::function<void(bool)> _calibrationCompleteCallback;
  uint32_t _lastStepTs = 0;
  int _curMotor = 0;
  int _curStep = 0;
  Sum _stepAcc;  // running sum for this 1‑s window
  uint32_t _spinUpStartTs = 0;
  bool _inSpinUp = false;

  std::array<std::array<mag_update_t, CALIBRATION_MOTOR_STEPS>, NUM_MOTORS> _perMotorCalibrationData;
  std::array<std::array<float, CALIBRATION_MOTOR_STEPS>, NUM_MOTORS> _perMotorCalibrationVoltage;
  std::vector<mag_update_t> _currentStepMagValues;

  // Calibration proceeds in CALIBRATION_MOTOR_STEPS steps (0%, 25%, etc.) for each motor 0-3
  // Calibration will average out the magnetometer values over a period
  // of CALIBRATION_STEP_TIME_MS milliseconds per step
  int _currentlyCalibratingMotor;
  int _currentMotorCalibrationStep;
  unsigned long _lastCalibrationStepTimeMillis;

  void _completeCalibration(void);
};

#endif  // MATLAB_SIM