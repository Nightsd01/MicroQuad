#pragma once

#include <QMC5883L.h>
#include <QuadcopterController.h>

#include <array>
#include <functional>

class PersistentKeyValueStore;

#define CALIBRATION_MOTOR_STEPS 10
#define CALIBRATION_STEP_TIME_MS 1000

class MotorMagCompensationHandler
{
 public:
  MotorMagCompensationHandler(QMC5883L *magSensor, PersistentKeyValueStore *kvStore);

  bool isCalibrating;
  bool isCalibrated;

  // callback returns a boolean indicating if calibration succeeded/failed
  void beginCalibration(
      std::function<void(motor_outputs_t)> motorOutputCallback, std::function<void(bool)> calibrationCompleteCallback);

  void updateMagValue(mag_update_t magValues);

 private:
  QMC5883L *_magSensor;
  PersistentKeyValueStore *_kvStore;
  std::function<void(motor_outputs_t)> _motorOutputCallback;
  std::function<void(bool)> _calibrationCompleteCallback;
  std::array<std::array<mag_update_t, CALIBRATION_MOTOR_STEPS>, NUM_MOTORS> _perMotorCalibrationData;
  std::vector<mag_update_t> _currentStepMagValues;

  // Calibration proceeds in CALIBRATION_MOTOR_STEPS steps (0%, 25%, etc.) for each motor 0-3
  // Calibration will average out the magnetometer values over a period
  // of CALIBRATION_STEP_TIME_MS milliseconds per step
  int _currentlyCalibratingMotor;
  int _currentMotorCalibrationStep;
  unsigned long _lastCalibrationStepTimeMillis;

  void _completeCalibration(void);
};