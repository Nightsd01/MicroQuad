#pragma once

#include <string>
#include <vector>

#include "DebugDataManager.h"

class DebugHelper
{
 public:
  double ypr[3];
  double accelRaw[3];
  double gyroRaw[3];
  double accelFiltered[3];
  double gyroFiltered[3];
  double angleOutputs[3];
  double rateOutputs[3];
  double motorValues[4];
  double magValues[4];  // x, y, z, heading
  double throttle;
  double inputTimescale;
  double voltage;
  double setPoints[3];
  double ekfQuaternion[4];
  double ekfYawPitchRoll[3];
  double ekfAltitude;
  double ekfVerticalVelocity;
  double relativeAltitudeBarometer;
  double relativeAltitudeVL53;

#ifndef MATLAB_SIM
  void saveValues(unsigned long timestamp);
#else   // MATLAB_SIM
  void saveValues(unsigned long timestamp) {};
#endif  // MATLAB_SIM
  DebugDataManager dataManager;
};
