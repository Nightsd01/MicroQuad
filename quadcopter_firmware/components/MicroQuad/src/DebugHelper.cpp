#include "DebugHelper.h"

#ifndef MATLAB_SIM

#include <stdio.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "Arduino.h"
#include "Logger.h"

// double magValuesRaw[4];                            // x, y, z, heading
// double magValuesPostSoftHardMatrixCalibration[4];  // x, y, z, heading
// double magValuesPostMotorMagCompCalibration[4];    // x, y, z, heading
static int sampleRate = 4;
static int sampleCounter = 0;
void DebugHelper::saveValues(unsigned long timestamp)
{
  double ts = (double)timestamp;
  dataManager.addDouble(&ts, 1);
  dataManager.addDouble(ypr, 3);
  dataManager.addDouble(accelRaw, 3);
  dataManager.addDouble(gyroRaw, 3);
  dataManager.addDouble(accelFiltered, 3);
  dataManager.addDouble(gyroFiltered, 3);
  dataManager.addDouble(angleOutputs, 3);
  dataManager.addDouble(rateOutputs, 3);
  dataManager.addDouble(motorValues, 4);
  dataManager.addDouble(magValuesRaw, 4);
  dataManager.addDouble(magValuesPostSoftHardMatrixCalibration, 4);
  dataManager.addDouble(magValuesPostMotorMagCompCalibration, 4);
  dataManager.addDouble(&throttle, 1);
  dataManager.addDouble(&voltage, 1);
  dataManager.addDouble(setPoints, 3);
  dataManager.addDouble(ekfQuaternion, 4);
  dataManager.addDouble(ekfYawPitchRoll, 3);
  dataManager.addDouble(&ekfAltitude, 1);
  dataManager.addDouble(&ekfVerticalVelocity, 1);
  dataManager.addDouble(&relativeAltitudeBarometer, 1);
  dataManager.addDouble(&relativeAltitudeVL53, 1);
  dataManager.numSamples++;
}

#endif  // MATLAB_SIM