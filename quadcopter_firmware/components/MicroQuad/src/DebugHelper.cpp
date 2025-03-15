#include "DebugHelper.h"

#include <stdio.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "Arduino.h"
#include "Logger.h"

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
  dataManager.addDouble(magValues, 4);
  dataManager.addDouble(&throttle, 1);
  dataManager.addDouble(&voltage, 1);
  dataManager.addDouble(setPoints, 3);
  dataManager.numSamples++;
}