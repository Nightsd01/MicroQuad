#ifndef DEBUGHELPER_H
#define DEBUGHELPER_H

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
  double magHeading;
  double throttle;
  double inputTimescale;
  double voltage;
  void saveValues(unsigned long timestamp);
  DebugDataManager dataManager;
};

#endif
