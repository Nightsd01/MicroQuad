#pragma once

#ifndef MATLAB_SIM

#include "BatteryStatus.h"

class BLEController;
class DebugHelper;
class TelemetryController;

class BatteryController
{
 public:
  BatteryController(
      TelemetryController *telemetryController, DebugHelper *debugHelper);
  void loopHandler(void);
  float batteryVoltage(void);

 private:
  TelemetryController *_telemetryController;
  BLEController *_bluetoothController;
  DebugHelper *_debugHelper;
};

#endif  // MATLAB_SIM