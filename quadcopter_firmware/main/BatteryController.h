#pragma once

#ifndef MATLAB_SIM

class BLEController;
class DebugHelper;
class TelemetryController;

class BatteryController
{
 public:
  BatteryController(
      TelemetryController *telemetryController, BLEController *bluetoothController, DebugHelper *debugHelper);
  void loopHandler(void);
  float batteryVoltage(void);

 private:
  TelemetryController *_telemetryController;
  BLEController *_bluetoothController;
  DebugHelper *_debugHelper;
};

#endif  // MATLAB_SIM