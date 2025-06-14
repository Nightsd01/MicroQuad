#pragma once

#ifndef MATLAB_SIM

#include <cstdint>  // Ensure uint8_t is defined

#include "BatteryStatus.h"

class BLEController;
class DebugHelper;
class TelemetryController;

// Struct for battery status telemetry event
typedef struct
{
  float voltage;
  BatteryStatus status;
} battery_status_event_t;

class BatteryController
{
 public:
  BatteryController(TelemetryController *telemetryController, DebugHelper *debugHelper);
  void loopHandler(void);
  float batteryVoltage(void);

 private:
  TelemetryController *_telemetryController;
  BLEController *_bluetoothController;
  DebugHelper *_debugHelper;
};

#endif  // MATLAB_SIM