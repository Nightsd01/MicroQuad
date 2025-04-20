#include "BatteryController.h"

#ifndef MATLAB_SIM

#include "Constants.h"
#include "DebugHelper.h"
#include "Logger.h"
#include "TelemetryController.h"

#define BATTERY_SCALE 0.001639280125196f

BatteryController::BatteryController(
    TelemetryController *telemetryController, BLEController *bluetoothController, DebugHelper *debugHelper)
{
  _telemetryController = telemetryController;
  _bluetoothController = bluetoothController;
  _debugHelper = debugHelper;

  LOG_INFO("Enabling charging");
  pinMode(BATTERY_CHARGE_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_CHARGE_ENABLE_PIN, HIGH);

  pinMode(BATTERY_STAT1_PIN, INPUT);
  pinMode(BATTERY_STAT2_PIN, INPUT);
  pinMode(BATTERY_PG_PIN, INPUT);
}

float BatteryController::batteryVoltage(void)
{
  const int val = analogRead(BATTERY_SENSE_PIN);
  return BATTERY_SCALE * (float)val;
}

void BatteryController::loopHandler(void)
{
  // No need to execute this too frequently, once every 100ms is ok
  EXECUTE_PERIODIC(100, {
    float voltage = batteryVoltage();
    _telemetryController->updateTelemetryEvent(TelemetryEvent::BatteryVoltage, &voltage, sizeof(float));
    _debugHelper->voltage = voltage;
  });
}
#endif  // MATLAB_SIM