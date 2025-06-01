#include "BatteryController.h"

#ifndef MATLAB_SIM

#include "BatteryStatus.h"
#include "Constants.h"
#include "DebugHelper.h"
#include "Logger.h"
#include "TelemetryController.h"

#define BATTERY_SCALE 0.001639280125196f

BatteryController::BatteryController(
    TelemetryController *telemetryController, DebugHelper *debugHelper)
{
  _telemetryController = telemetryController;
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
    // Read battery voltage
    float voltage = batteryVoltage();
    
    // Read status pins
    const int stat1 = digitalRead(BATTERY_STAT1_PIN);
    const int stat2 = digitalRead(BATTERY_STAT2_PIN);
    const int pg = digitalRead(BATTERY_PG_PIN);
    
    // Construct battery status bitmask
    // HIGH = Hi-Z = 1, LOW = L = 0
    BatteryStatus status = (BatteryStatus)(
        ((stat1 == HIGH) ? 0x01 : 0x00) |  // Bit 0: STAT1
        ((stat2 == HIGH) ? 0x02 : 0x00) |  // Bit 1: STAT2
        ((pg == HIGH)    ? 0x04 : 0x00)    // Bit 2: PG
    );
    
    // Create battery status event
    battery_status_event_t batteryEvent;
    batteryEvent.voltage = voltage;
    batteryEvent.status = status;
    
    // Send telemetry update
    _telemetryController->updateTelemetryEvent(
        TelemetryEvent::BatteryStatusUpdate, 
        &batteryEvent, 
        sizeof(battery_status_event_t));
    
    // Update debug helper
    _debugHelper->voltage = voltage;
  });
}
#endif  // MATLAB_SIM
