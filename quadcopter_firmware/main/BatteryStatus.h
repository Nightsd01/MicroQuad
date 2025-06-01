#pragma once

#include <cstdint>
#include "CrossPlatformEnum.h"

// Battery status bitmask based on MCP73871 status pins
// Bit 0 = STAT1 (0 = LOW, 1 = HIGH/Hi-Z)
// Bit 1 = STAT2 (0 = LOW, 1 = HIGH/Hi-Z)  
// Bit 2 = PG    (0 = LOW, 1 = HIGH/Hi-Z)
CROSS_PLATFORM_ENUM(uint8_t, BatteryStatus){
    // 000 = STAT1:L, STAT2:L, PG:L
    // Temperature Fault OR Timer Fault (cannot distinguish)
    Fault = 0b000,
    
    // 001 = STAT1:Hi-Z, STAT2:L, PG:L
    // Charge Complete - Standby
    ChargeComplete = 0b001,
    
    // 010 = STAT1:L, STAT2:Hi-Z, PG:L
    // Preconditioning OR Constant Current OR Constant Voltage (cannot distinguish)
    Charging = 0b010,
    
    // 011 = STAT1:Hi-Z, STAT2:Hi-Z, PG:L
    // Shutdown (VDD = IN) OR Shutdown (CE = L) OR No Battery Present (cannot distinguish)
    ShutdownOrNoBattery = 0b011,
    
    // 100 = STAT1:L, STAT2:L, PG:Hi-Z
    // Not defined in truth table
    Undefined100 = 0b100,
    
    // 101 = STAT1:Hi-Z, STAT2:L, PG:Hi-Z
    // Not defined in truth table
    Undefined101 = 0b101,
    
    // 110 = STAT1:L, STAT2:Hi-Z, PG:Hi-Z
    // Low Battery Output
    LowBattery = 0b110,
    
    // 111 = STAT1:Hi-Z, STAT2:Hi-Z, PG:Hi-Z
    // Shutdown (VDD = VBAT) OR No Input Power Present (cannot distinguish)
    NoInputPower = 0b111
}; 

// Struct for battery status telemetry event
typedef struct {
    float voltage;
    BatteryStatus status;
} battery_status_event_t;