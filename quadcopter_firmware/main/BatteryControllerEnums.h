#pragma once

#include "CrossPlatformEnum.h"

CROSS_PLATFORM_ENUM(uint8_t, BatteryChargeStatus){
    Shutdown = 0,
    Charging = 1,
    ChargeComplete = 2,
    Fault = 3,
    LowBatteryOutput = 4,
    NoBattery = 5,
    NoInputPowerPresent = 6};

CROSS_PLATFORM_ENUM(uint8_t, BatteryControlEvent){EnableCharging = 0, DisableCharging = 1};