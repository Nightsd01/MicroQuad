#pragma once

#ifndef CROSS_PLATFORM_ENUM
#define CROSS_PLATFORM_ENUM(_type, _name) enum _name : _type
#endif

CROSS_PLATFORM_ENUM(uint8_t, TelemetryEvent){
    ArmStatusChange = 0,
    BatteryVoltage = 1,
    EulerYawPitchRoll = 2,
    AccelerometerXYZRaw = 3,
    AccelerometerXYZFiltered = 4,
    GyroXYZRaw = 5,
    GyroXYZFiltered = 6,
    MemoryStats = 7,
    MotorValues = 8,
    LoopUpdateRate = 9};
