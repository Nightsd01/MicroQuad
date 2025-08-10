#pragma once

#include "CrossPlatformEnum.h"

CROSS_PLATFORM_ENUM(uint8_t, TelemetryEvent){
    ArmStatusChange = 0,
    BatteryStatusUpdate = 1,
    EulerYawPitchRoll = 2,
    AccelerometerXYZRaw = 3,
    AccelerometerXYZFiltered = 4,
    GyroXYZRaw = 5,
    GyroXYZFiltered = 6,
    MemoryStats = 7,
    MotorValues = 8,
    LoopUpdateRate = 9,
    IMUUpdateRate = 10,
    MagnetometerXYZRaw = 11,
    VL53L1XRawDistance = 12,
    VL53L1XEstimatedAltitudeUpdate = 13,
    EKFAltitudeEstimate = 14,
    EKFVerticalVelocityEstimate = 15,
    GPSFixData = 16,
    Count = 17,  // NOTE: keep this last
};
