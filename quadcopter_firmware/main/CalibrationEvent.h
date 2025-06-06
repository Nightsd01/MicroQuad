#pragma once

#include "CrossPlatformEnum.h"

#define NUM_ACCELGYRO_CALIBRATION_STAGES 6

CROSS_PLATFORM_ENUM(uint8_t, CalibrationRequest){
    PlaceFlat = 0,
    PitchUp = 1,
    PitchDown = 2,
    UpsideDown = 3,
    RollRight = 4,
    RollLeft = 5,
    Roll360 = 6,
    Complete = 7,
    Failed = 8};

CROSS_PLATFORM_ENUM(uint8_t, CalibrationResponse){
    Start = 0,
    Continue = 1,
    Cancel = 2,
};

CROSS_PLATFORM_ENUM(uint8_t, CalibrationType){
    AccelerometerGyro = 0, Magnetometer = 1, MagnetometerMotorsCompensation = 2};

CROSS_PLATFORM_ENUM(uint8_t, ControlAxis){Yaw = 0, Pitch = 1, Roll = 2, Vertical = 3};

CROSS_PLATFORM_ENUM(uint8_t, PIDType){Angle = 0, Rate = 1, VerticalVelocity = 2};
