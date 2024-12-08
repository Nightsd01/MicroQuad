#pragma once

#include "CrossPlatformEnum.h"

#define NUM_CALIBRATION_STAGES 6

CROSS_PLATFORM_ENUM(uint8_t, CalibrationRequest){
    PlaceFlat = 0, PitchUp = 1, PitchDown = 2, UpsideDown = 3, RollRight = 4, RollLeft = 5, Complete = 6, Failed = 7};

CROSS_PLATFORM_ENUM(uint8_t, CalibrationResponse){
    Start = 0,
    Continue = 1,
    Cancel = 2,
};

CROSS_PLATFORM_ENUM(uint8_t, CalibrationType){Accelerometer = 0, Gyro = 1, Magnetometer = 2};
