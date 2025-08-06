#pragma once

#include "CrossPlatformEnum.h"

CROSS_PLATFORM_ENUM(uint8_t, BLELargeDataBlobType){
    GPSStartupFixData = 0,  // contains lat/long estimate, timestamp, and ephemeris data
};
