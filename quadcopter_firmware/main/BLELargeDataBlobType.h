#pragma once

#include <stdint.h>

#include "CrossPlatformEnum.h"

CROSS_PLATFORM_ENUM(uint8_t, BLELargeDataBlobType){
    GPSStartupFixData = 0,  // contains lat/long estimate, timestamp, and ephemeris data
    TelemetryData = 1,
};

// Packed header used for L2CAP CoC transmissions
// Layout: [type: 1 byte][payload_size_bytes: 4 bytes little-endian]
#pragma pack(push, 1)
struct BLELargeDataBlobHeader
{
  enum BLELargeDataBlobType type;
  uint32_t payloadSizeBytes;
};
#pragma pack(pop)
