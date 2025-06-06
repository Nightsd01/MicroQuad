#pragma once

#include <string>

namespace PersistentKeysCommon
{

// Accelerometer Offsets
const std::string ACCEL_OFFSET_X = "ACCEL_OFFSET_X";
const std::string ACCEL_OFFSET_Y = "ACCEL_OFFSET_Y";
const std::string ACCEL_OFFSET_Z = "ACCEL_OFFSET_Z";

const std::string MAG_OFFSETS = "MAG_OFFSETS";

const std::string GYRO_OFFSET_X = "GYRO_OFFSET_X";
const std::string GYRO_OFFSET_Y = "GYRO_OFFSET_Y";
const std::string GYRO_OFFSET_Z = "GYRO_OFFSET_Z";

const std::string MAG_COMPENSATION_OFFSET_PREFIX = "MAGC_";

const std::string PID_CONSTANTS_PREFIX = "PID_";

const std::string MAG_HARD_IRON_OFFSETS = "HARD_IRON_OFSTS";
const std::string MAG_SOFT_IRON_OFFSETS = "SOFT_IRON_OFSTS";

};  // namespace PersistentKeysCommon