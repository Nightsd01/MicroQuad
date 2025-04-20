#include "VL53Manager.h"

#ifndef MATLAB_SIM

#include <cmath>      // For cos() and M_PI (or define PI manually)
#include <limits>     // For NaN checking/return
#include <stdexcept>  // For potentially throwing errors on invalid input
#include <vector>     // Although input is float[3], vector might be safer if size isn't guaranteed

#include "Constants.h"
#include "Logger.h"
#include "TelemetryController.h"
#include "TelemetryEvent.h"

// Define a maximum tilt angle beyond which the reading is considered unreliable for altitude
constexpr float MAX_RELIABLE_TILT_DEG = 60.0f;  // Example: 60 degrees

#define DISTANCE_MODE VL53L1X::Long

VL53Manager::VL53Manager() : _isInitialized(false) {}

static uint64_t _getDataUpdateHzForDistanceMode(VL53L1X::DistanceMode distanceMode)
{
  switch (distanceMode) {
    case VL53L1X::Short:
      return 20;
    case VL53L1X::Medium:
      return 33;
    case VL53L1X::Long:
      return 50;
    case VL53L1X::Unknown:
      LOG_ERROR("VL53Manager - Unknown distance mode");
      return 50;  // Default to the longest ranging mode
  }
  abort();
}

// The device is not assumed to be flying perfectly level, and so we need
// to correct for the pitch and roll of the device to get an actual altitude estimate
// This is still imperfect because it assumes the world is perfectly flat beneath
// the device, but it is a good first approximation
static inline float _getAltitudeMetersCorrectedForAttitude(float rawDistanceMeters, const EulerAngle &yawPitchRoll)
{
  // Check for valid distance reading (must be positive)
  if (rawDistanceMeters <= 0.0f || !std::isfinite(rawDistanceMeters)) {
    LOG_WARN("Got an invalid distance reading: %f", rawDistanceMeters);
    return std::numeric_limits<float>::quiet_NaN();  // Return NaN for invalid distance
  }

  // Check if the tilt is too extreme for a reliable correction
  // At high angles, the sensor points sideways, not downwards.
  if (std::abs(yawPitchRoll.pitch) > MAX_RELIABLE_TILT_DEG || std::abs(yawPitchRoll.roll) > MAX_RELIABLE_TILT_DEG) {
    LOG_WARN(
        "Tilt angle too high for reliable altitude correction: pitch = %f, roll = %f",
        yawPitchRoll.pitch,
        yawPitchRoll.roll);
    return std::numeric_limits<float>::quiet_NaN();  // Return NaN if tilt is excessive
  }

  // Convert pitch and roll from degrees to radians
  const float pitch_rad = DEG_TO_RADS(yawPitchRoll.pitch);
  const float roll_rad = DEG_TO_RADS(yawPitchRoll.roll);

  // Calculate the cosine factor based on pitch and roll
  // cos(tilt_angle) = cos(pitch) * cos(roll)
  const float cosFactor = std::cos(pitch_rad) * std::cos(roll_rad);

  // Correct the distance: altitude = rawDistance * cosFactor
  // Ensure cosFactor is positive (should be, given the MAX_RELIABLE_TILT_DEG check)
  if (cosFactor <= 0) {
    // This should theoretically not happen with the angle limits, but as safety check:
    LOG_ERROR("Cosine factor is non-positive: %f", cosFactor);
    return std::numeric_limits<float>::quiet_NaN();
  }

  const float correctedAltitude_m = rawDistanceMeters * cosFactor;

  return correctedAltitude_m;
}

void VL53Manager::begin(
    std::function<void(float)> altitudeHandler, TelemetryController *telemController, uint8_t address, TwoWire *theWire)
{
  if (!_vl53l1x.init()) {
    LOG_ERROR("Could not find a valid VL53L1X sensor, check wiring!");
    return;
  }
  _dataUpdateRateHz = _getDataUpdateHzForDistanceMode(DISTANCE_MODE);
  _vl53l1x.setAddress(address);
  _vl53l1x.setDistanceMode(DISTANCE_MODE);

  _vl53l1x.setMeasurementTimingBudget((1000.0f / _dataUpdateRateHz) * 1000.0f);
  _vl53l1x.startContinuous(1000.0f / _dataUpdateRateHz);

  _altitudeHandler = altitudeHandler;
  _telemController = telemController;

  LOG_INFO("Connected to VL53L1X range finding sensor");
  _isInitialized = true;
}

void VL53Manager::loopHandler(void)
{
  if (!_isInitialized) {
    return;
  }
  if (millis() - _lastUpdateTimestampMillis < 1000.0f / _dataUpdateRateHz) {
    return;
  }
  if (!_gotFirstIMUUpdate) {
    return;
  }
  _lastUpdateTimestampMillis = millis();

  _vl53l1x.read(false);

  if (!_vl53l1x.dataReady()) {
    LOG_INFO_PERIODIC_MILLIS(1000, "VL53L1X data not ready");
    return;
  }

  // Convert to meters
  float rangingDistanceMeters = static_cast<float>(_vl53l1x.ranging_data.range_mm) / 1000.0f;

  // correct for attitude of the platform
  float distanceMeters = _getAltitudeMetersCorrectedForAttitude(rangingDistanceMeters, _yawPitchRoll);

  if (std::isnan(distanceMeters)) {
    LOG_WARN("Got an invalid distance reading: %f", rangingDistanceMeters);
    return;  // don't send NaN values
  }

  // Call the altitude handler with the distance
  _altitudeHandler(distanceMeters);

  EXECUTE_PERIODIC(250, {
    _telemController->updateTelemetryEvent(TelemetryEvent::VL53L1XRawDistance, &rangingDistanceMeters, sizeof(float));
    _telemController->updateTelemetryEvent(
        TelemetryEvent::VL53L1XEstimatedAltitudeUpdate,
        &distanceMeters,
        sizeof(float));
  });
}

void VL53Manager::updatedAttitude(const EulerAngle &yawPitchRoll)
{
  _gotFirstIMUUpdate = true;
  _yawPitchRoll = yawPitchRoll;
}
#endif  // MATLAB_SIM