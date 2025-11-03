#include "PIDPreferences.h"

#ifndef MATLAB_SIM

#include <AsyncController.h>
#include <Logger.h>

#include "BLEController.h"
#include "PersistentKeyValueStore.h"
#include "PersistentKeysCommon.h"

static quadcopter_config_t _defaultGains = {
    .angleGains =
        {{// yaw
          .kP = 2.0f,
          .kI = 0.01f,
          .kD = 0.0f},
                     {// pitch
          .kP = 3.0f,
          .kI = 0.02f,
          .kD = 0.01f},
                     {// roll
          .kP = 3.0f,
          .kI = 0.02f,
          .kD = 0.01f}},
    .rateGains =
        {{// yaw
          .kP = 2.1f,
          .kI = 0.01f,
          .kD = 0.0005f},
                     {// pitch
          .kP = 4.0f,
          .kI = 0.02f,
          .kD = 0.01f},
                     {// roll
          .kP = 4.0f,
          .kI = 0.02f,
          .kD = 0.01f}},
    .verticalVelocityGains = {
                     .kP = 8.0f,
                     .kI = 0.10f,
                     .kD = 2.0f,
                     }
};

PIDPreferences::PIDPreferences(BLEController *controller, PersistentKeyValueStore *kvStore)
{
  this->_kvStore = kvStore;
  this->gains = _initializeGains();

  controller->setPIDConstantsUpdateHandler(
      [this](ControlAxis axis, PIDType type, gains_t gains) { this->_updateGains(axis, type, gains); });
}

const std::string _getAxisKey(ControlAxis axis)
{
  switch (axis) {
    case ControlAxis::Yaw:
      return PersistentKeysCommon::PID_CONSTANTS_PREFIX + "YAW";
    case ControlAxis::Pitch:
      return PersistentKeysCommon::PID_CONSTANTS_PREFIX + "PITCH";
    case ControlAxis::Roll:
      return PersistentKeysCommon::PID_CONSTANTS_PREFIX + "ROLL";
    case ControlAxis::Vertical:
      return PersistentKeysCommon::PID_CONSTANTS_PREFIX + "VERT";
    default:
      LOG_ERROR("Invalid axis\n");
      abort();
  }
}

void PIDPreferences::_updateGains(ControlAxis axis, PIDType type, gains_t gains)
{
  LOG_INFO("Updating gains, axis = %i, type = %i, kP = %f", axis, type, gains.kP);
  switch (type) {
    case PIDType::Angle:
      this->gains.angleGains[(int)axis] = gains;
      break;
    case PIDType::Rate:
      this->gains.rateGains[(int)axis] = gains;
      break;
    case PIDType::VerticalVelocity:
      this->gains.verticalVelocityGains = gains;
      break;
  }
  if (axis != ControlAxis::Vertical) {
    std::vector<float> newGains = {
        (float)this->gains.angleGains[(int)axis].kP,
        (float)this->gains.angleGains[(int)axis].kI,
        (float)this->gains.angleGains[(int)axis].kD,
        (float)this->gains.rateGains[(int)axis].kP,
        (float)this->gains.rateGains[(int)axis].kI,
        (float)this->gains.rateGains[(int)axis].kD,
    };
    _kvStore->setValue(_getAxisKey(axis), newGains);
  } else {
    std::vector<float> newGains = {
        (float)this->gains.verticalVelocityGains.kP,
        (float)this->gains.verticalVelocityGains.kI,
        (float)this->gains.verticalVelocityGains.kD,
    };
    _kvStore->setValue(_getAxisKey(axis), newGains);
  }
}

quadcopter_config_t PIDPreferences::_initializeGains()
{
  // Start with the default configuration
  quadcopter_config_t config = _defaultGains;

  // Check if KeyValue store is available
  if (!_kvStore) {
    LOG_ERROR("KeyValue store is null, cannot load PID gains. Using defaults.");
    return config;  // Returns default gains
  }

  LOG_INFO("Loading PID gains from persistent storage...");

  // --- Try to load Yaw gains ---
  const std::string yawKey = _getAxisKey(ControlAxis::Yaw);
  if (_kvStore->hasValueForKey(yawKey)) {
    const std::vector<float> yawGains =
        _kvStore->getValue<float>(yawKey, 6);  // Expect 6 floats (angle P,I,D, rate P,I,D)
    if (yawGains.size() == 6) {
      config.angleGains[0] = {.kP = yawGains[0], .kI = yawGains[1], .kD = yawGains[2]};  // Index 0 assumed Yaw
      config.rateGains[0] = {.kP = yawGains[3], .kI = yawGains[4], .kD = yawGains[5]};   // Index 0 assumed Yaw
      LOG_INFO("Loaded custom Yaw PID gains from KV store.");
    } else {
      LOG_ERROR(
          "Invalid Yaw PID gains found in KV store (size %zu, expected 6), using defaults for Yaw.",
          yawGains.size());
      // No action needed, config already holds defaults for Yaw
    }
  } else {
    LOG_WARN("No Yaw PID gains found in KV store, using defaults for Yaw.");
    // No action needed, config already holds defaults for Yaw
  }

  // --- Try to load Pitch gains ---
  const std::string pitchKey = _getAxisKey(ControlAxis::Pitch);
  if (_kvStore->hasValueForKey(pitchKey)) {
    const std::vector<float> pitchGains = _kvStore->getValue<float>(pitchKey, 6);  // Expect 6 floats
    if (pitchGains.size() == 6) {
      config.angleGains[1] = {.kP = pitchGains[0], .kI = pitchGains[1], .kD = pitchGains[2]};  // Index 1 assumed Pitch
      config.rateGains[1] = {.kP = pitchGains[3], .kI = pitchGains[4], .kD = pitchGains[5]};   // Index 1 assumed Pitch
      LOG_INFO("Loaded custom Pitch PID gains from KV store.");
    } else {
      LOG_ERROR(
          "Invalid Pitch PID gains found in KV store (size %zu, expected 6), using defaults for Pitch.",
          pitchGains.size());
      // No action needed, config already holds defaults for Pitch
    }
  } else {
    LOG_WARN("No Pitch PID gains found in KV store, using defaults for Pitch.");
    // No action needed, config already holds defaults for Pitch
  }

  // --- Try to load Roll gains ---
  const std::string rollKey = _getAxisKey(ControlAxis::Roll);
  if (_kvStore->hasValueForKey(rollKey)) {
    const std::vector<float> rollGains = _kvStore->getValue<float>(rollKey, 6);  // Expect 6 floats
    if (rollGains.size() == 6) {
      config.angleGains[2] = {.kP = rollGains[0], .kI = rollGains[1], .kD = rollGains[2]};  // Index 2 assumed Roll
      config.rateGains[2] = {.kP = rollGains[3], .kI = rollGains[4], .kD = rollGains[5]};   // Index 2 assumed Roll
      LOG_INFO("Loaded custom Roll PID gains from KV store.");
    } else {
      LOG_ERROR(
          "Invalid Roll PID gains found in KV store (size %zu, expected 6), using defaults for Roll.",
          rollGains.size());
      // No action needed, config already holds defaults for Roll
    }
  } else {
    LOG_WARN("No Roll PID gains found in KV store, using defaults for Roll.");
    // No action needed, config already holds defaults for Roll
  }

  // --- Try to load Vertical gains ---
  const std::string verticalKey = _getAxisKey(ControlAxis::Vertical);
  if (_kvStore->hasValueForKey(verticalKey)) {
    const std::vector<float> verticalGains = _kvStore->getValue<float>(verticalKey, 3);  // Expect 3 floats (P,I,D)
    if (verticalGains.size() == 3) {
      config.verticalVelocityGains = {.kP = verticalGains[0], .kI = verticalGains[1], .kD = verticalGains[2]};
      LOG_INFO("Loaded custom Vertical PID gains from KV store.");
    } else {
      LOG_ERROR(
          "Invalid Vertical PID gains found in KV store (size %zu, expected 3), using defaults for Vertical.",
          verticalGains.size());
      // No action needed, config already holds defaults for Vertical
    }
  } else {
    LOG_WARN("No Vertical PID gains found in KV store, using defaults for Vertical.");
    // No action needed, config already holds defaults for Vertical
  }

  // Return the config, which contains defaults potentially overwritten by loaded values
  return config;
}

std::vector<uint8_t> PIDPreferences::serializeGains() const
{
  // Serialize all PID gains in the following order:
  // - angleGains: yaw(P,I,D), pitch(P,I,D), roll(P,I,D) = 9 floats
  // - rateGains: yaw(P,I,D), pitch(P,I,D), roll(P,I,D) = 9 floats
  // - verticalVelocityGains: (P,I,D) = 3 floats
  // Total: 21 floats = 84 bytes

  std::vector<uint8_t> data;
  data.reserve(21 * sizeof(float));

  // Helper lambda to append a float as bytes
  auto appendFloat = [&data](float value) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
    data.insert(data.end(), bytes, bytes + sizeof(float));
  };

  // Serialize angleGains (indices: 0=yaw, 1=pitch, 2=roll)
  for (int i = 0; i < 3; i++) {
    appendFloat(gains.angleGains[i].kP);
    appendFloat(gains.angleGains[i].kI);
    appendFloat(gains.angleGains[i].kD);
  }

  // Serialize rateGains (indices: 0=yaw, 1=pitch, 2=roll)
  for (int i = 0; i < 3; i++) {
    appendFloat(gains.rateGains[i].kP);
    appendFloat(gains.rateGains[i].kI);
    appendFloat(gains.rateGains[i].kD);
  }

  // Serialize verticalVelocityGains
  appendFloat(gains.verticalVelocityGains.kP);
  appendFloat(gains.verticalVelocityGains.kI);
  appendFloat(gains.verticalVelocityGains.kD);

  return data;
}

#endif  // MATLAB_SIM