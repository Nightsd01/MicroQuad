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

  controller->setPIDConstantsUpdateHandler([this](ControlAxis axis, PIDType type, gains_t gains) {
    AsyncController::main.executeAsync([this, axis, type, gains]() { this->_updateGains(axis, type, gains); });
  });
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
  LOG_INFO("Updating gains, axis = %i, type = %i, kP = %s", gains.kP);
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
    _kvStore->setVectorForKey<float>(_getAxisKey(axis), newGains);
  } else {
    std::vector<float> newGains = {
        (float)this->gains.verticalVelocityGains.kP,
        (float)this->gains.verticalVelocityGains.kI,
        (float)this->gains.verticalVelocityGains.kD,
    };
    _kvStore->setVectorForKey<float>(_getAxisKey(axis), newGains);
  }
}

quadcopter_config_t PIDPreferences::_initializeGains()
{
  const std::string yawKey = _getAxisKey(ControlAxis::Yaw);
  const std::string pitchKey = _getAxisKey(ControlAxis::Pitch);
  const std::string rollKey = _getAxisKey(ControlAxis::Roll);
  const std::string verticalKey = _getAxisKey(ControlAxis::Vertical);

  if (_kvStore->hasValueForKey(yawKey) && _kvStore->hasValueForKey(pitchKey) && _kvStore->hasValueForKey(rollKey)) {
    const std::vector<float> yawGains = _kvStore->getVectorForKey<float>(yawKey, 6);
    const std::vector<float> pitchGains = _kvStore->getVectorForKey<float>(pitchKey, 6);
    const std::vector<float> rollGains = _kvStore->getVectorForKey<float>(rollKey, 6);
    const std::vector<float> verticalGains = _kvStore->getVectorForKey<float>(verticalKey, 3);

    if (yawGains.size() == 6 && pitchGains.size() == 6 && rollGains.size() == 6 && verticalGains.size() == 3) {
      return {
          .angleGains =
              {{// yaw
                .kP = yawGains[0],
                .kI = yawGains[1],
                .kD = yawGains[2]},
                           {// pitch
                .kP = pitchGains[0],
                .kI = pitchGains[1],
                .kD = pitchGains[2]},
                           {// roll
                .kP = rollGains[0],
                .kI = rollGains[1],
                .kD = rollGains[2]}},
          .rateGains =
              {{// yaw
                .kP = yawGains[3],
                .kI = yawGains[4],
                .kD = yawGains[5]},
                           {// pitch
                .kP = pitchGains[3],
                .kI = pitchGains[4],
                .kD = pitchGains[5]},
                           {// roll
                .kP = rollGains[3],
                .kI = rollGains[4],
                .kD = rollGains[5]}},
          .verticalVelocityGains =
              {
                           .kP = verticalGains[0],
                           .kI = verticalGains[1],
                           .kD = verticalGains[2],
                           },
      };
    } else {
      LOG_ERROR("Failed to get PID gains from KV store - corrupt/invalid gains, using defaults");
    }

    LOG_WARN("Using default PID gains");
    return _defaultGains;
  }

  return _defaultGains;
}
#endif  // MATLAB_SIM