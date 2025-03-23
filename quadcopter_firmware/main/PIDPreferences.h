#pragma once

#include <PIDController.h>
#include <QuadcopterController.h>

#include "CalibrationEvent.h"
#include "CrossPlatformEnum.h"

class BLEController;
class PersistentKeyValueStore;

class PIDPreferences
{
 public:
  PIDPreferences(BLEController *controller, PersistentKeyValueStore *kvStore);

  quadcopter_config_t gains;

 private:
  PersistentKeyValueStore *_kvStore;
  quadcopter_config_t _initializeGains();
  void _updateGains(ControlAxis axis, PIDType type, gains_t gains);
};