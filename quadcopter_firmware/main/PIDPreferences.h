#pragma once

#ifndef MATLAB_SIM

#include <PIDController.h>
#include <QuadcopterController.h>

#include <vector>

#include "CalibrationEvent.h"
#include "CrossPlatformEnum.h"

class BLEController;
class PersistentKeyValueStore;

class PIDPreferences
{
 public:
  PIDPreferences(BLEController *controller, PersistentKeyValueStore *kvStore);

  quadcopter_config_t gains;
  
  // Serialize all PID gains for transmission over BLE
  std::vector<uint8_t> serializeGains() const;

 private:
  PersistentKeyValueStore *_kvStore;
  quadcopter_config_t _initializeGains();
  void _updateGains(ControlAxis axis, PIDType type, gains_t gains);
};

#endif  // MATLAB_SIM