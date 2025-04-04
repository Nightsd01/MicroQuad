#pragma once

#include <VL53L1X.h>
#include <Wire.h>

#include <functional>

#include "MahonyAHRS.h"

class TelemetryController;

class VL53Manager
{
 public:
  VL53Manager();

  void begin(
      std::function<void(float)> altitudeHandler,
      TelemetryController *telemController,
      uint8_t address = 0x29,
      TwoWire *theWire = &Wire);

  void updatedAttitude(const EulerAngle &yawPitchRoll);

  void loopHandler(void);

 private:
  VL53L1X _vl53l1x;
  TelemetryController *_telemController;
  std::function<void(float)> _altitudeHandler;
  uint64_t _lastUpdateTimestampMillis;
  uint64_t _dataUpdateRateHz = 50;
  bool _isInitialized;
  EulerAngle _yawPitchRoll;
  bool _gotFirstIMUUpdate = false;
};