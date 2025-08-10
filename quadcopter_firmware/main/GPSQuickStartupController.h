#pragma once

#ifndef MATLAB_SIM

#include <cstddef>
#include <cstdint>

class BLEController;
class GPSService;

class GPSQuickStartupController
{
 public:
  GPSQuickStartupController(BLEController* bleController, GPSService* gpsService);
  ~GPSQuickStartupController();

 private:
  BLEController* _bleController;
  GPSService* _gpsService;

  static double _readLittleEndianDouble(const uint8_t* bytes);
  void _registerL2CAPListener();
};

#endif  // MATLAB_SIM
