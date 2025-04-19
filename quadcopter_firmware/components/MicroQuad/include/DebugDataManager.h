#pragma once

#include "esp_system.h"

#define DEBUG_PACKET_SIZE 200
#define INITIAL_DATA_SIZE 1024 * 96

class DebugDataManager
{
 public:
#ifndef MATLAB_SIM
  DebugDataManager();
  void addDouble(double *vals, uint64_t size);
#else   // MATLAB_SIM
  DebugDataManager() : numSamples(0), data(nullptr), currentSize(0) {}
  void addDouble(double *vals, uint64_t size) {}
#endif  // MATLAB_SIM

  uint64_t numSamples;
  uint8_t *data;
  uint64_t currentSize;

 private:
  uint64_t _head;
  volatile bool _reallocating;
  void _growHeapIfNeeded(void);
};
