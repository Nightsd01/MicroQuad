#ifndef DEBUG_DATA_MANAGER_H
#define DEBUG_DATA_MANAGER_H

#include "esp_system.h"

#define DEBUG_PACKET_SIZE 200
#define INITIAL_DATA_SIZE 1024 * 96

class DebugDataManager
{
 public:
  DebugDataManager();
  void addDouble(double *vals, uint64_t size);
  uint64_t numSamples;
  uint8_t *data;
  uint64_t currentSize;

 private:
  uint64_t _head;
  volatile bool _reallocating;
  void _growHeapIfNeeded(void);
};

#endif