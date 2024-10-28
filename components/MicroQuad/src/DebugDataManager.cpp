#include "DebugDataManager.h"

#include <cstring>

#include "Logger.h"

DebugDataManager::DebugDataManager()
{
  data = (uint8_t *)malloc(INITIAL_DATA_SIZE);
  if (data == NULL) {
    LOG_ERROR("Failed to allocate data for debug logging");
    return;
  }
  currentSize = INITIAL_DATA_SIZE;
  _head = 0;
}

void DebugDataManager::addDouble(double *vals, uint64_t size)
{
  while (_reallocating) {
  }
  _growHeapIfNeeded();
  while (_reallocating) {
  }
  memcpy(&data[_head + (size * sizeof(double))], vals, sizeof(double) * size);
  _head += size * sizeof(double);
}

void DebugDataManager::_growHeapIfNeeded(void)
{
  if (_head + DEBUG_PACKET_SIZE >= currentSize) {
    _reallocating = true;
    LOG_INFO(
        "Growing debug buffer with %i current samples, current head = %i, from "
        "size %i to %i",
        (int)numSamples,
        (int)_head,
        (int)currentSize,
        (int)(currentSize + (1024 * 4)));
    uint64_t newDataSize = currentSize + (1024 * 4);
    uint8_t *newData = (uint8_t *)realloc(data, newDataSize);
    if (newData == NULL) {
      LOG_ERROR("Failed to grow data heap size");
    } else {
      LOG_INFO("Successfully grew data heap size");
      data = newData;
      currentSize += 1024 * 4;
    }
    _reallocating = false;
  }
}