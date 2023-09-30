#include "DebugHelper.h"

#include <stdio.h>
#include <memory>
#include <string>
#include <stdexcept>

#include <cstdio>
#include <cstring>
#include <cassert>

#include "Arduino.h"
#include "Logger.h"

template< typename... Args >
std::string string_sprintf( const char* format, Args... args ) {
  int length = std::snprintf( nullptr, 0, format, args... );
  assert( length >= 0 );

  char* buf = new char[length + 1];
  std::snprintf( buf, length + 1, format, args... );

  std::string str( buf );
  delete[] buf;
  return str;
}

static volatile bool reallocating = false;

unsigned long lastTime = 0;

DebugHelper::DebugHelper()
{
  data = (uint8_t *)malloc(INITIAL_DATA_SIZE); // reserve 16Kb
  if (data == NULL) {

  }
  _currentDataSize = INITIAL_DATA_SIZE;
  _currentByteIndex = 0;
  _currentSamples = 0;

}

void DebugHelper::printValues(unsigned long timestamp) {
  if (millis() - lastTime > 100) {
    lastTime = millis();
    // printf(
    //   "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %ld\n",
    //   ypr[0],
    //   ypr[1],
    //   ypr[2],
    //   desiredValues[0],
    //   desiredValues[1],
    //   desiredValues[2],
    //   inputTimescale,
    //   updates[0],
    //   updates[1],
    //   updates[2],
    //   motorValues[0],
    //   motorValues[1],
    //   motorValues[2],
    //   motorValues[3],
    //   timestamp
    // );
  }
}

static unsigned long lastPrintMillis = 0;
static int samples = 0;
void DebugHelper::saveValues(unsigned long timestamp) {
  while (reallocating) {}
  _growHeapIfNeeded();
  while (reallocating) {}
  float ts = (float)timestamp;
  memcpy(&data[_currentByteIndex], &ypr, sizeof(float) * 3);
  // memcpy(&data[_currentByteIndex + (3 * sizeof(float))], &gyroYpr, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (3 * sizeof(float))], &accelRaw, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (6 * sizeof(float))], &gyroRaw, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (9 * sizeof(float))], &desiredValues, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (12 * sizeof(float))], &updates, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (15 * sizeof(float))], &motorValues, sizeof(float) * 4);
  memcpy(&data[_currentByteIndex + (19 * sizeof(float))], &throttle, sizeof(float));
  memcpy(&data[_currentByteIndex + (20 * sizeof(float))], &ts, sizeof(float));
  memcpy(&data[_currentByteIndex + (21 * sizeof(float))], &yawPidValues, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (24 * sizeof(float))], &pitchPidValues, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (27 * sizeof(float))], &rollPidValues, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (30 * sizeof(float))], &voltage, sizeof(float));
  _currentByteIndex += DEBUG_PACKET_SIZE;
  _currentSamples++;
  samples++;
  if (millis() - lastPrintMillis > 200) {
    LOG_INFO("Recording at %ihz", samples * 5);
    samples = 0;
    lastPrintMillis = millis();
  } 
}

void DebugHelper::_growHeapIfNeeded(void) {
  if (_currentByteIndex + DEBUG_PACKET_SIZE >= _currentDataSize) {
    reallocating = true;
    Serial.println("Growing buffer with " + String((int)_currentSamples) + " and " + String((int)_currentByteIndex) + " from " + String((int)_currentDataSize) + " to " + String((int)(_currentDataSize + (1024 * 4))));
    uint64_t newDataSize = _currentDataSize + (1024 * 4);
    uint8_t *newData = (uint8_t *)realloc(data, newDataSize);
    if (newData == NULL) {
      Serial.println("Failed to grow data heap size");
    } else {
      Serial.println("Successfully grew data heap size");
      data = newData;
      _currentDataSize += 1024 * 4;
    }
    reallocating = false;
  }
}

int64_t DebugHelper::totalDataSize(void)
{
  return _currentByteIndex;
}
