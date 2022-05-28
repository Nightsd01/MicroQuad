#include "DebugHelper.h"

#include <stdio.h>
#include <memory>
#include <string>
#include <stdexcept>

#include <cstdio>
#include <cstring>
#include <cassert>

#include "Arduino.h"

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
    printf(
      "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %ld\n",
      ypr[0],
      ypr[1],
      ypr[2],
      desiredValues[0],
      desiredValues[1],
      desiredValues[2],
      inputTimescale,
      updates[0],
      updates[1],
      updates[2],
      motorValues[0],
      motorValues[1],
      motorValues[2],
      motorValues[3],
      timestamp
    );
  }
}

void DebugHelper::saveValues(unsigned long timestamp) {
  // std::string column = string_sprintf(
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

  // _statements.push_back(column);

  while (reallocating) {}
  _growHeapIfNeeded();
  while (reallocating) {}
  float ts = (float)timestamp;
  memcpy(&data[_currentByteIndex], &ypr, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (3 * sizeof(float))], &gyroYpr, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (6 * sizeof(float))], &accelPr, sizeof(float) * 2);
  memcpy(&data[_currentByteIndex + (8 * sizeof(float))], &desiredValues, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (11 * sizeof(float))], &updates, sizeof(float) * 3);
  memcpy(&data[_currentByteIndex + (14 * sizeof(float))], &motorValues, sizeof(float) * 4);
  memcpy(&data[_currentByteIndex + (18 * sizeof(float))], &ts, sizeof(float));
  _currentByteIndex += DEBUG_PACKET_SIZE;
  _currentSamples++;
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
