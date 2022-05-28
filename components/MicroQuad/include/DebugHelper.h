#ifndef DEBUGHELPER_H
#define DEBUGHELPER_H

#include <vector>
#include <string>

#define DEBUG_PACKET_SIZE 76
#define INITIAL_DATA_SIZE 1024 * 96

class DebugHelper
{
  public:
    DebugHelper();
    float ypr[3];
    float accelPr[2];
    float gyroYpr[3];
    float updates[3];
    float motorValues[4];
    float desiredValues[3];
    float inputTimescale;
    void saveValues(unsigned long timestamp);
    void printValues(unsigned long timestamp);

    bool hasMoreBytes(int currentIndex);
    uint8_t *getNextReading(int currentIndex, int byteLength);
    int64_t totalDataSize(void);
    uint8_t *data;

  private:
    uint64_t _currentSamples;
    int64_t _currentByteIndex;
    int64_t _currentDataSize;
    void _growHeapIfNeeded(void);
};

#endif
