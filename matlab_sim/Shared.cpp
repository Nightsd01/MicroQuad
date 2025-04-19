#include "Shared.h"

#include <ctime>
#include <chrono>

static bool _useFakeData = false;
static uint64_t _fakeMicros = 0;

void setFakeTime(uint64_t time)
{
  _useFakeData = true;
  _fakeMicros = time;
}

uint64_t millis()
{
  if (_useFakeData)
  {
    return _fakeMicros / 1000;
  }
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

uint64_t micros()
{
  if (_useFakeData)
  {
    return _fakeMicros;
  }
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}