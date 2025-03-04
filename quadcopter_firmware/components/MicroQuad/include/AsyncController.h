#ifndef ASYNC_CONTROLLER_H
#define ASYNC_CONTROLLER_H

#include <Arduino.h>

#include <functional>
#include <mutex>

#define _CONCATENATE_DETAIL(x, y) x##y
#define _CONCATENATE(x, y) _CONCATENATE_DETAIL(x, y)

// Macro to generate a unique variable name
#define _UNIQUE_VAR_NAME(base) _CONCATENATE(base, __COUNTER__)

#define _EXECUTE_PERIODIC_IMPL(periodMillis, code, varName) \
  do {                                                      \
    static uint64_t varName = 0;                            \
    if (millis() - varName > (periodMillis)) {              \
      varName = millis();                                   \
      code                                                  \
    }                                                       \
  } while (0)

// Execute the given code at most once every periodMillis
// This isn't a timer, but it lets you limit how often code can execute
#define EXECUTE_PERIODIC(periodMillis, code)                                       \
  do {                                                                             \
    _EXECUTE_PERIODIC_IMPL(periodMillis, code, _UNIQUE_VAR_NAME(previousLogTime)); \
  } while (0)

#define AsyncControllerOnceToken std::once_flag

class AsyncController
{
 public:
  // Static member variables
  static AsyncController main;
  static AsyncController background;
  void executePossiblySync(std::function<void()> task);
  void executeAsync(std::function<void()> task);
  void executeAfter(uint64_t delayMillis, std::function<void()> task);

  // Guaranteed to execute only once per the given flag
  // If you are currently running on main and try to call this
  // from bg (or visa versa), this will be an async call
  // Otherwise it will be synchronous
  void executeOnce(AsyncControllerOnceToken &flag, std::function<void()> task);

 private:
  AsyncController(bool core, const char *taskLabel);
  void _execute(std::function<void()> task, bool allowSync);
  bool _core;
  const char *_taskLabel;
};

#endif