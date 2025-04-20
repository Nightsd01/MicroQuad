// AsyncController.h
#pragma once

#include <functional>
#include <mutex>  // For std::once_flag

// --- Standard C++ includes needed for Host version ---
#ifdef MATLAB_SIM
#include <atomic>
#include <chrono>
#include <future>  // For std::async
#include <thread>  // For std::this_thread::sleep_for
#endif             // MATLAB_SIM

// --- Portable Macros ---
#define _CONCATENATE_DETAIL(x, y) x##y
#define _CONCATENATE(x, y) _CONCATENATE_DETAIL(x, y)
#define _UNIQUE_VAR_NAME(base) _CONCATENATE(base, __COUNTER__)

// --- Conditional Time Function ---
#ifndef MATLAB_SIM
// Assumes millis() is available in the ESP32 environment (e.g., via Arduino.h or ESP-IDF time functions)
// If using plain ESP-IDF, you might need: #include "esp_timer.h" inline uint64_t millis() { return esp_timer_get_time()
// / 1000; }
#else
// Host implementation using std::chrono
inline uint64_t host_millis()
{
  // Use steady_clock for monotonic time suitable for measuring intervals
  auto now = std::chrono::steady_clock::now();
  // Calculate milliseconds since an arbitrary epoch (e.g., program start or clock's epoch)
  // Using time_since_epoch is fine for calculating differences, which is the typical use case.
  return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}
#define millis host_millis  // Map millis() call to host_millis() in SIM mode
#endif                      // MATLAB_SIM

// --- Conditional EXECUTE_PERIODIC Implementation Detail ---
// The implementation now uses the conditionally defined millis() or host_millis()
#define _EXECUTE_PERIODIC_IMPL(periodMillis, code, varName)      \
  do {                                                           \
    static uint64_t varName = 0;                                 \
    uint64_t currentTime = millis(); /* Uses correct function */ \
    if (currentTime - varName >= (periodMillis)) {               \
      varName = currentTime;                                     \
      code                                                       \
    }                                                            \
  } while (0)

#define EXECUTE_PERIODIC(periodMillis, code)                                       \
  do {                                                                             \
    _EXECUTE_PERIODIC_IMPL(periodMillis, code, _UNIQUE_VAR_NAME(previousLogTime)); \
  } while (0)

// --- Portable EXECUTE_ONCE Implementation (using std::call_once) ---
#define _EXECUTE_ONCE_IMPL(flagVarName, ...)  \
  do {                                        \
    static std::once_flag flagVarName;        \
    std::call_once(flagVarName, __VA_ARGS__); \
  } while (0)

#define EXECUTE_ONCE(...)                                         \
  do {                                                            \
    _EXECUTE_ONCE_IMPL(_UNIQUE_VAR_NAME(onceFlag_), __VA_ARGS__); \
  } while (0)

// --- Alias for the token used by executeOnce ---
#define AsyncControllerOnceToken std::once_flag

// --- AsyncController Class Definition (Interface remains the same) ---
class AsyncController
{
 public:
  // Static instances (distinction mainly for labeling in host mode)
  static AsyncController main;
  static AsyncController background;

  // Executes task. If allowSync=true and called from the 'correct' context (same core on ESP, always true on Host),
  // runs synchronously. Otherwise, runs asynchronously.
  void executePossiblySync(std::function<void()> task);

  // Always executes the task asynchronously.
  void executeAsync(std::function<void()> task);

  // Executes the task asynchronously after a delay.
  void executeAfter(uint64_t delayMillis, std::function<void()> task);

  // Guaranteed to execute only once globally for the given flag.
  // Execution follows executePossiblySync rules (sync if possible, else async).
  void executeOnce(AsyncControllerOnceToken &flag, std::function<void()> task);

 private:
  // Constructor is private to enforce singleton-like access via static members
  AsyncController(int sim_core_id_or_index, const char *taskLabel);  // Changed core type slightly for clarity

  // Internal execution logic (platform-specific implementation in .cpp)
  void _execute(std::function<void()> task, bool allowSync);

  // Member variables
  int _sim_core_id_or_index;  // On ESP: 0 or 1. On Host: 0 or 1 (index), but not used for affinity.
  const char *_taskLabel;
};