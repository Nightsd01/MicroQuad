#pragma once

#include <AsyncController.h>

// TODO: Reduce duplication for each log level
// Ideally we can get to a point where this file is only
// 6 lines long (one macro for each level)

void LOG_ERROR(const char *format, ...);
void LOG_WARN(const char *format, ...);
void LOG_INFO(const char *format, ...);
void LOG_DEBUG(const char *format, ...);
void LOG_VERBOSE(const char *format, ...);
void LOG_CONSOLE(const char *format, ...);

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)

// Macro to generate a unique variable name
#define UNIQUE_VAR_NAME(base) CONCATENATE(base, __COUNTER__)

#define __LOG_PERIODIC_MILLIS_IMPL(level, period, varName, format, ...) \
  do {                                                                  \
    static uint64_t varName = 0;                                        \
    if (millis() - varName > (period)) {                                \
      varName = millis();                                               \
      LOG_##level(format, ##__VA_ARGS__);                               \
    }                                                                   \
  } while (0)

#define __LOG_PERIODIC_MILLIS(level, period, format, ...)                                               \
  do {                                                                                                  \
    __LOG_PERIODIC_MILLIS_IMPL(level, period, UNIQUE_VAR_NAME(previousLogTime), format, ##__VA_ARGS__); \
  } while (0)

#define __LOG(level, format, ...)                                                             \
  do {                                                                                        \
    AsyncController::main.executePossiblySync([=]() { LOG_##level(format, ##__VA_ARGS__); }); \
  } while (0)

#define LOG_ERROR_ASYNC_ON_MAIN(format, ...) __LOG(ERROR, format, ##__VA_ARGS__)
#define LOG_WARN_ASYNC_ON_MAIN(format, ...) __LOG(WARN, format, ##__VA_ARGS__)
#define LOG_INFO_ASYNC_ON_MAIN(format, ...) __LOG(INFO, format, ##__VA_ARGS__)
#define LOG_DEBUG_ASYNC_ON_MAIN(format, ...) __LOG(DEBUG, format, ##__VA_ARGS__)
#define LOG_VERBOSE_ASYNC_ON_MAIN(format, ...) __LOG(VERBOSE, format, ##__VA_ARGS__)
#define LOG_CONSOLE_ASYNC_ON_MAIN(format, ...) __LOG(CONSOLE, format, ##__VA_ARGS__)

#define LOG_ERROR_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(ERROR, intervalMillis, format, ##__VA_ARGS__)
#define LOG_WARN_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(WARN, intervalMillis, format, ##__VA_ARGS__)
#define LOG_INFO_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(INFO, intervalMillis, format, ##__VA_ARGS__)
#define LOG_DEBUG_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(DEBUG, intervalMillis, format, ##__VA_ARGS__)
#define LOG_VERBOSE_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(VERBOSE, intervalMillis, format, ##__VA_ARGS__)
#define LOG_CONSOLE_PERIODIC_MILLIS(intervalMillis, format, ...) \
  __LOG_PERIODIC_MILLIS(CONSOLE, intervalMillis, format, ##__VA_ARGS__)
