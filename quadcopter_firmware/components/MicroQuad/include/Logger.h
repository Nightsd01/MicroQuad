#ifndef MATLAB_SIM

#pragma once

#include <AsyncController.h>
#include <string.h>

// This is where all the supported log levels are defined
// Uses a macro to avoid duplication
#define LOG_LEVELS(X) \
  X(none)             \
  X(error)            \
  X(warn)             \
  X(info)             \
  X(debug)            \
  X(verbose)          \
  X(console)

enum LogLevel
{
#define X(name) name,
  LOG_LEVELS(X)
#undef X
      count
};

// String to enum conversion
constexpr LogLevel stringToLogLevel(const char *str)
{
#define X(name) \
  if (strcmp(str, #name) == 0) return LogLevel::name;
  LOG_LEVELS(X)
#undef X
  return LogLevel::none;  // Default if no match
}

// TODO: Reduce duplication for each log level
// Ideally we can get to a point where this file is only
// 6 lines long (one macro for each level)

void LOG_ERROR(const char *format, ...);
void LOG_WARN(const char *format, ...);
void LOG_INFO(const char *format, ...);
void LOG_DEBUG(const char *format, ...);
void LOG_VERBOSE(const char *format, ...);
void LOG_CONSOLE(const char *format, ...);

#define __LOG(level, format, ...)                                                             \
  do {                                                                                        \
    AsyncController::main.executePossiblySync([=]() { LOG_##level(format, ##__VA_ARGS__); }); \
  } while (0);

#define __LOG_PERIODIC_MILLIS(level, periodMillis, format, ...)              \
  do {                                                                       \
    EXECUTE_PERIODIC(periodMillis, { LOG_##level(format, ##__VA_ARGS__); }); \
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

#else  // MATLAB_SIM

#define LOG_ERROR(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_VERBOSE(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_CONSOLE(format, ...) printf(format, ##__VA_ARGS__)

#define LOG_ERROR_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_WARN_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_INFO_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_DEBUG_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_VERBOSE_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)
#define LOG_CONSOLE_ASYNC_ON_MAIN(format, ...) printf(format, ##__VA_ARGS__)

#define LOG_ERROR_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)
#define LOG_WARN_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)
#define LOG_INFO_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)
#define LOG_DEBUG_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)
#define LOG_VERBOSE_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)
#define LOG_CONSOLE_PERIODIC_MILLIS(intervalMillis, format, ...) printf(format, ##__VA_ARGS__)

#endif  // MATLAB_SIM