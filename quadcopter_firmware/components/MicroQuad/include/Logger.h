#pragma once

#include <AsyncController.h>

void LOG_ERROR(const char *format, ...);
void LOG_WARN(const char *format, ...);
void LOG_INFO(const char *format, ...);
void LOG_DEBUG(const char *format, ...);
void LOG_VERBOSE(const char *format, ...);
void LOG_CONSOLE(const char *format, ...);

#define LOG_ERROR_ASYNC_ON_MAIN(format, ...)                                                                           \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_ERROR(format, ##__VA_ARGS__); });                                        \
  } while (0)

#define LOG_WARN_ASYNC_ON_MAIN(format, ...)                                                                            \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_WARN(format, ##__VA_ARGS__); });                                         \
  } while (0)

#define LOG_INFO_ASYNC_ON_MAIN(format, ...)                                                                            \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_INFO(format, ##__VA_ARGS__); });                                         \
  } while (0)

#define LOG_DEBUG_ASYNC_ON_MAIN(format, ...)                                                                           \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_DEBUG(format, ##__VA_ARGS__); });                                        \
  } while (0)

#define LOG_VERBOSE_ASYNC_ON_MAIN(format, ...)                                                                         \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_VERBOSE(format, ##__VA_ARGS__); });                                      \
  } while (0)

#define LOG_CONSOLE_ASYNC_ON_MAIN(format, ...)                                                                         \
  do {                                                                                                                 \
    AsyncController::main.execute([=]() { LOG_CONSOLE(format, ##__VA_ARGS__); });                                      \
  } while (0)