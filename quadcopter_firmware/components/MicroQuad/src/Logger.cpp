#include "Logger.h"

#include <Arduino.h>
#include <esp_log.h>

#include <string>

const char *TAG = "MicroQuad";

enum LogLevel
{
  none,
  error,
  warn,
  info,
  debug,
  verbose,
  console
};

static void _log(char *statement, LogLevel level, unsigned long ts)
{
  switch (level) {
  case console: {
    ESP_LOGI(TAG, "%s", statement);
    return;
  }
  case verbose: {
    ESP_LOGV(TAG, "%s", statement);
    break;
  }
  case debug: {
    ESP_LOGD(TAG, "%s", statement);
    break;
  }
  case info: {
    ESP_LOGI(TAG, "%s", statement);
    break;
  }
  case warn: {
    ESP_LOGW(TAG, "%s", statement);
    break;
  }
  case error: {
    ESP_LOGE(TAG, "%s", statement);
    break;
  }
  case none: {
    return;
  }
  }
}

void _logImpl(LogLevel level, const char *format, va_list args)
{
  char *buffer = NULL;
  if (vasprintf(&buffer, format, args) == -1) {
    ESP_LOGE(TAG, "Unable to allocate memory for formatted log string");
    return;
  } else if (buffer == NULL) {
    ESP_LOGE(TAG, "Failed to format logging string");
    return;
  }
  _log(buffer, level, millis());
  free(buffer);
}

// technically it's not a good practice to depend on the caller's enclosing
// scope for variables like `format` but for the sake of simplicity it's worth
// it for avoiding repetition especially since this is just used internally
#define __LOG(level)                                                                                                   \
  va_list args;                                                                                                        \
  va_start(args, format);                                                                                              \
  _logImpl(level, format, args);                                                                                       \
  va_end(args);

void LOG_ERROR(const char *format, ...) { __LOG(LogLevel::error); }

void LOG_WARN(const char *format, ...) { __LOG(LogLevel::warn); }

void LOG_INFO(const char *format, ...) { __LOG(LogLevel::info); }

void LOG_DEBUG(const char *format, ...) { __LOG(LogLevel::debug); }

void LOG_VERBOSE(const char *format, ...) { __LOG(LogLevel::verbose); }

void LOG_CONSOLE(const char *format, ...) { __LOG(LogLevel::console); }
