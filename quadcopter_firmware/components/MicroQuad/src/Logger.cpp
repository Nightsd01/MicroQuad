#include "Logger.h"

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <esp_log.h>

#include <string>

const char *TAG = "MicroQuad";

#define EMOJI_CONSOLE u8"\U0001F50D" /* Magnifying glass   */
#define EMOJI_VERBOSE u8"\U0001F4E3" /* Megaphone          */
#define EMOJI_BUG u8"\U0001F41E"     /* Bug */
#define EMOJI_INFO u8"\u2139\uFE0F"  /* Information        */
#define EMOJI_WARN u8"\u26A0\uFE0F"  /* Warning            */
#define EMOJI_ERROR u8"\u274C"       /* Cross mark        */

static void _log(char *statement, LogLevel level, unsigned long ts)
{
  switch (level) {
    case console: {
      ESP_LOGI(TAG, "[%s ] %s", (const char *)EMOJI_CONSOLE, statement);
      return;
    }
    case verbose: {
      ESP_LOGV(TAG, "[%s ] %s", (const char *)EMOJI_VERBOSE, statement);
      break;
    }
    case debug: {
      ESP_LOGD(TAG, "[%s ] %s", (const char *)EMOJI_BUG, statement);
      break;
    }
    case info: {
      ESP_LOGI(TAG, "[%s ] %s", (const char *)EMOJI_INFO, statement);
      break;
    }
    case warn: {
      ESP_LOGW(TAG, "[%s ] %s", (const char *)EMOJI_WARN, statement);
      break;
    }
    case error: {
      ESP_LOGE(TAG, "[%s ] %s", (const char *)EMOJI_ERROR, statement);
      break;
    }
    case none: {
      return;
    }
    case count: {
      ESP_LOGE(TAG, "Invalid log level");
      abort();
    }
  }
}

void _logImpl(LogLevel level, const char *format, va_list args)
{
#ifdef QUAD_LOG_LEVEL
  const LogLevel logLevel = stringToLogLevel(QUAD_LOG_LEVEL);
  if (level > logLevel) {
    return;
  }
#endif
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
#define __LOG(level)             \
  va_list args;                  \
  va_start(args, format);        \
  _logImpl(level, format, args); \
  va_end(args);

void LOG_ERROR(const char *format, ...) { __LOG(LogLevel::error); }

void LOG_WARN(const char *format, ...) { __LOG(LogLevel::warn); }

void LOG_INFO(const char *format, ...) { __LOG(LogLevel::info); }

void LOG_DEBUG(const char *format, ...) { __LOG(LogLevel::debug); }

void LOG_VERBOSE(const char *format, ...) { __LOG(LogLevel::verbose); }

void LOG_CONSOLE(const char *format, ...) { __LOG(LogLevel::console); }

#endif  // MATLAB_SIM