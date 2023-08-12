#include "Logger.h"

#include <string>
#include <esp_log.h>
#include <Arduino.h>

const char *TAG = "MicroQuad";

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

void LOG_ERROR(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::error, format, args);
    va_end(args);
}

void LOG_WARN(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::warn, format, args);
    va_end(args);
}

void LOG_INFO(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::info, format, args);
    va_end(args);
}

void LOG_DEBUG(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::debug, format, args);
    va_end(args);
}

void LOG_VERBOSE(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::verbose, format, args);
    va_end(args);
}

void LOG_CONSOLE(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    _logImpl(LogLevel::console, format, args);
    va_end(args);
}
