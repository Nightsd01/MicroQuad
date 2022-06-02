#include "Arduino.h"

// Setting the log level to none results in no console/SD card logs
// Console logs are only logged to console - not saved to disk
enum LogLevel { none, error, warn, info, debug, verbose, console };

/**
 * @brief The DebugUtility is intended to simplify/consolidate all debug log and diagnostics log logic
 * 
 * @param sdDebugLogDirectory - tells the logger which directory to save debug logs to
 * @param sdDiagnosticsDirectory - tells the logger which directory to save the diagnostics CSV to
 * @param sdDiagnosticsDirectory - the GPIO chip select (SPI) pin for the SD card
 * @param logLevel - sets the min log level for logs saved to disk, defaults to warn
 * @return * void 
 */
extern void DebugUtilityInitialize(
    const char *sdDebugLogDirectory, 
    const char *sdDiagnosticsDirectory, 
    int sdCsPin, 
    LogLevel logLevel = LogLevel::warn
);

extern void __logImpl_DoNotUse(String statement, LogLevel level);

#define LOG_ERROR(statement) __logImpl_DoNotUse(statement, LogLevel::error)
#define LOG_WARN(statement) __logImpl_DoNotUse(statement, LogLevel::warn)
#define LOG_INFO(statement) __logImpl_DoNotUse(statement, LogLevel::info)
#define LOG_DEBUG(statement) __logImpl_DoNotUse(statement, LogLevel::debug)
#define LOG_VERBOSE(statement) __logImpl_DoNotUse(statement, LogLevel::verbose)
#define LOG_CONSOLE(statement) __logImpl_DoNotUse(statement, LogLevel::console)

extern void __diagnosticsImpl_DoNotUse(const char* format, ...);

#define DIAGNOSTICS_SAVE(format, ...)  __diagnosticsImpl_DoNotUse(format, ##__VA_ARGS__)