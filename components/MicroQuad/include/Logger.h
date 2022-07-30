#ifndef LOGGER_H
#define LOGGER_H

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

extern void LOG_ERROR(const char *format, ...);
extern void LOG_WARN(const char *format, ...);
extern void LOG_INFO(const char *format, ...);
extern void LOG_DEBUG(const char *format, ...);
extern void LOG_VERBOSE(const char *format, ...);
extern void LOG_CONSOLE(const char *format, ...);

extern void DIAGNOSTICS_SAVE(const char* format, ...);

#endif