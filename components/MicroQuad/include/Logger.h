#pragma once

enum LogLevel { none, error, warn, info, debug, verbose, console };

extern void LOG_ERROR(const char *format, ...);
extern void LOG_WARN(const char *format, ...);
extern void LOG_INFO(const char *format, ...);
extern void LOG_DEBUG(const char *format, ...);
extern void LOG_VERBOSE(const char *format, ...);
extern void LOG_CONSOLE(const char *format, ...);