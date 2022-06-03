#include "Logger.h"

#include <vector>

#include "DiagnosticsWriter.h"
#include "esp_log.h"

#define MAIN_CORE_ID 1

typedef struct log_task_t {
    char *logCopy;
    LogLevel level;
    unsigned long long timestamp;
} log_task_t;

static DiagnosticsWriter *_debugLogWriter = NULL;
static DiagnosticsWriter *_diagnosticsCsvWriter = NULL;
static LogLevel _logLevel = LogLevel::warn;
static bool _initialized = false;
const char *TAG = "MicroQuad";
static std::vector<log_task_t *> _preInitializationLogs;

static void _log(String statement, LogLevel level, unsigned long ts) 
{
    if (level > _logLevel) {
        return;
    }

    String line;
    switch (level) {
        case console: {
            line = String(ts) + " (CONSOLE): " + statement;
            ESP_LOGI(TAG, "%s", line.c_str());
            return;
        }
        case verbose: {
            line = String(ts) + " (VERBOSE): " + statement;
            ESP_LOGV(TAG, "%s", line.c_str());
            break;
        }
        case debug: {
            line = String(ts) + " (DEBUG): " + statement;
            ESP_LOGD(TAG, "%s", line.c_str());
            break;
        }
        case info: {
            line = String(ts) + " (INFO): " + statement;
            ESP_LOGI(TAG, "%s", line.c_str());
            break;
        }
        case warn: {
            line = String(ts) + " (WARN): " + statement;
            ESP_LOGW(TAG, "%s", line.c_str());
            break;
        }
        case error: {
            line = String(ts) + " (ERROR): " + statement;
            ESP_LOGE(TAG, "%s", line.c_str());
            break;
        }
        case none: {
            return;
        }
    }

    _debugLogWriter->writeDiagnostics(line);
}

static void _logAndFreeTask(log_task_t *task)
{
    _log(String(task->logCopy), task->level, task->timestamp);
    free(task->logCopy);
    free(task);
}

static void _mainThreadLog(void *params) {
    log_task_t *task = (log_task_t *)params;
    // now that we have hopped back to the main core, log again
    _logAndFreeTask(task);
    vTaskDelete(NULL);
}

void DebugUtilityInitialize(
    const char *sdDebugLogDirectory, 
    const char *sdDiagnosticsDirectory, 
    int sdCsPin, 
    LogLevel logLevel
)
{
    if (_initialized) {
        ESP_LOGE(TAG, "Already initialized debug utility");
        return;
    }
    _initialized = true;
    _logLevel = logLevel;
    if (sdDiagnosticsDirectory != NULL) {
        _diagnosticsCsvWriter = new DiagnosticsWriter(sdDiagnosticsDirectory, sdCsPin);
    }
    if (sdDebugLogDirectory != NULL) {
        _debugLogWriter = new DiagnosticsWriter(sdDebugLogDirectory, sdCsPin);
    }

    for (int i = 0; i < _preInitializationLogs.size(); i++) {
        _logAndFreeTask(_preInitializationLogs[i]);
    }
    _preInitializationLogs.clear();
}

static log_task_t *_allocateLogTaskWithMessageCopy(String statement, LogLevel level)
{
    log_task_t *task = (log_task_t *)calloc(1, sizeof(struct log_task_t));
    char *strCpy = (char *)malloc(sizeof(char) * strlen(statement.c_str()));
    if (task == NULL || strCpy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate logging memory");
        return nullptr;
    }
    strcpy(strCpy, statement.c_str());
    task->logCopy = strCpy;
    task->level = level;
    task->timestamp = millis();
    return task;
}

void __logImpl_DoNotUse(String statement, LogLevel level)
{        
    if (!_initialized) {
        // Add the log information to _preInitializationLogs
        log_task_t *task = _allocateLogTaskWithMessageCopy(statement, level);
        if (task == nullptr) {
            return;
        }
        _preInitializationLogs.push_back(task);
        return;
    }

    // If logging from a different thread, hop over to the main thread for logging
    if (xPortGetCoreID() != MAIN_CORE_ID) {
        log_task_t *task = _allocateLogTaskWithMessageCopy(statement, level);
        if (task == nullptr) {
            return;
        }
        xTaskCreatePinnedToCore(
            _mainThreadLog,      /* Function to implement the task */
            "logTask",           /* Name of the task */
            4096,                /* Stack size in words */
            (void *)task,        /* Task input parameter */
            1,                   /* Priority of the task */
            NULL,                /* Task handle. */
            MAIN_CORE_ID         /* Core selection parameter */
        );
        return;
    }
    
    _log(statement, level, millis());
}

void __diagnosticsImpl_DoNotUse(const char* format, ...)
{
    if (_diagnosticsCsvWriter == NULL) {
        return;
    }
    va_list argptr;
    va_start(argptr,format);
    char *buffer = nullptr;
    if (asprintf(&buffer, format, argptr) == -1) {
        ESP_LOGE(TAG, "Unable to allocate memory for formatted log string");
        va_end(argptr);
        return;
    }
    va_end(argptr);
    String result = String(buffer);
    _diagnosticsCsvWriter->writeDiagnostics(result);
    free(buffer);
}
