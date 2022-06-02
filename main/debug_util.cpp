#include "debug_util.h"

#include "diagnostics.h"
#include "esp_log.h"

#define MAIN_CORE_ID 1

static DiagnosticsWriter *_debugLogWriter = NULL;
static DiagnosticsWriter *_diagnosticsCsvWriter = NULL;
static LogLevel _logLevel = LogLevel::warn;
static bool _initialized = false;
const char *TAG = "MicroQuad";

typedef struct log_task_t {
    char *logCopy;
    LogLevel level;
} log_task_t;

static void _mainThreadLog(void *params) {
    log_task_t *task = (log_task_t *)params;
    // now that we have hopped back to the main core, log again
    __logImpl_DoNotUse(task->logCopy, task->level);
    free(task->logCopy);
    free(task);
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
    if (sdDebugLogDirectory != NULL) {
        _debugLogWriter = new DiagnosticsWriter(sdDebugLogDirectory, sdCsPin);
    }
    if (sdDiagnosticsDirectory != NULL) {
        _diagnosticsCsvWriter = new DiagnosticsWriter(sdDiagnosticsDirectory, sdCsPin);
    }
}

void __logImpl_DoNotUse(String statement, LogLevel level)
{
    // If logging from a different thread, hop over to the main thread for logging
    if (xPortGetCoreID() != MAIN_CORE_ID) {
        log_task_t *task = (log_task_t *)calloc(1, sizeof(struct log_task_t));
        char *strCpy = (char *)malloc(sizeof(char) * strlen(statement.c_str()));
        strcpy(strCpy, statement.c_str());
        task->logCopy = strCpy;
        task->level = level;
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
            
    if (!_initialized) {
        ESP_LOGE(TAG, "Attempted to log before initializing the debug utility");
        return;
    }
    
    if (level > _logLevel) {
        return;
    }

    String errorLevelString;
    switch (level) {
        case console: {
            ESP_LOGI(TAG, "%s", statement.c_str());
            return;
        }
        case verbose: {
            errorLevelString = "VERBOSE";
            ESP_LOGV(TAG, "%s", statement.c_str());
            break;
        }
        case debug: {
            errorLevelString = "DEBUG";
            ESP_LOGD(TAG, "%s", statement.c_str());
            break;
        }
        case info: {
            errorLevelString = "INFO";
            ESP_LOGI(TAG, "%s", statement.c_str());
            break;
        }
        case warn: {
            errorLevelString = "WARN";
            ESP_LOGW(TAG, "%s", statement.c_str());
            break;
        }
        case error: {
            errorLevelString = "ERROR";
            ESP_LOGE(TAG, "%s", statement.c_str());
            break;
        }
        case none: {
            return;
        }
    }

    String line = String(millis()) + "(" + errorLevelString + "): " + statement;
    _debugLogWriter->writeDiagnostics(line);
}

void __diagnosticsImpl_DoNotUse(const char* format, ...)
{
    va_list argptr;
    va_start(argptr,format);
    char *buffer = nullptr;
    asprintf(&buffer, format, argptr);
    va_end(argptr);
    String result = String(millis()) + "," + String(buffer);
    _diagnosticsCsvWriter->writeDiagnostics(result);
    free(buffer);
}
