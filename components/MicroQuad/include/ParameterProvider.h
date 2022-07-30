#ifndef PARAMETER_PROVIDER_H
#define PARAMETER_PROVIDER_H

#include <string>
#include <functional>
#include <map>
#include <vector>

#include "Arduino.h"
#include "SD.h"

#include "Logger.h"

enum SupportedValueType {
    intParameter = 0,
    doubleParameter,
    stringParameter
};

template <typename T> SupportedValueType _valueType(T parameter);

// Updates the cached in-memory value for this parameter
template <typename T>
void _updateParameterInMemory(std::map<const char *, T> *valueMap, const char *name, T newValue) {
    if (valueMap->find(name) == valueMap->end()) {
        LOG_ERROR("Attempted to update an unregistered parameter (%s)", name);
        return;
    }
    valueMap->erase(name);
    valueMap->emplace(name, newValue);
    LOG_INFO("1 Adding param %s to map", name);
}

template <typename T>
void _loadParameterFromDiskString(
    std::map<const char *, T> *valueMap,
    const char *parameterName, 
    SupportedValueType type, 
    T value
) {
    if (valueMap->find(parameterName) == valueMap->end()) {
        valueMap->erase(parameterName);
        valueMap->emplace(parameterName, value);
        LOG_INFO("2 Adding param %s to map", parameterName);
    } else {
        _updateParameterInMemory(valueMap, parameterName, value);
    }
}

struct parameter_provider_disk_value_t {
    uint32_t beginIndex; // the character position/index in the whole parameters file
    SupportedValueType valueType;
};

/**
 * @brief The ParameterProvider is a disk-based parameter cache, which
 * allows us to provide "default" values in code but also update them
 * dynamically at runtime. Once a parameter is updated, it is cached 
 * to disk and the disk value will be used from then on.
 */

class ParameterProvider {
    public:
        template <typename T> T get(const char *name, T defaultValue) {
            std::map<const char *, T> *valueMap = _getValueMap<T>();
            LOG_INFO("Getting value for %s", name);
            if (valueMap->find(name) == valueMap->end()) {
                LOG_INFO("Value for %s not found in map", name);
                valueMap->emplace(name, defaultValue);
                return defaultValue;
            }
            LOG_INFO("Value for %s found in map", name);
            return (*valueMap)[name];
        }
        
        template <typename T> void watch(const char *name, std::function<void(T)> updateHandler) {
            std::map<const char *, T> *valueMap = _getValueMap<T>();
            if (valueMap->find(name) == valueMap->end()) {
                LOG_ERROR("Attempted to add a listener for changes to parameter %s before it was registered", name);
                return;
            }
            LOG_INFO("Watching parameter for name %s", name);
            std::map<const char *, std::vector<std::function<void(T)>>> *updateHandlers = _getUpdateHandlers<T>();
            if (updateHandlers->find(name) == updateHandlers->end()) {
                (*updateHandlers)[name] = { updateHandler };
            } else {
                (*updateHandlers)[name].push_back(updateHandler);
            }
        }

        template <typename T> void update(const char *name, T newValue) {
            std::map<const char *, T> *valueMap = _getValueMap<T>();
            LOG_INFO("Attempting to update parameter for name %s", name);
            if (valueMap->find(name) != valueMap->end() && (*valueMap)[name] == newValue) {
                // There's no need to update the value since it already was registered and has not changed
                LOG_INFO("Parameter for name %s has not changed - not updating", name);
                return;
            }
            LOG_INFO("Parameter for name %s found - updating", name);
            _updateParameterInMemory(valueMap, name, newValue);

            char *parameterStringValue = _createStringFromParameter(newValue);
            LOG_INFO("Updating parameter %s = %s", name, parameterStringValue);
            _updateParametersOnDisk();
            free(parameterStringValue);

            // tell any listeners that the value has changed
            std::map<const char *, std::vector<std::function<void(T)>>> *updateHandlers = _getUpdateHandlers<T>();
            if (updateHandlers->find(name) == updateHandlers->end()) {
                // no registered update handlers, it's fine to early return
                return;
            }
            for (auto & handler : (*updateHandlers)[name]) {
                handler(newValue);
            }
        }

        void begin(const char *sdPath);

    private:
        std::map<const char *, parameter_provider_disk_value_t> _diskValueMap;
        File _sdFile;
        bool _beganSuccess;
        const char *_filePath;
        void _updateParametersOnDisk(void);
        void _processParameterRowFromDisk(String row);

        std::map<const char *, int> _intMap;
        std::map<const char *, double> _doubleMap;
        std::map<const char *, std::string> _stringMap;

        std::map<const char *, std::vector<std::function<void(int)>>> _intUpdateHandlers;
        std::map<const char *, std::vector<std::function<void(double)>>> _doubleUpdateHandlers;
        std::map<const char *, std::vector<std::function<void(std::string)>>> _stringUpdateHandlers;

        template <typename T>
        std::map<const char *, T> *_getValueMap(void);

        template <typename T> 
        std::map<const char *, std::vector<std::function<void(T)>>> *_getUpdateHandlers(void);

        template <typename T>
        char *_createStringFromParameter(T parameter);


        template <class T> void _writeParamsToFile(
            std::map<const char *, T> *map, 
            File *file, 
            int *linesWritten
        ) {
            LOG_INFO("Writing params file with %i keys", (int)(map->size()));
            if (map->size() > 0) {
                for (const auto &keyPair : (*map)) {
                    LOG_INFO("Found %s in map", keyPair.first);
                    char *row = NULL;
                    char *paramStringValue = _createStringFromParameter(keyPair.second);
                    if (asprintf(&row, "%s,%i,%s", keyPair.first, (int)_valueType(keyPair.second), paramStringValue) == -1) {
                        LOG_ERROR("Failed to write parameter (%s) to disk - unable to allocate memory for the row", keyPair.first);
                        continue;
                    }
                    LOG_INFO("Writing int line to SD: %s", row);
                    file->println(row);
                    (*linesWritten)++;
                    free(row);
                    free(paramStringValue);
                }
            }
        }
};

#endif