#include "ParameterProvider.h"

#include "Arduino.h"
#include "SD.h"
#include "esp_log.h"

#define DEFAULT_READ_LINE_SIZE 100

template<> SupportedValueType _valueType(int parameter) { 
    return SupportedValueType::intParameter; 
}

template<> SupportedValueType _valueType(double parameter) { 
    return SupportedValueType::doubleParameter; 
}

template<> SupportedValueType _valueType(std::string parameter) { 
    return SupportedValueType::stringParameter; 
}

template<>
std::map<const char *, int> *ParameterProvider::_getValueMap(void) {
    return &_intMap;
}

template<>
std::map<const char *, double> *ParameterProvider::_getValueMap(void) {
    return &_doubleMap;
}

template<>
std::map<const char *, std::string> *ParameterProvider::_getValueMap(void) {
    return &_stringMap;
}

template<>
std::map<const char *, std::vector<std::function<void(int)>>> *ParameterProvider::_getUpdateHandlers(void) {
    return &_intUpdateHandlers;
}

template<>
std::map<const char *, std::vector<std::function<void(double)>>> *ParameterProvider::_getUpdateHandlers(void) {
    return &_doubleUpdateHandlers;
}

template<>
std::map<const char *, std::vector<std::function<void(std::string)>>> *ParameterProvider::_getUpdateHandlers(void) {
    return &_stringUpdateHandlers;
}

template<>
char *ParameterProvider::_createStringFromParameter(int parameter) {
    char *parameterStringValue = NULL;
    asprintf(&parameterStringValue, "%i", parameter);
    return parameterStringValue;
}

template<>
char *ParameterProvider::_createStringFromParameter(double parameter) {
    char *parameterStringValue = NULL;
    asprintf(&parameterStringValue, "%f", parameter);
    return parameterStringValue;
}

template<>
char *ParameterProvider::_createStringFromParameter(std::string parameter) {
    char *cString = (char *)malloc((parameter.length() + 1) * sizeof(char));
    strcpy(cString, parameter.c_str());
    return cString;
}

void ParameterProvider::_processParameterRowFromDisk(String row) {
    LOG_INFO("Processing disk row: %s", row.c_str());
    const int nameDelimiterIndex = row.indexOf(',', 0);
    String parameterName = row.substring(0, nameDelimiterIndex);

    const int parameterTypeIndex = row.indexOf(',', nameDelimiterIndex + 1);
    String parameterTypeString = row.substring(nameDelimiterIndex + 1, parameterTypeIndex);
    SupportedValueType parameterType = (SupportedValueType)parameterTypeString.toInt();

    const int valueDelimiterIndex = row.indexOf(',', parameterTypeIndex + 1);
    String valueString = row.substring(valueDelimiterIndex + 1);
    valueString.trim(); // remove any whitespace from beginning/end and remove the final \n char

    // we now have the parameter name, type, and value
    switch (parameterType) {
        case intParameter:
            _loadParameterFromDiskString(&_intMap, parameterName.c_str(), parameterType, (int)valueString.toInt());
            break;
        case doubleParameter:
            _loadParameterFromDiskString(&_doubleMap, parameterName.c_str(), parameterType, (double)valueString.toDouble());
            break;
        case stringParameter:
            _loadParameterFromDiskString<std::string>(&_stringMap, parameterName.c_str(), parameterType, valueString.c_str());
            break;
    }
}

void ParameterProvider::begin(const char *sdPath) {
    if (SD.exists(sdPath)) {
        _sdFile = SD.open(sdPath, "rw");
    } else {
        _sdFile = SD.open(sdPath, "w");
    }
    if (!_sdFile) {
        LOG_ERROR("Parameter provider failed to open file at path %s", sdPath);
        return;
    }
    _filePath = sdPath;
    _beganSuccess = true;
    while (_sdFile.available() > 1) {
        // Here is an example of a registered integer row in the file
        // {paramter name},{type enum},{value}
        // "param_name,0,123"
        LOG_INFO("Disk available");
        
        char *buf = (char *)malloc(sizeof(char) * DEFAULT_READ_LINE_SIZE);
        if (_sdFile.readBytesUntil('\n', buf, DEFAULT_READ_LINE_SIZE) > 0 
            && strcmp(buf, "\n") != 0 
            && strcmp(buf, "")) {
            LOG_INFO("Found disk row: %s", buf);
            _processParameterRowFromDisk(String(buf));
            free(buf);
            continue;
        }
        break;
    }

    // reset our position to the beginning of the file
    _sdFile.seek(0);
}

void ParameterProvider::_updateParametersOnDisk(void) {
    unsigned long long beginMillis = millis();
    if (!_beganSuccess) {
        LOG_ERROR("Attempted to update disk parameters with invalid parameter provider state");
        return;
    }
    LOG_INFO("Re-writing parameters file");
    // Removing & re-writing the entire file is highly inefficient, but should not be a frequent operation
    // Consider refactoring with disk B-tree if this ever becomes a bottleneck
    SD.remove(_filePath);

    File parameterFile = SD.open(_filePath, "w");
    if (!parameterFile) {
        LOG_ERROR("Attempted to update disk parameters but was unable to open the parameters file");
        return;
    }

    int numParams = 0;
    _writeParamsToFile(&_intMap, &parameterFile, &numParams);
    _writeParamsToFile(&_doubleMap, &parameterFile, &numParams);
    _writeParamsToFile(&_stringMap, &parameterFile, &numParams);

    parameterFile.flush();
    _sdFile = parameterFile;
    _sdFile.seek(0);
    LOG_INFO("Wrote %i parameters to disk in %llu milliseconds", numParams, millis() - beginMillis);
}
