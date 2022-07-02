#include "ParameterProvider.h"

#include "Arduino.h"
#include "SD.h"
#include "esp_log.h"

#include "Logger.h"

static bool _createDir(fs::FS &fs, const char * path){
    if(!fs.mkdir(path)){
        LOG_ERROR("Could not create parameters file with name: %s", path);
        return false;
    }
    return true;
}

void ParameterProvider::registerParameter(const char *name, int defaultValue) {
    _intMap.emplace(name, defaultValue);
}

void ParameterProvider::registerParameter(const char *name, double defaultValue) {
    _doubleMap.emplace(name, defaultValue);
}

void ParameterProvider::registerParameter(const char *name, std::string defaultValue) {
    _stringMap.emplace(name, defaultValue);
}

// Updates the cached in-memory value for this parameter
template <class T>
static void _updateParameterInMemory(std::map<const char *, T> *valueMap, const char *name, T newValue) {
    if (valueMap->find(name) == valueMap->end()) {
        LOG_ERROR("Attempted to update an unregistered parameter (%s)", name);
        return;
    }
    valueMap->erase(name);
    valueMap->emplace(name, newValue);
}

void ParameterProvider::updateInt(const char *name, int newValue)
{
    _updateParameterInMemory(&_intMap, name, newValue);
    char *parameterStringValue = NULL;
    asprintf(&parameterStringValue, "%i", newValue);
    _updateParameterOnDisk(name, parameterStringValue, SupportedValueType::intParameter);
    free(parameterStringValue);
}

void ParameterProvider::updateDouble(const char *name, double newValue)
{
    _updateParameterInMemory(&_doubleMap, name, newValue);
    char *parameterStringValue = NULL;
    asprintf(&parameterStringValue, "%f", newValue);
    _updateParameterOnDisk(name, parameterStringValue, SupportedValueType::doubleParameter);
    free(parameterStringValue);
}

void ParameterProvider::updateString(const char *name, std::string newValue)
{
    _updateParameterInMemory(&_stringMap, name, newValue);
    _updateParameterOnDisk(name, newValue.c_str(), SupportedValueType::intParameter);
}

template <class T>
static void _loadParameterFromDiskString(std::map<const char *, T> *valueMap, const char *parameterName, SupportedValueType type, T value) {
    if (valueMap->find(parameterName) == valueMap->end()) {
        valueMap->erase(parameterName);
        valueMap->emplace(parameterName, value);
    } else {
        _updateParameterInMemory(valueMap, parameterName, value);
    }
}

void ParameterProvider::_processParameterRowFromDisk(String row) {
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
    _sdFile = SD.open(sdPath, "rw");
    if (!_sdFile) {
        LOG_ERROR("Parameter provider failed to open file at path %s", sdPath);
        return;
    }
    _filePath = sdPath;
    _beganSuccess = true;
    while (_sdFile.available()) {
        // Here is an example of a registered integer row in the file
        // {paramter name}, {type enum}, {value}
        // "param_name,0,123"
        String row = _sdFile.readStringUntil('\n');
        _processParameterRowFromDisk(row);
    }

    // reset our position to the beginning of the file
    _sdFile.seek(0);
}

int ParameterProvider::getInt(const char *name) {
    return _intMap[name];
}

double ParameterProvider::getDouble(const char *name) {
    return _doubleMap[name];
}

std::string ParameterProvider::getString(const char *name) {
    return _stringMap[name];
}

void ParameterProvider::_updateParameterOnDisk(
    const char *name, 
    const char *stringValue,
    SupportedValueType type
) {
    if (!_beganSuccess) {
        LOG_ERROR("Attempted to update parameter (%s) with invalid parameter provider state", name);
        return;
    }
    File parameterFile = SD.open(_filePath);
    if (!parameterFile) {
        LOG_ERROR("Attempted to update parameter (%s) but was unable to open the parameters file", name);
        return;
    }
}
