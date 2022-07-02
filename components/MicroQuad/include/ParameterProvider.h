#ifndef PARAMETER_PROVIDER_H
#define PARAMETER_PROVIDER_H

#include <string>
#include <map>
#include <vector>

#include "Arduino.h"
#include "SD.h"

enum SupportedValueType {
    intParameter = 0,
    doubleParameter,
    stringParameter
};

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
        // NOTE: Parameter names must not contain a comma (,)
        void registerParameter(const char *name, int defaultValue);
        void registerParameter(const char *name, double defaultValue);
        void registerParameter(const char *name, std::string defaultValue);

        void updateInt(const char *name, int newValue);
        void updateDouble(const char *name, double newValue);
        void updateString(const char *name, std::string newValue);

        void begin(const char *sdPath);

        int getInt(const char *name);
        double getDouble(const char *name);
        std::string getString(const char *name);

    private:
        std::map<const char *, int> _intMap;
        std::map<const char *, double> _doubleMap;
        std::map<const char *, std::string> _stringMap;
        std::map<const char *, parameter_provider_disk_value_t> _diskValueMap;
        File _sdFile;
        bool _beganSuccess;
        const char *_filePath;
        void _updateParameterOnDisk(const char *name, const char *stringValue, SupportedValueType type);
        void _processParameterRowFromDisk(String row);
};

#endif