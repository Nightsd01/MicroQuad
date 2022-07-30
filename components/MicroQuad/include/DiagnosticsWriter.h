#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"

class File;

class DiagnosticsWriter 
{
    public:
        DiagnosticsWriter(const char *diagnosticsDirPath, int sdCardCSPin);
        void writeDiagnostics(String line);
        void writeDiagnosticsAtTimestamp(unsigned long long time, String line);

    private:
        File _diagnosticsFile;
        unsigned long _lastFileWrite;
        unsigned long _uncommittedDataSize;
        void _checkIfShouldFlushSD(void);
};

#endif