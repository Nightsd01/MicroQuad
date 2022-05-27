#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

class File;

class DiagnosticsWriter 
{
    public:
        DiagnosticsWriter(const char *diagnosticsPath);
        void writeDiagnostics(String line);

    private:
        File _diagnosticsFile;
        unsigned long _lastFileWrite;
}

#endif