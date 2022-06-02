#include "diagnostics.h"

#include "Arduino.h"
#include "SD.h"

static bool _createDir(fs::FS &fs, const char * path){
    if(!fs.mkdir(path)){
        Serial.println("DIAGNOSTICS ERROR: mkdir failed");
        return false;
    }
    return true;
}

DiagnosticsWriter::DiagnosticsWriter(const char *diagnosticsDirPath, int sdCardCSPin) {
    if (SD.cardType() == CARD_NONE && !SD.begin(sdCardCSPin)) {
        Serial.println("DIAGNOSTICS ERROR: Failed to initialize SD card");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("DIAGNOSTICS ERROR: No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
        return;
    }

    File root = SD.open(diagnosticsDirPath);
    if (!root) {
        Serial.println("DIAGNOSTICS WARNING: No root diagnostics folder - creating one");
        delay(500);
        if (!_createDir(SD, diagnosticsDirPath)) {
            Serial.println("DIAGNOSTICS ERROR: Failed to create diagnostics directory");
            return;
        } else {
            root = SD.open(diagnosticsDirPath);
        }
    }

    File file = root.openNextFile();
    int diagnosticsCount = 0;
    while (file) {
        diagnosticsCount++;
        file = root.openNextFile();
    }

    String fullPath = String(diagnosticsDirPath) + "/diags_" + String(diagnosticsCount) + ".txt";
    Serial.println("RECORDING DIAGNOSTIC DATA TO " + fullPath);
    _diagnosticsFile = SD.open(fullPath, FILE_WRITE);
    if (!_diagnosticsFile) {
        Serial.println("DIAGNOSTICS ERROR: FAILED TO OPEN DIAGNOSTICS FILE AT " + fullPath);
        return;
    }
}

void DiagnosticsWriter::writeDiagnostics(String line)
{
    if (!_diagnosticsFile.print(": " + line + "\n")) {
        Serial.println("DIAGNOSTICS ERROR: Failed to write diagnostics data");
        return;
    }
    if (millis() - _lastFileWrite > 500 && xPortGetCoreID() == 1) {
        _diagnosticsFile.flush();
        _lastFileWrite = millis();
    }
}