#include "diagnostics.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Arduino.h"

#define SD_CS_PIN 12

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("DIAGNOSTICS ERROR: Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("DIAGNOSTICS ERROR: Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

static bool _createDir(fs::FS &fs, const char * path){
    if(!fs.mkdir(path)){
        Serial.println("DIAGNOSTICS ERROR: mkdir failed");
        return false;
    }
    return true;
}

void removeDir(fs::FS &fs, const char * path){
    if(!fs.rmdir(path)){
        Serial.println("DIAGNOSTICS ERROR: rmdir failed");
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("DIAGNOSTICS ERROR: Failed to open file for writing");
        return;
    }
    if(!file.print(message)){
        Serial.println("DIAGNOSTICS ERROR: Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("DIAGNOSTICS ERROR: Failed to open file for appending");
        return;
    }
    if(file.print(message)){
    } else {
        Serial.println("DIAGNOSTICS ERROR: Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    if (!fs.rename(path1, path2)) {
        Serial.println("DIAGNOSTICS ERROR: Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    if(!fs.remove(path)){
        Serial.println("DIAGNOSTICS ERROR: Delete failed");
    }
}

DiagnosticsWriter::DiagnosticsWriter(const char *diagnosticsPath) {
    if (!SD.begin(SD_CS_PIN)) {
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

    File root = SD.open(diagnosticsPath);
    if (!root) {
        Serial.println("DIAGNOSTICS WARNING: No root diagnostics folder - creating one");
        if (!_createDir(SD, diagnosticsPath)) {
            Serial.println("DIAGNOSTICS ERROR: Failed to create diagnostics directory");
            return;
        } else {
            root = SD.open(diagnosticsPath);
        }
    }

    File file = root.openNextFile();
    int diagnosticsCount = 0;
    while (file) {
        diagnosticsCount++;
        file = root.openNextFile();
    }

    String fullPath = String(diagnosticsPath) + "/diags_" + String(diagnosticsCount) + ".txt";
    _diagnosticsFile = SD.open(fullPath, FILE_WRITE);
    if (!_diagnosticsFile) {
        Serial.println("DIAGNOSTICS ERROR: FAILED TO OPEN DIAGNOSTICS FILE AT " + fullPath);
        return;
    }
}

void DiagnosticsWriter::writeDiagnostics(String line)
{
    unsigned long time1 = micros();
    if (!_diagnosticsFile.print(line)) {
        Serial.println("DIAGNOSTICS ERROR: Failed to write diagnostics data");
        return;
    }
    Serial.println("Wrote diags in %lu milliseconds", millis() - time1);
}