// #include "DiagnosticsWriter.h"

// #include "Arduino.h"
// // #include "SD.h"

// #include "esp_log.h"

// const char *diagnosticsTag = "diagnostics";

// #define MAX_SD_BUFFER_BYTE_SIZE 512

// static bool _createDir(fs::FS &fs, const char * path){
//     // if(!fs.mkdir(path)){
//     //     ESP_LOGE(diagnosticsTag, "mkdir failed");
//     //     return false;
//     // }
//     return true;
// }

// DiagnosticsWriter::DiagnosticsWriter(const char *diagnosticsDirPath, int sdCardCSPin) {
//     // if (SD.cardType() == CARD_NONE && !SD.begin(sdCardCSPin)) {
//     //     ESP_LOGE(diagnosticsTag, "Failed to initialize SD card");
//     //     return;
//     // }
//     // uint8_t cardType = SD.cardType();

//     // if(cardType == CARD_NONE){
//     //     ESP_LOGE(diagnosticsTag, "No SD card attached");
//     //     return;
//     // }

//     // if (cardType == CARD_MMC){
//     //     ESP_LOGI(diagnosticsTag, "SD Card Type: MMC");
//     // } else if (cardType == CARD_SD){
//     //     ESP_LOGI(diagnosticsTag, "SD Card Type: SDSC");
//     // } else if (cardType == CARD_SDHC){
//     //     ESP_LOGI(diagnosticsTag, "SD Card Type: SDHC");
//     // } else {
//     //     ESP_LOGI(diagnosticsTag, "SD Card Type: UNKNOWN");
//     //     return;
//     // }

//     // ESP_LOGI(diagnosticsTag, "Attempting to open diagnostics file at path %s", diagnosticsDirPath);
//     // File root = SD.open(diagnosticsDirPath);
//     // if (!root) {
//     //     ESP_LOGI(diagnosticsTag, "No root diagnostics folder - creating one");
//     //     delay(500);
//     //     if (!_createDir(SD, diagnosticsDirPath)) {
//     //         ESP_LOGE(diagnosticsTag, "Failed to create diagnostics directory");
//     //         return;
//     //     } else {
//     //         root = SD.open(diagnosticsDirPath);
//     //     }
//     // }

//     // File file = root.openNextFile();
//     // int diagnosticsCount = 0;
//     // while (file) {
//     //     diagnosticsCount++;
//     //     file = root.openNextFile();
//     // }

//     // String fullPath = String(diagnosticsDirPath) + "/" + String(diagnosticsCount) + ".txt";
//     // ESP_LOGI(diagnosticsTag, "%s", ("RECORDING DIAGNOSTIC DATA TO " + fullPath).c_str());
//     // _diagnosticsFile = SD.open(fullPath, FILE_WRITE);
//     // if (!_diagnosticsFile) {
//     //     ESP_LOGE(diagnosticsTag, "%s", ("FAILED TO OPEN DIAGNOSTICS FILE AT " + fullPath).c_str());
//     //     return;
//     // }
// }

// void DiagnosticsWriter::writeDiagnostics(String line)
// {
//     // if (_uncommittedDataSize + (sizeof(char) * val.length()) > MAX_SD_BUFFER_BYTE_SIZE)

//     // if (!_diagnosticsFile.println(line)) {
//     //     ESP_LOGE(diagnosticsTag, "Failed to write diagnostics data");
//     //     return;
//     // }

//     // if (xPortGetCoreID() == 1 && millis() - _lastFileWrite > 5000) {
//     //     _diagnosticsFile.flush();
//     //     _lastFileWrite = millis();
//     // } 
//     // else {
//     //     _uncommittedDataSize += sizeof(char) * val.count();
//     // }
// }