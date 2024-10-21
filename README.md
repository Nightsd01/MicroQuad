# MicroQuad
## An ESP32 based micro-quadcopter project
This repo is based on the ESP32 architecture to implement a Bluetooth controlled 'smart' quadcopter, it is in the early stages. 

### To Build & Flash
- Run `idf.py build` to check if the project builds
- Run `idf.py flash` to flash to a connected ESP32

### To Debug in VS Code 
- Launch OpenOCD either through VS code (currently not working for me) or via the command `idf.py openocd`
- In VS Code's debug menu click 'Start Debugging' with the [Eclipse CDT Remote](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/DEBUGGING.md) target

### Notes on Hacks
- Currently there are a few hardcoded hacks here:
- In VS Code's `.vscode/settings.json`, we are currently hardcoding a lot of `esp32` executable paths which is not ideal because it will break whenever versions change
- Also in `settings.json` we are hard coding the device serial port which is even hackier
