# MicroQuad
## An ESP32 based micro-quadcopter project [Note: Not Flying Yet!]
This repo is based on the ESP32 architecture to implement a custom Bluetooth controlled 'smart' quadcopter, with a custom PCB design, using the BetaFPV 65S frame + brushless motors, it is in the early stages. 

### Features
- ESP32-S3 SoC
- ICM42688P Accelerometer + Gyro
- RYS8830 GPS
- QMC5883L Magnetometer
- BME280 Air Pressure Sensor
- VL53L1CX Range Finder
- MCP73871T Battery Mgmt + Load Sharing
- Breakout Board with USB-C via Small Ribbon Cable

![alt text](assets/header_img.png)

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

### TODO List
- [ ] Update firmware to utilize radians everywhere in the codebase instead of using degrees
- [ ] Clean up the DebugHelper, there is a lot of now-unncessesary code in there
- [ ] Break up the codebase in general, for example the core stability logic should be extracted to a different component from the Logger
