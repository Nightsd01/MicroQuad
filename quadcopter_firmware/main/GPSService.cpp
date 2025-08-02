#include "GPSService.h"

#include <Arduino.h>
#include <esp_log.h>

static const char *TAG = "GPSService";

GPSService::GPSService(int rxPin, int txPin, GPSCallback callback)
    : _rxPin(rxPin), _txPin(txPin), _gpsSerial(2), // Use Serial2 for GPS
      _callback(callback), _isConnected(false), _lastDataReceivedMs(0),
      _connectionCheckIntervalMs(0) {}

GPSService::~GPSService() { _gpsSerial.end(); }

void GPSService::begin() {
  ESP_LOGI(TAG, "Initializing GPS on RX=%d, TX=%d at %d baud", _rxPin, _txPin,
           GPS_BAUD_RATE);

  // Initialize hardware serial for GPS
  _gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);

  // Wait a bit for GPS to stabilize
  delay(100);

  // Clear any garbage data
  while (_gpsSerial.available()) {
    _gpsSerial.read();
  }

  // Send initialization commands
  _sendInitCommands();

  // Mark initial connection check time
  _connectionCheckIntervalMs = millis();

  ESP_LOGI(TAG, "GPS initialization complete, waiting for data...");
}

void GPSService::loopHandler() {
  bool dataReceived = false;

  // Process all available serial data
  while (_gpsSerial.available()) {
    char c = _gpsSerial.read();
    dataReceived = true;

    // Feed the character to TinyGPS++
    if (_gps.encode(c)) {
      // A complete NMEA sentence was processed
      _processGPSData();
    }
  }

  // Update connection status
  if (dataReceived) {
    _lastDataReceivedMs = millis();
    if (!_loggedConnectionStatus && !_isConnected) {
      _isConnected = true;
      _loggedConnectionStatus = true;
      ESP_LOGI(TAG, "GPS module connected and responding");
    }
  }

  // Check for connection timeout
  uint32_t currentMs = millis();
  uint32_t timeSinceLastData = currentMs - _lastDataReceivedMs;

  // Initial connection check
  if (!_isConnected && _lastDataReceivedMs == 0) {
    if (!_loggedConnectionStatus && currentMs - _connectionCheckIntervalMs >
                                        INITIAL_CONNECTION_TIMEOUT_MS) {
      _loggedConnectionStatus = true;
      ESP_LOGE(TAG,
               "GPS module not responding after %lu ms. Check wiring (RX=%d, "
               "TX=%d) and power.",
               (unsigned long)INITIAL_CONNECTION_TIMEOUT_MS, _rxPin, _txPin);
      _connectionCheckIntervalMs = currentMs; // Reset to avoid spamming logs
    }
  }
  // Ongoing connection monitoring
  else if (_isConnected && timeSinceLastData > CONNECTION_TIMEOUT_MS) {
    _isConnected = false;
    _loggedConnectionStatus = false;
    ESP_LOGE(TAG, "GPS connection lost - no data received for %lu ms",
             (unsigned long)CONNECTION_TIMEOUT_MS);
  }

  // Periodic status logging if not connected
  static uint32_t lastStatusLogMs = 0;
  if (!_isConnected &&
      currentMs - lastStatusLogMs > 30000) { // Log every 30 seconds
    lastStatusLogMs = currentMs;
    ESP_LOGE(TAG, "GPS still not connected. No data received. Check GPS module "
                  "and antenna.");
  }
}

uint32_t GPSService::getTimeSinceLastData() const {
  if (_lastDataReceivedMs == 0) {
    return UINT32_MAX; // Never received data
  }
  return millis() - _lastDataReceivedMs;
}

gps_telem_event_t GPSService::getTelemetryData() const {
  gps_telem_event_t telem;

  telem.latitude = _lastFix.latitude;
  telem.longitude = _lastFix.longitude;
  telem.altitude = _lastFix.altitude;
  telem.hdop = _lastFix.hdop;
  telem.satellites = _lastFix.satellites;
  telem.fix_quality = _lastFix.fix_quality;

  return telem;
}

void GPSService::_sendInitCommands() {
  ESP_LOGI(TAG, "Sending GPS initialization commands");

  // Query version number
  _gpsSerial.println("$PQVERNO,R*3F");
  delay(100);

  // Set search mode to GPS + GLONASS
  _gpsSerial.println("$PMTK353,1,1,0,0,0*2B");
  delay(100);

  // Configure NMEA output sentences
  // Enable RMC, VTG, GGA, GSA, GSV
  _gpsSerial.println("$PMTK314,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);

  // Set update rate to 1Hz (1000ms)
  _gpsSerial.println("$PMTK220,1000*1F");
  delay(100);

  ESP_LOGI(TAG, "GPS initialization commands sent");
}

void GPSService::_processGPSData() {
  gps_fix_info_t fix;

  // Get system timestamp
  fix.timestamp_ms = millis();

  // Extract position data
  if (_gps.location.isValid()) {
    fix.latitude = static_cast<float>(_gps.location.lat());
    fix.longitude = static_cast<float>(_gps.location.lng());
    fix.position_valid = true;
  } else {
    fix.position_valid = false;
  }

  // Extract altitude
  if (_gps.altitude.isValid()) {
    fix.altitude = static_cast<float>(_gps.altitude.meters());
    fix.altitude_valid = true;
  } else {
    fix.altitude_valid = false;
  }

  // Extract speed (convert from km/h to m/s)
  if (_gps.speed.isValid()) {
    fix.speed = static_cast<float>(_gps.speed.kmph() / 3.6);
    fix.speed_valid = true;
  } else {
    fix.speed_valid = false;
  }

  // Extract course
  if (_gps.course.isValid()) {
    fix.course = static_cast<float>(_gps.course.deg());
    fix.course_valid = true;
  } else {
    fix.course_valid = false;
  }

  // Extract satellite and accuracy information
  if (_gps.satellites.isValid()) {
    fix.satellites = static_cast<uint8_t>(_gps.satellites.value());
  } else {
    fix.satellites = 0;
  }

  if (_gps.hdop.isValid()) {
    fix.hdop = static_cast<float>(_gps.hdop.hdop());
  } else {
    fix.hdop = 99.9f;
  }

  // Determine fix quality based on satellite count and HDOP
  if (fix.position_valid) {
    if (fix.satellites >= 4 && fix.hdop < 5.0f) {
      fix.fix_quality = 2; // Good fix
    } else if (fix.satellites >= 3) {
      fix.fix_quality = 1; // Basic fix
    } else {
      fix.fix_quality = 0; // Invalid
    }
  } else {
    fix.fix_quality = 0;
  }

  // Extract GPS time
  if (_gps.time.isValid()) {
    fix.gps_time_ms =
        (_gps.time.hour() * 3600000UL) + (_gps.time.minute() * 60000UL) +
        (_gps.time.second() * 1000UL) + _gps.time.centisecond() * 10;
  } else {
    fix.gps_time_ms = 0;
  }

  // Store the last fix
  _lastFix = fix;

  // Always call the callback to send telemetry updates, even without a valid
  // fix This allows the iOS app to show GPS status (satellites, connection,
  // etc.)
  if (_callback) {
    _callback(fix);
  }

  // Log GPS status periodically
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime > 5000) { // Log every 5 seconds
    lastLogTime = millis();

    if (fix.position_valid) {
      ESP_LOGI(
          TAG, "GPS Fix: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d, HDOP=%.1f",
          fix.latitude, fix.longitude, fix.altitude, fix.satellites, fix.hdop);
    } else {
      ESP_LOGW(TAG, "No GPS fix. Satellites: %d", fix.satellites);
    }
  }
}