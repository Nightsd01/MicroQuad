#pragma once

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include <cstdint>
#include <functional>
#include <vector>

// GPS fix information structure
struct gps_fix_info_t
{
  double latitude;
  double longitude;
  double altitude;  // meters
  float speed;      // m/s
  float course;     // degrees
  uint8_t satellites;
  float hdop;
  uint8_t fix_quality;  // 0=no fix, 1=GPS fix, 2=DGPS fix
  bool position_valid;
  uint32_t timestamp_ms;  // millis() when fix was obtained
};

// Telemetry event structure (subset for BLE transmission)
struct gps_telem_event_t
{
  float latitude;
  float longitude;
  float altitude;
  float hdop;
  uint8_t satellites;
  uint8_t fix_quality;
} __attribute__((packed));

// Callback type for GPS fix updates
using GPSCallback = std::function<void(const gps_fix_info_t&)>;

class GPSService
{
 public:
  // Constructor
  // rxPin: ESP32 pin connected to GPS TX
  // txPin: ESP32 pin connected to GPS RX
  // callback: Function called when GPS fix is updated (optional)
  GPSService(int rxPin, int txPin, GPSCallback callback = nullptr);

  // Destructor
  ~GPSService();

  // Initialize the GPS module
  bool begin();

  // Process incoming GPS data (call this regularly from main loop)
  void loopHandler();

  // Get the last known position
  gps_fix_info_t getLastFix() const { return _lastFix; }

  // Get telemetry event data
  gps_telem_event_t getTelemetryData() const;

  // Check if GPS has a valid fix
  bool hasValidFix() const { return _lastFix.position_valid; }

  // Get the number of satellites in view
  uint8_t getSatelliteCount() const { return _lastFix.satellites; }

  // Check if GPS module is responding
  bool isConnected() const { return _isConnected; }

  // Get time since last GPS data received (milliseconds)
  uint32_t getTimeSinceLastData() const;

  // Apply quick startup ephemeris and seed location/time to the GPS module
  // ephemerisBytes: raw EPO/AGPS payload
  // latitude/longitude in degrees, currentTimestampSecs in seconds since 1970
  void applyQuickStartupEphemeris(
      uint8_t* ephemerisBytes, size_t ephemerisSize, double latitude, double longitude, double currentTimestampSecs);

 private:
  // GPS module configuration
  const int _rxPin;
  const int _txPin;
  static constexpr int GPS_BAUD_RATE = 115200;
  static constexpr int GPS_FALLBACK_BAUD_RATE = 9600;

  // Hardware serial for GPS communication
  HardwareSerial _gpsSerial;

  // TinyGPS++ object for NMEA parsing
  TinyGPSPlus _gps;

  // Callback for fix updates
  GPSCallback _callback;

  // Connection status
  bool _isConnected;
  bool _loggedConnectionStatus = false;
  uint32_t _lastDataReceivedMs;
  uint32_t _connectionCheckIntervalMs;
  static constexpr uint32_t CONNECTION_TIMEOUT_MS = 10000;         // 10 seconds
  static constexpr uint32_t INITIAL_CONNECTION_TIMEOUT_MS = 5000;  // 5 seconds for initial connection

  // GPS restart status
  bool _waitingForRestart = false;
  uint32_t _restartWaitStartMs = 0;
  static constexpr uint32_t RESTART_TIMEOUT_MS = 5000;  // 5 seconds to wait for restart

  // Last known fix
  gps_fix_info_t _lastFix;

  // EPO upload state machine
  enum class EPOUploadState
  {
    IDLE,
    CLEAR_EPO,
    SEND_CHUNKS,
    ENABLE_EPO,
    SEND_POSITION,
    SEND_TIME,
    COMPLETE
  };

  struct EPOUploadContext
  {
    uint8_t* data = nullptr;
    size_t dataSize = 0;
    size_t currentOffset = 0;
    double latitude = 0;
    double longitude = 0;
    double timestampSecs = 0;
    EPOUploadState state = EPOUploadState::IDLE;
    uint64_t lastActionTimeMs = 0;
    uint32_t delayBetweenActions = 50;  // ms
  };

  EPOUploadContext _epoUpload;

  // Command queue management
  struct GPSCommand
  {
    String command;
    bool waitForAck;
    uint32_t timeoutMs;
  };

  std::vector<GPSCommand> _commandQueue;
  bool _waitingForAck = false;
  uint32_t _ackWaitStartMs = 0;
  uint32_t _currentAckTimeout = 1000;  // Default 1 second timeout
  String _lastSentCommand;

  // Process EPO upload state machine
  void _processEPOUpload();

  // Send initialization commands to GPS
  void _sendInitCommands();

  // Send initialization commands after GPS restart
  void _sendInitCommandsAfterRestart();

  // Detect and configure GPS baud rate
  bool _detectAndConfigureBaudRate();

  // Process parsed GPS data
  void _processGPSData();

  // Send GPS command with optional ACK wait
  void _sendGPSCommand(const char* command, bool waitForAck = false, uint32_t timeoutMs = 1000);

  // Process command queue
  void _processCommandQueue();

  // Check for PMTK001 acknowledgment
  void _checkForAck(const char* nmeaSentence);
};

#endif  // MATLAB_SIM