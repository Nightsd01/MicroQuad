#pragma once

#include <functional>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// GPS fix information structure
struct gps_fix_info_t {
    // Position
    float latitude;          // degrees
    float longitude;         // degrees
    float altitude;          // meters above sea level
    
    // Velocity
    float speed;             // meters/second
    float course;            // degrees (true north)
    
    // Accuracy metrics
    float hdop;              // Horizontal Dilution of Precision
    uint8_t satellites;      // Number of satellites in view
    uint8_t fix_quality;     // 0=invalid, 1=GPS fix, 2=DGPS fix
    
    // Time
    uint32_t timestamp_ms;   // System timestamp when fix was received
    uint32_t gps_time_ms;    // GPS time of day in milliseconds
    
    // Validity flags
    bool position_valid;
    bool altitude_valid;
    bool speed_valid;
    bool course_valid;
    
    // Constructor with default values
    gps_fix_info_t() : 
        latitude(0.0f), longitude(0.0f), altitude(0.0f),
        speed(0.0f), course(0.0f),
        hdop(99.9f), satellites(0), fix_quality(0),
        timestamp_ms(0), gps_time_ms(0),
        position_valid(false), altitude_valid(false),
        speed_valid(false), course_valid(false) {}
};

// Telemetry event structure for GPS data (max 22 bytes)
#pragma pack(push, 1)
struct gps_telem_event_t {
    float latitude;       // 4 bytes
    float longitude;      // 4 bytes
    float altitude;       // 4 bytes
    float hdop;          // 4 bytes
    uint32_t time_ms;    // 4 bytes
    uint8_t satellites;  // 1 byte
    uint8_t fix_quality; // 1 byte
    // Total: 22 bytes
};
#pragma pack(pop)

class GPSService {
public:
    // Callback type for GPS fix updates
    using GPSCallback = std::function<void(const gps_fix_info_t&)>;
    
    // Constructor with pin configuration
    GPSService(int rxPin, int txPin, GPSCallback callback);
    
    // Destructor
    ~GPSService();
    
    // Initialize the GPS module
    void begin();
    
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
    
private:
    // GPS module configuration
    const int _rxPin;
    const int _txPin;
    static constexpr int GPS_BAUD_RATE = 9600;
    
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
    static constexpr uint32_t CONNECTION_TIMEOUT_MS = 10000;  // 10 seconds
    static constexpr uint32_t INITIAL_CONNECTION_TIMEOUT_MS = 5000;  // 5 seconds for initial connection
    
    // Last known fix
    gps_fix_info_t _lastFix;
    
    // Send initialization commands to GPS
    void _sendInitCommands();
    
    // Process parsed GPS data
    void _processGPSData();
}; 