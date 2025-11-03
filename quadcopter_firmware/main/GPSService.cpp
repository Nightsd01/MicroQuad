#include "GPSService.h"

#include <Arduino.h>
#include <inttypes.h>  // for PRIx32
#include <stdint.h>

#include <algorithm>  // for std::min

#include "Logger.h"

// ---------- small UTC helper (no dependencies) ----------
namespace
{
struct UTCTime
{
  int year, month, day, hour, minute, second;
};

static inline bool isLeapYear(int y) { return (y % 4 == 0) && ((y % 100 != 0) || (y % 400 == 0)); }

static UTCTime epochToUTC(double epochSecs)
{
  // Convert (approx.) Unix epoch seconds -> UTC date/time (ignores leap seconds).
  // Works well for embedded (ESP32).
  uint64_t t = (epochSecs >= 0.0) ? (uint64_t)(epochSecs + 0.5) : 0ULL;

  uint32_t secOfDay = (uint32_t)(t % 86400ULL);
  uint64_t days = t / 86400ULL;

  UTCTime out{};
  out.hour = secOfDay / 3600U;
  secOfDay %= 3600U;
  out.minute = secOfDay / 60U;
  out.second = secOfDay % 60U;

  int y = 1970;
  while (true) {
    uint32_t diy = isLeapYear(y) ? 366U : 365U;
    if (days >= diy) {
      days -= diy;
      ++y;
    } else {
      break;
    }
  }
  out.year = y;

  static const uint8_t mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int m = 0;
  while (m < 12) {
    uint32_t dim = mdays[m];
    if (m == 1 && isLeapYear(y)) ++dim;  // February in a leap year
    if (days >= dim) {
      days -= dim;
      ++m;
    } else {
      break;
    }
  }
  out.month = m + 1;
  out.day = (int)days + 1;
  return out;
}

// Build "$XXX,...*CS" into dest; dest must have room.
// Returns length written (excluding terminating '\0').
static int appendNMEAChecksum(char* dest, size_t capacity)
{
  if (capacity == 0) return 0;
  size_t len = strnlen(dest, capacity);
  uint8_t cs = 0;
  // XOR from first char AFTER '$' up to the end (excluded '*')
  for (size_t i = 1; i < len; ++i) cs ^= (uint8_t)dest[i];
  int n = snprintf(dest + len, capacity - len, "*%02X", cs);
  if (n < 0) return (int)len;
  return (int)(len + n);
}

}  // namespace
// --------------------------------------------------------

GPSService::GPSService(int rxPin, int txPin, GPSCallback callback)
    : _rxPin(rxPin),
      _txPin(txPin),
      _gpsSerial(2),  // Use Serial2 for GPS
      _callback(callback),
      _isConnected(false),
      _lastDataReceivedMs(0),
      _connectionCheckIntervalMs(0)
{
}

GPSService::~GPSService() { _gpsSerial.end(); }

bool GPSService::begin()
{
  LOG_INFO("Initializing GPS on RX=%d, TX=%d", _rxPin, _txPin);

  // Detect current GPS baud rate and configure to 115200
  if (!_detectAndConfigureBaudRate()) {
    LOG_ERROR("Failed to detect GPS module baud rate");
    return false;
  }

  // Send initialization commands
  _sendInitCommands();

  // Mark initial connection check time
  _connectionCheckIntervalMs = millis();

  LOG_INFO("GPS initialization complete at %d baud, waiting for data...", GPS_BAUD_RATE);
  return true;
}

void GPSService::loopHandler()
{
  bool dataReceived = false;
  static String nmeaBuffer;

  // 1) Read ALL available serial first so ACKs in the buffer don't "time out"
  while (_gpsSerial.available()) {
    char c = _gpsSerial.read();
    dataReceived = true;

    // Build NMEA sentence for ACK checking
    if (c == '$') {
      nmeaBuffer = "$";
    } else if (c == '\n' || c == '\r') {
      if (nmeaBuffer.length() > 0) {
        // Log every NMEA sentence received
        LOG_INFO("GPS RX: %s", nmeaBuffer.c_str());

        if (nmeaBuffer.startsWith("$PMTK001")) {
          _checkForAck(nmeaBuffer.c_str());  // <-- This can clear _waitingForAck
        } else if (nmeaBuffer.startsWith("$PMTK010,001")) {
          // Module has restarted (PMTK010,001 notification)
          if (_waitingForRestart) {
            LOG_INFO("GPS module restart complete (PMTK010,001 received)");
            _waitingForRestart = false;
            // Now send the rest of the init commands
            _sendInitCommandsAfterRestart();
          }
        }
        nmeaBuffer = "";
      }
    } else {
      nmeaBuffer += c;
    }

    // Feed TinyGPS++
    if (_gps.encode(c)) {
      _processGPSData();
    }
  }

  // 2) EPO state machine (may enqueue commands)
  if (_epoUpload.state != EPOUploadState::IDLE) {
    _processEPOUpload();
  }

  // 3) Check for restart timeout
  if (_waitingForRestart && (millis() - _restartWaitStartMs > RESTART_TIMEOUT_MS)) {
    LOG_WARN("GPS restart timeout - proceeding with initialization anyway");
    _waitingForRestart = false;
    _sendInitCommandsAfterRestart();
  }

  // 4) Process command queue / timeouts AFTER consuming serial
  _processCommandQueue();

  // ----- (keep your connection status logic as-is below) -----
  if (dataReceived) {
    _lastDataReceivedMs = millis();
    if (!_loggedConnectionStatus && !_isConnected) {
      _isConnected = true;
      _loggedConnectionStatus = true;
      LOG_INFO("GPS module connected and responding");
    }
  }

  uint32_t currentMs = millis();
  uint32_t timeSinceLastData = currentMs - _lastDataReceivedMs;

  if (!_isConnected && _lastDataReceivedMs == 0) {
    if (!_loggedConnectionStatus && currentMs - _connectionCheckIntervalMs > INITIAL_CONNECTION_TIMEOUT_MS) {
      _loggedConnectionStatus = true;
      LOG_ERROR(
          "GPS module not responding after %lu ms. Check wiring (RX=%d, TX=%d) and power.",
          (unsigned long)INITIAL_CONNECTION_TIMEOUT_MS,
          _rxPin,
          _txPin);
      _connectionCheckIntervalMs = currentMs;
    }
  } else if (_isConnected && timeSinceLastData > CONNECTION_TIMEOUT_MS) {
    _isConnected = false;
    _loggedConnectionStatus = false;
    LOG_ERROR("GPS connection lost - no data received for %lu ms", (unsigned long)CONNECTION_TIMEOUT_MS);
  }

  if (!_isConnected) {
    LOG_ERROR_PERIODIC_MILLIS(30000, "GPS still not connected. No data received. Check GPS module and antenna.");
  }
}

uint32_t GPSService::getTimeSinceLastData() const
{
  if (_lastDataReceivedMs == 0) {
    return UINT32_MAX;  // Never received data
  }
  return millis() - _lastDataReceivedMs;
}

gps_telem_event_t GPSService::getTelemetryData() const
{
  gps_telem_event_t telem;

  telem.latitude = _lastFix.latitude;
  telem.longitude = _lastFix.longitude;
  telem.altitude = _lastFix.altitude;
  telem.hdop = _lastFix.hdop;
  telem.satellites = _lastFix.satellites;
  telem.fix_quality = _lastFix.fix_quality;

  return telem;
}

// Helper: write an NMEA/PMTK command (non-blocking version)
// DEPRECATED - use _sendGPSCommand instead
// static inline void gps_write_command(HardwareSerial &serial, const char *cmd) { serial.println(cmd); }

void GPSService::applyQuickStartupEphemeris(
    uint8_t* ephemerisBytes, size_t ephemerisSize, double latitude, double longitude, double currentTimestampSecs)
{
  if (!_isConnected) {
    LOG_ERROR("Cannot apply quick startup ephemeris: GPS not connected");
    return;
  }

  // Check if already uploading
  if (_epoUpload.state != EPOUploadState::IDLE) {
    LOG_WARN("EPO upload already in progress, ignoring new request");
    return;
  }

  // Basic validation: Host-EPO uses 72-byte per record
  if (ephemerisSize == 0 || (ephemerisSize % 72) != 0) {
    LOG_ERROR("EPO: size %u is not a multiple of 72 bytes; wrong file.", (unsigned)ephemerisSize);
    return;
  }

  LOG_INFO(
      "Starting non-blocking quick-start GPS seed: lat=%.6f, lon=%.6f, t=%.0f, epoBytes=%u",
      latitude,
      longitude,
      currentTimestampSecs,
      (unsigned)ephemerisSize);

  // Initialize EPO upload context
  _epoUpload.data = ephemerisBytes;  // Note: caller must keep this memory valid!
  _epoUpload.dataSize = ephemerisSize;
  _epoUpload.currentOffset = 0;
  _epoUpload.latitude = latitude;
  _epoUpload.longitude = longitude;
  _epoUpload.timestampSecs = currentTimestampSecs;
  _epoUpload.state = EPOUploadState::CLEAR_EPO;
  _epoUpload.lastActionTimeMs = millis();
  _epoUpload.delayBetweenActions = 50;
}

void GPSService::_processEPOUpload()
{
  // Check if enough time has passed since last action
  if (millis() - _epoUpload.lastActionTimeMs < _epoUpload.delayBetweenActions) {
    return;  // Still waiting
  }

  // Static flags scoped to this method to avoid header changes
  static bool s_scanLogged = false;

  switch (_epoUpload.state) {
    case EPOUploadState::CLEAR_EPO:
      // Clear existing EPO data
      _sendGPSCommand("$PMTK127*36", true, 2000);    // Wait for ACK with 2s timeout
      _epoUpload.state = EPOUploadState::SEND_TIME;  // Send time FIRST before EPO data
      _epoUpload.lastActionTimeMs = millis();
      _epoUpload.delayBetweenActions = 50;
      s_scanLogged = false;  // reset for a new upload session
      LOG_INFO("EPO: Cleared existing data");
      break;

    case EPOUploadState::SEND_CHUNKS: {
      const size_t EPO_SATELLITE_BLOCK_SIZE = 72;  // 18 words × 4 bytes

      // Optional: one-time pre-scan to help diagnose wrong file types
      if (!s_scanLogged && _epoUpload.currentOffset == 0) {
        size_t count = _epoUpload.dataSize / EPO_SATELLITE_BLOCK_SIZE;
        size_t gpsCnt = 0, gloCnt = 0, headerCnt = 0, otherCnt = 0;
        for (size_t off = 0; off + EPO_SATELLITE_BLOCK_SIZE <= _epoUpload.dataSize; off += EPO_SATELLITE_BLOCK_SIZE) {
          uint8_t svid = _epoUpload.data[off + 3];
          if (svid == 0xFE || svid == 0xFF) {
            headerCnt++;
            continue;
          }
          if (svid >= 1 && svid <= 32) {
            gpsCnt++;
            continue;
          }
          if (svid >= 65 && svid <= 88) {
            gloCnt++;
            continue;
          }
          otherCnt++;
        }
        LOG_INFO(
            "EPO scan: total=%u, gps=%u, glo=%u, header=%u, other=%u",
            (unsigned)count,
            (unsigned)gpsCnt,
            (unsigned)gloCnt,
            (unsigned)headerCnt,
            (unsigned)otherCnt);
        if ((gpsCnt + gloCnt) == 0) {
          LOG_ERROR("EPO: No GPS/GLONASS records found; this file is not compatible with PMTK721 Host-EPO.");
          _epoUpload.state = EPOUploadState::COMPLETE;
          _epoUpload.lastActionTimeMs = millis();
          break;
        }
        s_scanLogged = true;
      }

      if (_epoUpload.currentOffset >= _epoUpload.dataSize) {
        // All satellites processed
        _epoUpload.state = EPOUploadState::COMPLETE;
        _epoUpload.lastActionTimeMs = millis();
        _epoUpload.delayBetweenActions = 50;
        LOG_INFO("EPO: All satellite data sent (%zu bytes)", _epoUpload.dataSize);
        break;
      }

      // Check if we have a complete satellite block
      if (_epoUpload.currentOffset + EPO_SATELLITE_BLOCK_SIZE > _epoUpload.dataSize) {
        LOG_WARN(
            "EPO: Incomplete satellite block at offset %zu, skipping remaining %zu bytes",
            _epoUpload.currentOffset,
            _epoUpload.dataSize - _epoUpload.currentOffset);
        _epoUpload.state = EPOUploadState::COMPLETE;
        _epoUpload.lastActionTimeMs = millis();
        break;
      }

      // Extract satellite ID directly from the record
      // The SVID is the MSB of the first 32-bit word (byte index 3 of the record).
      uint32_t firstWord = 0;
      for (int byteIdx = 0; byteIdx < 4; byteIdx++) {
        firstWord |= ((uint32_t)_epoUpload.data[_epoUpload.currentOffset + byteIdx]) << (byteIdx * 8);
      }
      uint8_t satId = (firstWord >> 24) & 0xFF;
      uint8_t blockIndex = _epoUpload.currentOffset / EPO_SATELLITE_BLOCK_SIZE;

      // Skip constellation headers (some EPO/QEPO variants include them)
      if (satId == 0xFE || satId == 0xFF) {
        _epoUpload.currentOffset += EPO_SATELLITE_BLOCK_SIZE;
        _epoUpload.lastActionTimeMs = millis();
        // Not an error; just a header/meta record.
        return;
      }

      // Accept only GPS (1..32) and GLONASS (65..88) for PMTK721 Host-EPO
      bool isGPS = (satId >= 1 && satId <= 32);
      bool isGLONASS = (satId >= 65 && satId <= 88);
      if (!isGPS && !isGLONASS) {
        // Quietly skip unsupported constellations (e.g., BDS/GAL) for Host-EPO
        _epoUpload.currentOffset += EPO_SATELLITE_BLOCK_SIZE;
        _epoUpload.lastActionTimeMs = millis();
        return;
      }

      // Build PMTK721 command
      // Format: $PMTK721,<SVID-HEX>,<W0>,<W1>,...,<W17>*<checksum>
      char cmd[512];
      int cmdLen = snprintf(cmd, sizeof(cmd), "$PMTK721,%02X", (unsigned int)satId);

      // Add 18 little-endian words from the EPO data
      for (int wordIdx = 0; wordIdx < 18; wordIdx++) {
        uint32_t word = 0;
        for (int byteIdx = 0; byteIdx < 4; byteIdx++) {
          size_t dataIdx = _epoUpload.currentOffset + (wordIdx * 4) + byteIdx;
          uint8_t byte = _epoUpload.data[dataIdx];
          word |= ((uint32_t)byte) << (byteIdx * 8);
        }
        cmdLen += snprintf(cmd + cmdLen, sizeof(cmd) - cmdLen, ",%08" PRIX32, word);
      }

      // Calculate NMEA checksum and append
      {
        uint8_t cs = 0;
        for (int i = 1; i < cmdLen; ++i) {
          cs ^= static_cast<uint8_t>(cmd[i]);
        }
        snprintf(cmd + cmdLen, sizeof(cmd) - cmdLen, "*%02X", cs);
      }

      // Debug log for the first satellite to verify format
      if (blockIndex == 0) {
        LOG_INFO("EPO: First PMTK721 command (first 100 chars): %.100s", cmd);
      }

      _sendGPSCommand(cmd, true, 5000);  // Wait for ACK with 5s timeout per satellite
      _epoUpload.currentOffset += EPO_SATELLITE_BLOCK_SIZE;
      _epoUpload.lastActionTimeMs = millis();
      _epoUpload.delayBetweenActions = 50;  // Small delay between satellites

      // Log progress
      uint8_t totalSatellites = _epoUpload.dataSize / EPO_SATELLITE_BLOCK_SIZE;
      if (isGPS) {
        LOG_INFO(
            "EPO: Sent GPS satellite block %d/%d (PRN %d, ID 0x%02X)",
            blockIndex + 1,
            totalSatellites,
            satId,
            satId);
      } else {
        LOG_INFO(
            "EPO: Sent GLONASS satellite block %d/%d (Slot %d, ID 0x%02X)",
            blockIndex + 1,
            totalSatellites,
            satId - 64,
            satId);
      }
      break;
    }

    case EPOUploadState::SEND_TIME: {
      // Provide UTC time (PMTK740,YYYY,MM,DD,hh,mm,ss)
      UTCTime t = epochToUTC(_epoUpload.timestampSecs);
      char buf[64];
      int n = snprintf(
          buf,
          sizeof(buf),
          "$PMTK740,%04d,%d,%d,%d,%d,%d",
          t.year,
          t.month,
          t.day,
          t.hour,
          t.minute,
          t.second);
      if (n < 0 || (size_t)n >= sizeof(buf)) {
        LOG_ERROR("EPO: PMTK740 build failed");
        _epoUpload.state = EPOUploadState::COMPLETE;
        break;
      }
      appendNMEAChecksum(buf, sizeof(buf));
      _sendGPSCommand(buf, true, 2000);  // Wait for ACK with 2s timeout

      _epoUpload.state = EPOUploadState::SEND_POSITION;  // Now send position
      _epoUpload.lastActionTimeMs = millis();
      _epoUpload.delayBetweenActions = 50;
      LOG_INFO("EPO: Sent time aiding (PMTK740)");
      break;
    }

    case EPOUploadState::SEND_POSITION: {
      // Provide reference position + time (PMTK741)
      // If altitude unknown, send 0.0
      UTCTime t = epochToUTC(_epoUpload.timestampSecs);
      char buf[128];
      int n = snprintf(
          buf,
          sizeof(buf),
          "$PMTK741,%.6f,%.6f,%.1f,%04d,%d,%d,%d,%d,%d",
          _epoUpload.latitude,
          _epoUpload.longitude,
          0.0f,
          t.year,
          t.month,
          t.day,
          t.hour,
          t.minute,
          t.second);
      if (n < 0 || (size_t)n >= sizeof(buf)) {
        LOG_ERROR("EPO: PMTK741 build failed");
        _epoUpload.state = EPOUploadState::COMPLETE;
        break;
      }
      appendNMEAChecksum(buf, sizeof(buf));
      _sendGPSCommand(buf, true, 2000);  // Wait for ACK with 2s timeout

      _epoUpload.state = EPOUploadState::SEND_CHUNKS;  // Now send the actual EPO data
      _epoUpload.lastActionTimeMs = millis();
      _epoUpload.delayBetweenActions = 50;
      LOG_INFO("EPO: Sent position aiding (PMTK741)");
      break;
    }

    case EPOUploadState::ENABLE_EPO:
      // Not used: PMTK313 is SBAS enable/disable, not "enable EPO".
      // For backward compatibility with any existing state transitions,
      // just finalize here.
      LOG_INFO("EPO: Finalizing (ENABLE_EPO stage not used).");
      _epoUpload.state = EPOUploadState::COMPLETE;
      _epoUpload.lastActionTimeMs = millis();
      _epoUpload.delayBetweenActions = 50;
      break;

    case EPOUploadState::COMPLETE:
      // Clean up and return to idle
      LOG_INFO("EPO: Upload complete! GPS should achieve faster fix now.");
      _epoUpload.data = nullptr;
      _epoUpload.dataSize = 0;
      _epoUpload.currentOffset = 0;
      _epoUpload.state = EPOUploadState::IDLE;
      break;

    case EPOUploadState::IDLE:
    default:
      break;
  }
}

bool GPSService::_detectAndConfigureBaudRate()
{
  // First try at 115200 (our target baud rate)
  LOG_INFO("Trying GPS at %d baud...", GPS_BAUD_RATE);
  _gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
  delay(50);

  // Clear any garbage data
  while (_gpsSerial.available()) {
    _gpsSerial.read();
  }

  // Send a test command (query firmware version)
  const char* testCmd = "$PMTK605*31";  // Query firmware release information
  _gpsSerial.println(testCmd);
  LOG_INFO("GPS TX (baud detect): %s", testCmd);

  // Wait for response
  unsigned long startTime = millis();
  bool gotResponse = false;
  while (millis() - startTime < 500) {  // Wait up to 500ms
    if (_gpsSerial.available()) {
      gotResponse = true;
      // Clear the response
      while (_gpsSerial.available()) {
        _gpsSerial.read();
      }
      break;
    }
    delay(10);
  }

  if (gotResponse) {
    LOG_INFO("GPS detected at %d baud", GPS_BAUD_RATE);
    return true;
  }

  // If no response at 115200, try 9600 (default baud rate)
  LOG_INFO("No response at %d baud, trying %d baud...", GPS_BAUD_RATE, GPS_FALLBACK_BAUD_RATE);
  _gpsSerial.end();
  delay(10);

  _gpsSerial.begin(GPS_FALLBACK_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
  delay(50);

  // Clear any garbage data
  while (_gpsSerial.available()) {
    _gpsSerial.read();
  }

  // Send test command again
  _gpsSerial.println(testCmd);
  LOG_INFO("GPS TX (baud detect at %d): %s", GPS_FALLBACK_BAUD_RATE, testCmd);

  // Wait for response
  startTime = millis();
  gotResponse = false;
  while (millis() - startTime < 500) {
    if (_gpsSerial.available()) {
      gotResponse = true;
      // Clear the response
      while (_gpsSerial.available()) {
        _gpsSerial.read();
      }
      break;
    }
    delay(10);
  }

  if (!gotResponse) {
    LOG_ERROR("GPS module not responding at either %d or %d baud", GPS_BAUD_RATE, GPS_FALLBACK_BAUD_RATE);
    return false;
  }

  LOG_INFO("GPS detected at %d baud, switching to %d baud...", GPS_FALLBACK_BAUD_RATE, GPS_BAUD_RATE);

  // Send command to switch to 115200 baud
  // PMTK251 sets baud rate: $PMTK251,115200*1F
  const char* baudCmd = "$PMTK251,115200*1F";
  _gpsSerial.println(baudCmd);
  LOG_INFO("GPS TX (switch baud): %s", baudCmd);
  delay(100);  // Give GPS time to process the command

  // Switch ESP32 serial to 115200
  _gpsSerial.end();
  delay(10);
  _gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
  delay(50);

  // Verify the switch worked
  _gpsSerial.println(testCmd);
  LOG_INFO("GPS TX (verify baud at %d): %s", GPS_BAUD_RATE, testCmd);
  startTime = millis();
  gotResponse = false;
  while (millis() - startTime < 500) {
    if (_gpsSerial.available()) {
      gotResponse = true;
      // Clear the response
      while (_gpsSerial.available()) {
        _gpsSerial.read();
      }
      break;
    }
    delay(10);
  }

  if (gotResponse) {
    LOG_INFO("Successfully switched GPS to %d baud", GPS_BAUD_RATE);
    return true;
  } else {
    LOG_ERROR("Failed to switch GPS to %d baud", GPS_BAUD_RATE);
    return false;
  }
}

void GPSService::_sendInitCommands()
{
  LOG_INFO("Restarting GPS module with PMTK101");

  // Send hot restart command (PMTK101) - Module does NOT ACK this command
  // It will respond with PMTK010,001 when restart is complete
  const char* restartCmd = "$PMTK101*32";
  _gpsSerial.println(restartCmd);
  LOG_INFO("GPS TX: %s", restartCmd);

  // Set flag to wait for restart notification
  _waitingForRestart = true;
  _restartWaitStartMs = millis();

  LOG_INFO("Waiting for GPS module restart (PMTK010,001)...");
}

void GPSService::_sendInitCommandsAfterRestart()
{
  LOG_INFO("Sending GPS initialization commands after restart");

  // Query version number (no ACK expected)
  _sendGPSCommand("$PQVERNO,R*3F", false);
  delay(100);  // Give module time to respond

  // Set search mode to GPS + GLONASS first (expects ACK)
  // This may be required before the module accepts NMEA configuration
  _sendGPSCommand("$PMTK353,1,1,0,0,0*2B", true, 2000);

  // Small delay between commands for module processing
  delay(50);

  // Configure NMEA output sentences (expects ACK)
  // Enable RMC, VTG, GGA, GSA, GSV (19 fields total)
  // Fields: GLL,RMC,VTG,GGA,GSA,GSV,GRS,GST,Reserved(9),ZDA,MCHN
  _sendGPSCommand("$PMTK314,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29", true, 3000);

  // Small delay between commands
  delay(50);

  // 1 Hz navigation & NMEA rate (match both)
  _sendGPSCommand("$PMTK300,1000,0,0,0,0*1C", true, 2000);  // fix interval 1000 ms
  _sendGPSCommand("$PMTK220,1000*1F", true, 2000);          // NMEA interval 1000 ms

  // Set update rate to 10Hz (100ms) - faster with 115200 baud (expects ACK)
  // _sendGPSCommand("$PMTK220,100*2F", true, 2000);

  LOG_INFO("GPS initialization commands queued");
}

void GPSService::_processGPSData()
{
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
  } else {
    fix.altitude = 0.0;
  }

  // Extract speed (convert from km/h to m/s)
  if (_gps.speed.isValid()) {
    fix.speed = static_cast<float>(_gps.speed.kmph() / 3.6);
  } else {
    fix.speed = 0.0;
  }

  // Extract course
  if (_gps.course.isValid()) {
    fix.course = static_cast<float>(_gps.course.deg());
  } else {
    fix.course = 0.0;
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
      fix.fix_quality = 2;  // Good fix
    } else if (fix.satellites >= 3) {
      fix.fix_quality = 1;  // Basic fix
    } else {
      fix.fix_quality = 0;  // Invalid
    }
  } else {
    fix.fix_quality = 0;
  }

  // Set timestamp
  fix.timestamp_ms = millis();

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
  if (millis() - lastLogTime > 5000) {  // Log every 5 seconds
    lastLogTime = millis();

    if (fix.position_valid) {
      LOG_INFO(
          "GPS Fix: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d, HDOP=%.1f",
          fix.latitude,
          fix.longitude,
          fix.altitude,
          fix.satellites,
          fix.hdop);
    } else {
      LOG_WARN("No GPS fix. Satellites: %d", fix.satellites);
    }
  }
}

void GPSService::_sendGPSCommand(const char* command, bool waitForAck, uint32_t timeoutMs)
{
  GPSCommand cmd;
  cmd.command = String(command);
  cmd.waitForAck = waitForAck;
  cmd.timeoutMs = timeoutMs;

  _commandQueue.push_back(cmd);

  // If not currently waiting for an ACK, process the queue immediately
  if (!_waitingForAck) {
    _processCommandQueue();
  }
}

void GPSService::_processCommandQueue()
{
  // If waiting for ACK, don't send next command
  if (_waitingForAck) {
    // If there is unread serial, give loopHandler a chance to parse the ACK first
    if ((_gpsSerial.available() == 0) && (millis() - _ackWaitStartMs > _currentAckTimeout)) {
      LOG_WARN("ACK timeout for command: %s", _lastSentCommand.c_str());
      _waitingForAck = false;  // Continue despite timeout
    }
    return;
  }

  // Process next command in queue (unchanged)
  if (!_commandQueue.empty()) {
    GPSCommand cmd = _commandQueue.front();
    _commandQueue.erase(_commandQueue.begin());

    _gpsSerial.println(cmd.command);
    if (cmd.waitForAck) {
      LOG_INFO("GPS TX (expecting ACK): %s", cmd.command.c_str());
      _waitingForAck = true;
      _ackWaitStartMs = millis();
      _currentAckTimeout = cmd.timeoutMs;
      _lastSentCommand = cmd.command;
    } else {
      LOG_INFO("GPS TX (no ACK expected): %s", cmd.command.c_str());
    }
  }
}

void GPSService::_checkForAck(const char* nmeaSentence)
{
  // PMTK001 format variations:
  // Standard: $PMTK001,Cmd,Flag*CS
  // Extended (e.g., PMTK353): $PMTK001,Cmd,Flag,<echo of command params>,Result*CS
  // Flag/Result: 0=Invalid, 1=Unsupported, 2=Valid but failed, 3=Success

  if (!_waitingForAck) {
    return;
  }

  // Parse the PMTK001 response
  String sentence = String(nmeaSentence);
  int firstComma = sentence.indexOf(',');
  int secondComma = sentence.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0) {
    return;  // Invalid format
  }

  // Extract command ID
  String cmdStr = sentence.substring(firstComma + 1, secondComma);

  // Find the flag/result field
  String remaining = sentence.substring(secondComma + 1);
  int starPos = remaining.indexOf('*');
  if (starPos > 0) {
    remaining = remaining.substring(0, starPos);
  }

  int commaCount = 0;
  for (int i = 0; i < remaining.length(); i++) {
    if (remaining[i] == ',') commaCount++;
  }

  int flag = -1;
  if (commaCount == 0) {
    // Standard format: just the flag value
    flag = remaining.toInt();
  } else {
    // Extended format: might have echoed parameters
    int thirdComma = remaining.indexOf(',');
    if (thirdComma > 0) {
      // Immediate flag field
      String flagStr = remaining.substring(0, thirdComma);
      flag = flagStr.toInt();

      // Some commands also have a result field at the end (e.g., PMTK353)
      if (cmdStr == "353") {
        int lastComma = remaining.lastIndexOf(',');
        if (lastComma > 0) {
          String resultStr = remaining.substring(lastComma + 1);
          flag = resultStr.toInt();
        }
      }
    } else {
      flag = remaining.toInt();
    }
  }

  // Extract command ID from last sent command (e.g., "721" from "$PMTK721,...")
  String lastCmdId = "";
  if (_lastSentCommand.startsWith("$PMTK")) {
    int cmdEnd = _lastSentCommand.indexOf(',');
    if (cmdEnd < 0) cmdEnd = _lastSentCommand.indexOf('*');
    if (cmdEnd > 5) {
      lastCmdId = _lastSentCommand.substring(5, cmdEnd);
    }
  }

  // Check if this ACK matches our last command
  if (cmdStr == lastCmdId) {
    _waitingForAck = false;

    if (flag == -1) {
      LOG_ERROR("Failed to parse ACK flag from response: %s", nmeaSentence);
    } else {
      switch (flag) {
        case 3:
          LOG_INFO("Command %s acknowledged successfully (flag=%d)", cmdStr.c_str(), flag);
          break;
        case 2:
          LOG_WARN("Command %s valid but action failed (flag=%d)", cmdStr.c_str(), flag);
          break;
        case 1:
          LOG_WARN("Command %s not supported (flag=%d)", cmdStr.c_str(), flag);
          break;
        case 0:
          LOG_ERROR("Command %s invalid (flag=%d)", cmdStr.c_str(), flag);
          break;
        default:
          LOG_WARN("Command %s unknown ACK flag: %d", cmdStr.c_str(), flag);
      }
    }

    // Process next command in queue
    _processCommandQueue();
  } else {
    // Debug log for unexpected ACK
    LOG_WARN("Received ACK for command %s but was expecting %s", cmdStr.c_str(), lastCmdId.c_str());
  }
}