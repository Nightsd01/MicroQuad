#include "TelemetryController.h"

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <Logger.h>

#include <vector>

#include "BLEControllerLargeDataTransmissionHandler.h"
#include "BLELargeDataBlobType.h"
static const char *_eventTypeToString(TelemetryEvent event)
{
  switch (event) {
    case ArmStatusChange:
      return "ArmStatusChange";
    case BatteryStatusUpdate:
      return "BatteryStatusUpdate";
    case EulerYawPitchRoll:
      return "EulerYawPitchRoll";
    case AccelerometerXYZRaw:
      return "AccelerometerXYZRaw";
    case AccelerometerXYZFiltered:
      return "AccelerometerXYZFiltered";
    case GyroXYZRaw:
      return "GyroXYZRaw";
    case GyroXYZFiltered:
      return "GyroXYZFiltered";
    case MemoryStats:
      return "MemoryStats";
    case MotorValues:
      return "MotorValues";
    case LoopUpdateRate:
      return "LoopUpdateRate";
    case IMUUpdateRate:
      return "IMUUpdateRate";
    case MagnetometerXYZRaw:
      return "MagnetometerXYZRaw";
    case VL53L1XRawDistance:
      return "VL53L1XRawDistance";
    case VL53L1XEstimatedAltitudeUpdate:
      return "VL53L1XEstimatedAltitudeUpdate";
    case EKFAltitudeEstimate:
      return "EKFAltitudeEstimate";
    case EKFVerticalVelocityEstimate:
      return "EKFVerticalVelocityEstimate";
    case GPSFixData:
      return "GPSFixData";
    default: {
      LOG_ERROR_PERIODIC_MILLIS(500, "Unknown telemetry event %i", event);
      return "Unknown";
    }
  }
}

TelemetryController::TelemetryController(BLEController *controller)
{
  _bleController = controller;
  _eventCount = 0;
  _lastTransmissionMillis = 0;
}

// telem packet structure:
// [0] - event type
// [1:] - data
void TelemetryController::updateTelemetryEvent(TelemetryEvent event, void *data, size_t size)
{
  if (disableTransmission) {
    return;
  }

  uint8_t *copy = (uint8_t *)malloc(size + 1);
  if (copy == NULL) {
    LOG_ERROR_PERIODIC_MILLIS(500, "Failed to allocate memory for telemetry event %s", _eventTypeToString(event));
    return;
  }

  // Store just the raw payload. We'll serialize event id during batch send
  memcpy(copy, data, size);

  // If an entry already exists for this event, free its memory before replacing
  auto it = _eventMap.find(event);
  if (it != _eventMap.end()) {
    if (it->second.data) {
      free(it->second.data);
    }
  }

  _eventMap[event] = {event, (void *)copy, size};
  _eventCount++;
}

void TelemetryController::loopHandler(void)
{
  // Transmit at most once every 30 ms
  static const uint64_t kBatchIntervalMs = 30;
  if (!_bleController->isConnected || _eventMap.empty()) {
    return;
  }

  if (millis() - _lastTransmissionMillis < kBatchIntervalMs) {
    return;
  }

  // Use the dedicated telemetry L2CAP channel (smaller MTU, lower latency)
  auto *l2cap = _bleController->getTelemetryL2CAPHandler();
  if (!l2cap || !l2cap->isReadyToSend()) {
    return;
  }

  // Serialize all events into a single buffer: [event,u16 size,bytes] x N
  std::vector<uint8_t> buffer;
  buffer.reserve(256);

  for (auto &kv : _eventMap) {
    const TelemetryEvent evType = kv.first;
    const _telem_event_t &ev = kv.second;

    // Event id
    buffer.push_back(static_cast<uint8_t>(evType));

    // Size (little-endian 16-bit). Clamp to 65535 if larger.
    uint16_t sz = static_cast<uint16_t>(ev.size > 0xFFFF ? 0xFFFF : ev.size);
    buffer.push_back(static_cast<uint8_t>(sz & 0xFF));
    buffer.push_back(static_cast<uint8_t>((sz >> 8) & 0xFF));

    // Data
    uint8_t *bytes = static_cast<uint8_t *>(ev.data);
    buffer.insert(buffer.end(), bytes, bytes + sz);
  }

  // Send as a single L2CAP CoC transaction with TelemetryData type
  if (!buffer.empty()) {
    bool ok = l2cap->sendData(BLELargeDataBlobType::TelemetryData, buffer.data(), buffer.size());
    if (!ok) {
      LOG_ERROR("Failed to send telemetry batch over L2CAP; dropping batch to avoid memory growth");
      // Free stored entries and clear the map to avoid memory growth on repeated failures
      for (auto &kv : _eventMap) {
        if (kv.second.data) {
          free(kv.second.data);
        }
      }
      _eventMap.clear();
      _lastTransmissionMillis = millis();
      return;
    }
  }

  // Free stored entries and clear the map
  for (auto &kv : _eventMap) {
    if (kv.second.data) {
      free(kv.second.data);
    }
  }
  _eventMap.clear();
  _lastTransmissionMillis = millis();
}

#endif  // MATLAB_SIM