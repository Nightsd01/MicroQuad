#include "TelemetryController.h"

#ifndef MATLAB_SIM

#include <Logger.h>

static const char *_eventTypeToString(TelemetryEvent event)
{
  switch (event) {
    case ArmStatusChange:
      return "ArmStatusChange";
    case BatteryVoltage:
      return "BatteryVoltage";
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
    default: {
      LOG_ERROR_PERIODIC_MILLIS(500, "Unknown telemetry event %i", event);
      return "Unknown";
    }
  }
}

TelemetryController::TelemetryController(BLEController *controller) { _bleController = controller; }

// telem packet structure:
// [0] - event type
// [1:] - data
void TelemetryController::updateTelemetryEvent(TelemetryEvent event, void *data, size_t size)
{
  if (!_bleController->isConnected || disableTransmission) {
    return;
  }

  if (size > MAX_TELEM_PACKET_SIZE) {
    LOG_ERROR_PERIODIC_MILLIS(500, "Telemetry packet/s size too large for event %s", _eventTypeToString(event));
    return;
  }

  uint8_t *copy = (uint8_t *)malloc(size + 1);
  if (copy == NULL) {
    LOG_ERROR_PERIODIC_MILLIS(500, "Failed to allocate memory for telemetry event %s", _eventTypeToString(event));
    return;
  }

  memcpy(copy + 1, data, size);
  *copy = (uint8_t)event;
  _eventQueue.push_back({event, copy, size});
  _eventCount++;

  if (_eventQueue.size() > QUEUE_WARNING_SIZE) {
    LOG_WARN_PERIODIC_MILLIS(500, "Telemetry queue size is growing too large: %i events", _eventQueue.size());
  }

  if (_eventQueue.size() > QUEUE_ERROR_DROP_SIZE) {
    LOG_ERROR_PERIODIC_MILLIS(500, "Telemetry queue size is too large, dropping events");
    _telem_event_t event = _eventQueue[0];
    _eventQueue.pop_front();
    free(event.data);
  }
}

void TelemetryController::loopHandler(void)
{
  if (!_bleController->isConnected || _eventQueue.size() == 0) {
    return;
  }

  if (!_waitingForTransmission) {
    _telem_event_t event = _eventQueue[0];
    uint8_t *data = (uint8_t *)event.data;

    _bleController->setTelemetryTransmissionCompleteHandler([this, data]() {
      _eventQueue.pop_front();
      _waitingForTransmission = false;
      free(data);
    });

    _bleController->sendTelemetryUpdate(data, event.size + 1);
  }
}

#endif  // MATLAB_SIM