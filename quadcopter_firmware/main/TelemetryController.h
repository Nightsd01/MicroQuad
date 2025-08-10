#pragma once

#ifndef MATLAB_SIM

#include <map>

#include "BLEController.h"
#include "TelemetryEvent.h"

struct _telem_event_t
{
  TelemetryEvent event;
  void *data;
  size_t size;
};

// defines the min spacing between transmissions
// if too many events hit this telem controller it will
// start dropping them and will log errors
#define MIN_TRANSMISSION_WINDOW_MILLIS 30

#define MAX_TELEM_PACKET_SIZE 19

// How many telemetry events can trigger a warning
#define QUEUE_WARNING_SIZE 100

// At what queue size should we start dropping events
#define QUEUE_ERROR_DROP_SIZE 1000

// Telemetry controller batches telemetry updates and sends them over
// L2CAP CoC to the iOS application at a fixed cadence.
class TelemetryController
{
 public:
  TelemetryController(BLEController *controller);

  // Data must be <= MAX_TELEM_PACKET_SIZE bytes in size
  void updateTelemetryEvent(TelemetryEvent event, void *data, size_t size);

  void loopHandler(void);

  bool disableTransmission = false;

 private:
  BLEController *_bleController;
  // Latest telemetry entries per event type
  std::map<TelemetryEvent, _telem_event_t> _eventMap;
  uint64_t _eventCount;
  uint64_t _lastTransmissionMillis = 0;
};

#endif  // MATLAB_SIM