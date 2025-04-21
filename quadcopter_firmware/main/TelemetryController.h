#pragma once

#ifndef MATLAB_SIM

#include <deque>

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

// Bluetooth packets must be limited to 20 bytes in iOS
// This controller makes it easier to send telemetry updates
// to the controller app
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
  std::deque<_telem_event_t> _eventQueue;
  bool _waitingForTransmission = false;
  uint64_t _eventCount;
};

#endif  // MATLAB_SIM