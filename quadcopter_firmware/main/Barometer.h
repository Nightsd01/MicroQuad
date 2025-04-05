#pragma once

#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Wire.h>

#include <functional>

#include "MedianFilter.h"

class Adafruit_BME280;
class TelemetryController;

#define DATA_UPDATE_RATE_HZ 100.0f

#define MEDIAN_FILTER_SIZE 10

#define NUM_REFERENCE_SAMPLES 200

class Barometer
{
 public:
  Barometer();
  bool begin(
      std::function<void(float)> altitudeHandler,
      TelemetryController *telemController,
      uint8_t address = 0x76,
      TwoWire *theWire = &Wire);
  void loopHandler(void);

 private:
  Adafruit_BME280 _bme;
  TelemetryController *_telemController;
  MedianFilter<float> _medianFilter = MedianFilter<float>(MEDIAN_FILTER_SIZE);
  MedianFilter<float> _referenceAltitudeMedianFilter = MedianFilter<float>(MEDIAN_FILTER_SIZE);
  int _numReferenceSamples;
  bool _isInitialized;
  float _referenceAltitude;  // Altitude measured at initialization (meters)
  std::function<void(float)> _altitudeHandler;
  uint64_t _lastUpdateTimestampMillis;
};