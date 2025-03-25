#include "Barometer.h"

#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Logger.h>
#include <Wire.h>

static float _altMeters(const float pressureHpa) { return 44330.0f * (1.0f - powf((pressureHpa / 1013.25f), 0.1903f)); }

Barometer::Barometer() : _isInitialized(false), _referenceAltitude(0.0f) {}

bool Barometer::begin(std::function<void(float)> altitudeHandler, uint8_t address, TwoWire *theWire)
{
  if (!_bme.begin(address, theWire)) {
    LOG_ERROR("Could not find a valid BME280 sensor, check wiring!");
    return false;
  }

  _altitudeHandler = altitudeHandler;

  LOG_INFO("Connected to barometer - BME280 sensor");

  const float pressureHpa = _bme.readPressure() / 100.0f;
  _referenceAltitude = _altMeters(pressureHpa);

  LOG_INFO("BME280 reference altitude set to %.2fm", _referenceAltitude);
  _isInitialized = true;
  return true;
}

void Barometer::loopHandler(void)
{
  if (!_isInitialized) {
    return;
  }
  if (millis() - _lastUpdateTimestampMillis < 1000.0f / DATA_UPDATE_RATE_HZ) {
    return;
  }
  _lastUpdateTimestampMillis = millis();

  const float pressureHpa = _bme.readPressure() / 100.0f;
  // Current altitude
  const float currentAltMeters = _altMeters(pressureHpa);
  // Return difference from reference altitude
  const float deltaAltMeters = (currentAltMeters - _referenceAltitude);

  _medianFilter.addValue(deltaAltMeters);

  if (_numReferenceSamples < NUM_REFERENCE_SAMPLES) {
    _referenceAltitudeMedianFilter.addValue(currentAltMeters);
    _numReferenceSamples++;
    _referenceAltitude = _referenceAltitudeMedianFilter.getMedian();
  }

  if (_altitudeHandler) {
    _altitudeHandler(_medianFilter.getMedian());
  }
}
