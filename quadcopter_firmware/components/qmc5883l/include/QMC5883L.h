#pragma once

#include <Wire.h>

#include <functional>

#include "esp_err.h"

struct mag_update_t
{
  float x, y, z;
  float heading;
};

struct mag_calibration_offsets_t
{
  esp_err_t error;
  float x_offset;
  float y_offset;
  float z_offset;
};

class QMC5883L
{
 public:
  QMC5883L(void);
  ~QMC5883L();

  enum class Range
  {
    GAUSS_2,
    GAUSS_8
  };

  enum class MeasurementMode
  {
    STANDBY,
    CONTINUOUS
  };

  enum class DataRate
  {
    HZ10,
    HZ50,
    HZ100,
    HZ200
  };

  enum class Samples
  {
    OSR512,
    OSR256,
    OSR128,
    OSR64
  };

  esp_err_t begin(TwoWire &wire, std::function<void(mag_update_t)> updateHandler);
  void loopHandler(void);

  void setRange(Range range);
  void setMeasurementMode(MeasurementMode mode);
  void setDataRate(DataRate dataRate);
  void setSamples(Samples samples);
  void setDeclinationAngle(float declinationAngle);
  void setXOffset(float xOffset);
  void setYOffset(float yOffset);
  void setZOffset(float zOffset);

  /**
   * @brief Perform a calibration by collecting samples.
   * Rotate the sensor in all directions during the calibration period.
   * @param sampleCount Number of samples to collect.
   * @param sampleDelayMs Delay in milliseconds between samples.
   * @return mag_calibration_offsets_t with error = ESP_OK on success, ESP_FAIL on failure.
   */
  mag_calibration_offsets_t calibrate(size_t sampleCount = 500, uint32_t sampleDelayMs = 20);

 private:
  class QMC5883LPrivate;
  QMC5883LPrivate *_p;
};
