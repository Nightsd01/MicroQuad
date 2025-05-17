#pragma once

#ifndef MATLAB_SIM

// Assume these headers provide necessary types and constants:
#include <QMC5883L.h>              // Provides mag_update_t {float x, y, z, heading;}
#include <QuadcopterController.h>  // Provides motor_outputs_t = std::array<float, NUM_MOTORS>, NUM_MOTORS

#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <string>
#include <vector>

#include "Constants.h"  // Provides THROTTLE_MIN, THROTTLE_MAX, DEG_TO_RADS, PI, DECLINATION_ANGLE_DEG
#include "PersistentKeyValueStore.h"

// Forward declaration
class PersistentKeyValueStore;

#define CALIBRATION_MOTOR_STEPS 10
#define CALIBRATION_STEP_TIME_MS 1000
#define SPINUP_MS 200  // warm‑up time

// Helper struct to accumulate magnetometer readings during calibration steps
struct MagSum
{
  float x{0}, y{0}, z{0};
  uint32_t n{0};

  void accumulate(const mag_update_t &m)
  {
    x += m.x;
    y += m.y;
    z += m.z;
    ++n;
  }

  void reset()
  {
    x = y = z = 0;
    n = 0;
  }

  // Calculate the mean magnetic field vector for the accumulated samples
  mag_update_t meanMag() const
  {
    const float inv = (n > 0 ? 1.0f / static_cast<float>(n) : 1.0f);
    mag_update_t m = {x * inv, y * inv, z * inv, 0};
    // Calculate heading based on mean X and Y - useful for seeing calibration effect
    m.heading = std::atan2(-m.y, m.x);  // Assuming NED frame and sensor orientation
    return m;
  }
};

class MotorMagCompensationHandler
{
 public:
  /**
   * @brief Constructor. Loads calibration data from persistent storage if available.
   * @param kvStore Pointer to the key-value store interface for loading/saving calibration.
   */
  MotorMagCompensationHandler(PersistentKeyValueStore *kvStore);

  /**
   * @brief Indicates if the calibration routine is currently running.
   */
  bool isCalibrating;

  /**
   * @brief Indicates if valid calibration data has been loaded or generated.
   */
  bool isCalibrated;

  /**
   * @brief Starts the magnetometer calibration process.
   * Ramps all motors together through defined steps.
   * @param motorOutputCallback Function to call to set motor throttle values.
   * @param calibrationCompleteCallback Function called when calibration finishes (true=success, false=failure).
   */
  void beginCalibration(
      std::function<void(motor_outputs_t)> motorOutputCallback, std::function<void(bool)> calibrationCompleteCallback);

  /**
   * @brief Processes a single magnetometer reading during the calibration phase.
   * This should be called repeatedly with new sensor data while isCalibrating is true.
   * @param mag The latest magnetometer reading (raw).
   */
  void updateMagValue(const mag_update_t &mag);

  /**
   * @brief Applies the stored magnetic compensation based on current motor throttles.
   * @param rawMagData The raw magnetometer reading.
   * @param motorThrottles Current throttle values for each motor (typically 0.0 to 1.0 or similar range).
   * @return mag_update_t The compensated magnetometer reading with heading recalculated.
   */
  mag_update_t applyMagneticMotorCompensation(const mag_update_t &rawMagData, const motor_outputs_t &motorThrottles);

 private:
  PersistentKeyValueStore *_kvStore;
  std::function<void(motor_outputs_t)> _motorOutputCallback;
  std::function<void(bool)> _calibrationCompleteCallback;

  mag_update_t _baselineMag;
  bool _baselineCaptured;  // To track if baseline is done
  uint32_t _lastStepTs = 0;
  int _curStep = 0;
  MagSum _stepAcc;  // Accumulator for the current calibration step
  uint32_t _spinUpStartTs = 0;
  bool _inSpinUp = false;

  // Stores the measured magnetic field bias (offset) for each throttle step.
  std::array<mag_update_t, CALIBRATION_MOTOR_STEPS> _calibrationData;

  /**
   * @brief Called internally when all calibration steps are completed. Saves data.
   */
  void _completeCalibration(void);

  /**
   * @brief Generates the key used for storing/retrieving calibration data for a specific step.
   * @param step The calibration step index (0 to CALIBRATION_MOTOR_STEPS - 1).
   * @return std::string The key for the persistent store.
   */
  std::string _getKey(int step);

  // Static helper functions (can be moved outside class if preferred)
  static float mapRange(float value, float inMin, float inMax, float outMin, float outMax);
  static mag_update_t lerpMag(
      const mag_update_t &a, const mag_update_t &b, float t);  // Linear interpolation for mag vectors
  static float _headingFromXY(float x, float y);               // Calculates heading in degrees from X, Y components
};

#endif  // MATLAB_SIM