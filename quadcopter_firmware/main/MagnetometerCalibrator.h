#pragma once

#ifndef MATLAB_SIM

#include <Matrix.h>  // Include the provided Matrix class definition

#include <cmath>
#include <functional>
#include <stdexcept>  // Used for potential exceptions
#include <vector>     // Still needed for std::vector to store measurements

#include "PersistentKeyValueStore.h"

using Vector3float = Matrix<float, 3, 1>;
using Matrix3x3float = Matrix<float, 3, 3>;

/**
 * @brief Performs Hard and Soft Iron Calibration for a magnetometer.
 *
 * Collects raw magnetometer readings (expected in Gauss) while the sensor is
 * rotated through diverse orientations. It then calculates the hard iron
 * offset (bias) and soft iron distortion matrix using ellipsoid fitting.
 * This version uses the provided Matrix class template for vector/matrix operations.
 */
class MagnetometerCalibrator
{
 public:
  /**
   * @brief Configuration settings for the calibration process.
   */
  struct Config
  {
    /** @brief Minimum number of data points required before attempting calibration calculation. */
    size_t min_points = 200;  // Adjust based on testing and algorithm needs

    /** @brief Maximum number of data points to store. Helps limit memory usage. 0 for unlimited. */
    size_t max_points = 1000;

    int num_seconds_for_calibration = 30;  // Duration to collect data before calibration

    /** @brief (Optional) Expected local magnetic field magnitude in Gauss.
     * Can be used for validation or scaling after fitting, though many
     * algorithms determine scale implicitly. Approximately 0.49 G for
     * Foster City, CA (as of April 2025).
     * Set to 0 or negative to ignore. */
    float expected_field_magnitude_gauss = 0.49f;
  };

  /**
   * @brief Constructor initializing the calibrator.
   * @param config Configuration parameters.
   */
  MagnetometerCalibrator(const Config& config, PersistentKeyValueStore* kvStore);

  /**
   * @brief Called to begin calibration
   * @param config Configuration parameters.
   */
  void startCalibration(std::function<void(bool)> calibrationCompleteCallback);

  /**
   * @brief Adds a new raw magnetometer measurement to the internal buffer.
   *
   * Call this repeatedly while rotating the sensor through many different
   * orientations covering all axes (+x, -x, +y, -y, +z, -z directions).
   *
   * @param raw_measurement A Vector3float (Matrix<float, 3, 1>) containing the
   * raw magnetometer reading (x, y, z) in Gauss.
   */
  void addMeasurement(const Vector3float& raw_measurement);

  /**
   * @brief Checks if the calibration has been successfully completed.
   * @return true if `calculateCalibration` has run successfully at least once,
   * false otherwise.
   */
  bool isCalibrationComplete() const;

  /**
   * @brief Retrieves the calculated hard iron offset vector (bias).
   * @return Vector3float (Matrix<float, 3, 1>) The hard iron offset in Gauss.
   * Returns a zero vector if calibration is not complete.
   */
  Vector3float getHardIronOffset() const;

  /**
   * @brief Retrieves the calculated soft iron correction matrix.
   * This is the matrix 'S' used in the correction formula:
   * m_corrected = S * (m_raw - hard_iron_offset)
   *
   * @return Matrix3x3float (Matrix<float, 3, 3>) The soft iron correction matrix.
   * Returns an identity matrix if calibration is not complete.
   */
  Matrix3x3float getSoftIronMatrix() const;

  /**
   * @brief Applies the calculated calibration parameters to a raw measurement.
   *
   * Corrects a raw reading for both hard and soft iron distortions using
   * the stored calibration parameters and Matrix class operations.
   * Requires that the Matrix class overloads operator* for matrix-vector
   * multiplication and operator- for vector subtraction.
   *
   * @param raw_measurement The raw magnetometer reading (Vector3float/Matrix<float, 3, 1>) in Gauss.
   * @return Vector3float (Matrix<float, 3, 1>) The calibrated magnetometer reading in Gauss.
   * @throws std::runtime_error if `isCalibrationComplete()` is false.
   */
  Vector3float applyCorrection(const Vector3float& raw_measurement) const;

  /**
   * @brief Resets the calibrator. Clears all collected measurements and
   * resets calibration status and parameters to default (zero offset,
   * identity matrix).
   */
  void reset();

  /**
   * @brief Gets the current number of measurements stored in the buffer.
   * @return size_t The count of measurements collected since the last reset.
   */
  size_t getMeasurementCount() const;

  /**
   * @brief Checks if the calibrator is currently in the calibration process.
   * @return true if calibration is in progress, false otherwise.
   */
  bool isCalibrating(void) const;

 private:
  /**
   * @brief Performs the core ellipsoid fitting algorithm.
   * This is where the complex numerical calculation happens, utilizing the Matrix class
   * operations like inversion, multiplication, etc.
   * @param measurements Vector of collected raw data points (Vector3float type).
   * @param[out] hard_iron Calculated hard iron offset (Vector3float type).
   * @param[out] soft_iron Calculated soft iron correction matrix (Matrix3x3float type).
   * @return true if fitting was successful, false otherwise.
   */
  bool solveEllipsoidFit(
      const std::vector<Vector3float>& measurements, Vector3float& hard_iron, Matrix3x3float& soft_iron);

  Config _config;                           // Stores configuration
  std::vector<Vector3float> _measurements;  // Buffer for collected raw readings
  Vector3float _hardIronOffset;             // Calculated hard iron offset (zeroed by default Matrix constructor)
  Matrix3x3float _softIronMatrix;           // Calculated soft iron matrix (initialized to identity in constructor)
  bool _calibrationComplete = false;        // Status flag
  PersistentKeyValueStore* _kvStore;        // Pointer to persistent key-value store for calibration data
  void _storeCalibrationData(void);         // Store calibration data in persistent storage
  std::function<void(bool)> _calibrationCompleteCallback;  // Callback to notify when calibration is complete
  bool _isCalibrationInProgress = false;                   // Flag to indicate if calibration is in progress
  uint64_t _calibrationStartTimeMillis = 0;                // Timestamp for when calibration started

  /**
   * @brief Attempts to calculate the hard and soft iron calibration parameters.
   *
   * This function should be called after sufficient data points with diverse
   * orientations have been collected using `addMeasurement`. It performs the
   * ellipsoid fitting algorithm using the Matrix class operations.
   *
   * @return true if the calibration calculation was successful and parameters
   * are updated, false otherwise (e.g., insufficient points,
   * poor data distribution leading to numerical instability).
   */
  bool _calculateCalibration();
};

#endif  // MATLAB_SIM