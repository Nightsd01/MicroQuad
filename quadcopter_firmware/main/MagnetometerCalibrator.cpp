
#ifndef MATLAB_SIM

#include "MagnetometerCalibrator.h"  // Include the header file defining the class

#include <Arduino.h>  // For Arduino-specific functions like millis()
#include <Logger.h>

#include <cmath>
#include <limits>  // Required for numeric limits, e.g., epsilon
#include <stdexcept>
#include <vector>  // Still needed for storing measurements

#include "Matrix.h"  // Include the FIXED-SIZE Matrix class header
#include "PersistentKeysCommon.h"

// Type aliases using the provided fixed-size Matrix template
using Vector3float = Matrix<float, 3, 1>;
using Matrix3x3float = Matrix<float, 3, 3>;
using Matrix9x9float = Matrix<float, 9, 9>;
using Vector9float = Matrix<float, 9, 1>;
using RowVector3float = Matrix<float, 1, 3>;  // For transpose results

// --- Helper Function: Eigenvalue Decomposition for 3x3 Symmetric Matrix ---
// Uses the Jacobi rotation method. Compatible with the fixed-size Matrix class.
// Calculates eigenvalues and eigenvectors for a symmetric 3x3 matrix.
// A = V * D * V^T, where D is diagonal (eigenvalues), V has eigenvectors as columns.
// Returns true on success, false on failure (e.g., exceeds iterations).
bool eigenDecompositionSym3x3(const Matrix3x3float& A, Vector3float& eigenvalues, Matrix3x3float& eigenvectors)
{
  const int MAX_ITERATIONS = 50;
  // Use a slightly larger epsilon than machine epsilon for float stability
  const float EPSILON = 1e-8f;

  eigenvectors = Matrix3x3float::identity();  // Initialize eigenvectors to identity
  Matrix3x3float D = A;                       // Copy A to D, we will diagonalize D

  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    // Find the largest off-diagonal element |D(p,q)|
    float max_off_diag = 0.0f;
    int p = 0, q = 1;
    for (int i = 0; i < 3; ++i) {
      for (int j = i + 1; j < 3; ++j) {
        if (std::fabs(D(i, j)) > max_off_diag) {
          max_off_diag = std::fabs(D(i, j));
          p = i;
          q = j;
        }
      }
    }

    // Check for convergence: sum of absolute values of off-diagonal elements
    float sum_off_diag = std::fabs(D(0, 1)) + std::fabs(D(0, 2)) + std::fabs(D(1, 2));
    if (sum_off_diag < EPSILON) {
      // Convergence achieved
      eigenvalues(0, 0) = D(0, 0);
      eigenvalues(1, 0) = D(1, 1);
      eigenvalues(2, 0) = D(2, 2);
      // Eigenvectors are already accumulated in V
      return true;
    }

    // Calculate rotation angle theta using numerically stable method
    float app = D(p, p);
    float aqq = D(q, q);
    float apq = D(p, q);
    float diff = aqq - app;
    float t;  // tan(theta)

    if (std::fabs(apq) < EPSILON) {
      t = 0.0f;  // No rotation needed if off-diagonal is already ~zero
    } else {
      float tau = diff / (2.0f * apq);
      // Avoid cancellation if tau is large
      if (std::fabs(tau) > 1.0f / EPSILON) {
        t = 1.0f / (2.0f * tau);  // Approximation for small t
      } else {
        t = (tau >= 0.0f) ? (1.0f / (tau + std::sqrt(1.0f + tau * tau)))
                          : (-1.0f / (-tau + std::sqrt(1.0f + tau * tau)));
      }
    }

    float c = 1.0f / std::sqrt(1.0f + t * t);  // cos(theta)
    float s = t * c;                           // sin(theta)

    // Construct rotation matrix R (implicitly, apply update directly)
    // Update D = R^T * D * R (optimized update)
    Matrix3x3float D_old = D;  // Temporary copy
    D(p, p) = c * c * D_old(p, p) + s * s * D_old(q, q) + 2.0f * c * s * D_old(p, q);
    D(q, q) = s * s * D_old(p, p) + c * c * D_old(q, q) - 2.0f * c * s * D_old(p, q);
    D(p, q) = 0.0f;  // Zero out the target element
    D(q, p) = 0.0f;

    // Update other elements affected by rotation
    for (int k = 0; k < 3; ++k) {
      if (k != p && k != q) {
        float dpk = D_old(k, p);  // Use k,p notation for consistency
        float dqk = D_old(k, q);
        D(k, p) = c * dpk + s * dqk;
        D(k, q) = -s * dpk + c * dqk;
        D(p, k) = D(k, p);  // Maintain symmetry
        D(q, k) = D(k, q);
      }
    }

    // Update V = V * R
    Matrix3x3float V_old = eigenvectors;
    for (int k = 0; k < 3; ++k) {
      float vpk = V_old(k, p);
      float vqk = V_old(k, q);
      eigenvectors(k, p) = c * vpk + s * vqk;
      eigenvectors(k, q) = -s * vpk + c * vqk;
    }
  }

  // Failed to converge within MAX_ITERATIONS
  // Store diagonal elements as approximate eigenvalues
  eigenvalues(0, 0) = D(0, 0);
  eigenvalues(1, 0) = D(1, 1);
  eigenvalues(2, 0) = D(2, 2);
  return false;
}

// --- MagnetometerCalibrator Class Implementation ---

MagnetometerCalibrator::MagnetometerCalibrator(const Config& config, PersistentKeyValueStore* kvStore)
    : _config(config),
      _hardIronOffset(0.0f),                        // Use Matrix constructor that fills with value
      _softIronMatrix(Matrix3x3float::identity()),  // Use static identity method
      _calibrationComplete(false),
      _kvStore(kvStore)
{
  if (kvStore->hasValueForKey(PersistentKeysCommon::MAG_HARD_IRON_OFFSETS) &&
      kvStore->hasValueForKey(PersistentKeysCommon::MAG_SOFT_IRON_OFFSETS)) {
    // Load hard and soft iron offsets from persistent storage
    std::vector<float> hardIronOffsets =
        kvStore->getVectorForKey<float>(PersistentKeysCommon::MAG_HARD_IRON_OFFSETS, 3);
    std::vector<float> softIronMatrix = kvStore->getVectorForKey<float>(PersistentKeysCommon::MAG_SOFT_IRON_OFFSETS, 9);

    if (hardIronOffsets.size() == 3 && softIronMatrix.size() == 9) {
      LOG_INFO("Hard iron offsets: %f, %f, %f", hardIronOffsets[0], hardIronOffsets[1], hardIronOffsets[2]);
      LOG_INFO(
          "Loaded magnetometer calibration from persistent storage, hard iron matrix = %s, soft iron matrix = %s",
          Vector3float(hardIronOffsets.data()).description().c_str(),
          Matrix3x3float(softIronMatrix.data()).description().c_str());
      _hardIronOffset = Vector3float(hardIronOffsets.data());
      _softIronMatrix = Matrix3x3float(softIronMatrix.data());
      _calibrationComplete = true;
    } else {
      LOG_ERROR("Invalid calibration data in persistent storage");
    }
  } else {
    LOG_WARN("No magnetometer calibration data - please calibrate the compass");
  }
  // Reserve memory if max_points is set and > 0
  if (_config.max_points > 0) {
    _measurements.reserve(_config.max_points);
  }
}

void MagnetometerCalibrator::startCalibration(std::function<void(bool)> calibrationCompleteCallback)
{
  _calibrationCompleteCallback = calibrationCompleteCallback;
  _isCalibrationInProgress = true;
  _calibrationStartTimeMillis = millis();  // Store the start time
}

bool MagnetometerCalibrator::isCalibrating(void) const { return _isCalibrationInProgress; }

void MagnetometerCalibrator::_storeCalibrationData(void)
{
  if (!_calibrationComplete) {
    LOG_ERROR("Calibration not complete, cannot store data");
    return;
  }
  constexpr size_t hardIronSize = 3;
  constexpr size_t softIronSize = 9;

  std::vector<float> hardIronOffsets(_hardIronOffset.data, _hardIronOffset.data + hardIronSize);
  _kvStore->setVectorForKey<float>(PersistentKeysCommon::MAG_HARD_IRON_OFFSETS, hardIronOffsets);

  std::vector<float> softIronVector(_softIronMatrix.data, _softIronMatrix.data + softIronSize);
  _kvStore->setVectorForKey<float>(
      PersistentKeysCommon::MAG_SOFT_IRON_OFFSETS,
      softIronVector);  // Store as flat vector
}

void MagnetometerCalibrator::addMeasurement(const Vector3float& raw_measurement)
{
  if (_config.max_points > 0 && _measurements.size() >= _config.max_points) {
    // Remove the oldest measurement to make space (FIFO)
    _measurements.erase(_measurements.begin());
  }
  _measurements.push_back(raw_measurement);

  if (millis() - _calibrationStartTimeMillis > _config.num_seconds_for_calibration * 1000) {
    if (_calculateCalibration()) {
      _calibrationCompleteCallback(true);
      _isCalibrationInProgress = false;
      _calibrationComplete = true;
    } else {
      _calibrationCompleteCallback(false);
      _isCalibrationInProgress = false;
      _calibrationComplete = false;
    }
  }
}

bool MagnetometerCalibrator::_calculateCalibration()
{
  if (_measurements.size() < _config.min_points) {
    // Not enough data points
    return false;
  }

  // Ensure minimum points is reasonable for the 9 unknowns
  if (_measurements.size() < 10) {
    return false;
  }

  Vector3float hard_iron_temp;
  Matrix3x3float soft_iron_temp;

  if (solveEllipsoidFit(_measurements, hard_iron_temp, soft_iron_temp)) {
    _hardIronOffset = hard_iron_temp;
    _softIronMatrix = soft_iron_temp;
    _calibrationComplete = true;
    return true;
  } else {
    // Calculation failed, reset to defaults to avoid using bad values
    // Consider logging an error here
    reset();
    return false;
  }
}

bool MagnetometerCalibrator::isCalibrationComplete() const { return _calibrationComplete; }

Vector3float MagnetometerCalibrator::getHardIronOffset() const { return _hardIronOffset; }

Matrix3x3float MagnetometerCalibrator::getSoftIronMatrix() const { return _softIronMatrix; }

Vector3float MagnetometerCalibrator::applyCorrection(const Vector3float& raw_measurement) const
{
  if (!_calibrationComplete) {
    // Optionally return raw_measurement instead of throwing?
    // throw std::runtime_error("Magnetometer calibration is not complete. Cannot apply correction.");
    // For embedded systems, maybe avoid exceptions:
    // Return raw if not calibrated, or signal error differently.
    // Let's stick to the original design for now:
    throw std::runtime_error("Magnetometer calibration is not complete. Cannot apply correction.");
  }
  // Assumes Matrix class overloads operator* for Matrix * Vector
  // and operator- for Vector - Vector
  return _softIronMatrix * (raw_measurement - _hardIronOffset);
}

void MagnetometerCalibrator::reset()
{
  _measurements.clear();
  // Re-reserve memory if needed
  if (_config.max_points > 0) {
    _measurements.reserve(_config.max_points);
  }
  _hardIronOffset.zeros();                       // Use Matrix class zeroing method
  _softIronMatrix = Matrix3x3float::identity();  // Reset to identity
  _calibrationComplete = false;
}

size_t MagnetometerCalibrator::getMeasurementCount() const { return _measurements.size(); }

// --- Private Methods ---

bool MagnetometerCalibrator::solveEllipsoidFit(
    const std::vector<Vector3float>& measurements, Vector3float& hard_iron, Matrix3x3float& soft_iron)
{
  size_t N = measurements.size();
  // Minimum points check already done in calculateCalibration, but double-check
  if (N < 10) return false;

  // --- Calculate D^T*D and D^T*1 directly ---
  Matrix9x9float DtD(0.0f);  // Initialize 9x9 matrix to zeros
  Vector9float Dt1(0.0f);    // Initialize 9x1 vector to zeros

  for (size_t i = 0; i < N; ++i) {
    float x = measurements[i](0, 0);
    float y = measurements[i](1, 0);
    float z = measurements[i](2, 0);

    // Create the current row of the conceptual D matrix
    float d_row[9] = {x * x, y * y, z * z, 2.0f * x * y, 2.0f * x * z, 2.0f * y * z, 2.0f * x, 2.0f * y, 2.0f * z};

    // Accumulate D^T * D (outer product of d_row with itself, summed over i)
    // DtD(j,k) = sum over i ( d_row[j] * d_row[k] )
    for (size_t j = 0; j < 9; ++j) {
      for (size_t k = 0; k < 9; ++k) {
        DtD(j, k) += d_row[j] * d_row[k];
      }
    }

    // Accumulate D^T * 1 (sum of d_row elements over i)
    // Dt1(j) = sum over i ( d_row[j] * 1 )
    for (size_t j = 0; j < 9; ++j) {
      Dt1(j, 0) += d_row[j];
    }
  }

  // --- Solve the system (D^T*D) * v = (D^T*1) ---

  // Check determinant before inversion (Requires determinant() in Matrix class)
  float det_DtD = DtD.determinant();
  if (std::fabs(det_DtD) < std::numeric_limits<float>::epsilon() * 100) {  // Add tolerance factor
    // Matrix is likely singular or ill-conditioned
    return false;
  }

  // Calculate inverse of DtD
  // Assuming invert() returns the inverted matrix and might produce NaNs/Infs on failure
  // Or throws an exception if Matrix_Imp.h implements it that way.
  // A robust implementation would check the result.
  Matrix9x9float DtD_inv = DtD.invert();

  // Optional: Add check for NaNs/Infs in DtD_inv if invert doesn't throw/signal error
  // for(size_t i=0; i<81; ++i) { if (!std::isfinite(DtD_inv.data[i])) return false; }

  // Calculate the parameter vector v = (D^T*D)^-1 * (D^T*1)
  Vector9float v = DtD_inv * Dt1;

  // Extract ellipsoid parameters from v = [A, B, C, D, E, F, G, H, I]^T
  Matrix3x3float Q;
  Q(0, 0) = v(0, 0);            // A
  Q(1, 1) = v(1, 0);            // B
  Q(2, 2) = v(2, 0);            // C
  Q(0, 1) = Q(1, 0) = v(3, 0);  // D
  Q(0, 2) = Q(2, 0) = v(4, 0);  // E
  Q(1, 2) = Q(2, 1) = v(5, 0);  // F

  Vector3float GHI;  // Vector [G, H, I] - Renamed to avoid conflict with hard_iron 'H'
  GHI(0, 0) = v(6, 0);
  GHI(1, 0) = v(7, 0);
  GHI(2, 0) = v(8, 0);

  // Calculate hard iron offset: H_offset = -Q^-1 * GHI
  float det_Q = Q.determinant();
  if (std::fabs(det_Q) < std::numeric_limits<float>::epsilon() * 10) {
    return false;  // Cannot invert Q
  }
  Matrix3x3float Q_inv = Q.invert();
  // Optional: check Q_inv for validity

  hard_iron = Q_inv * GHI * -1.0f;

  // Calculate the shape matrix M' = Q / scale
  // scale = H_offset^T * Q * H_offset + GHI^T * H_offset + 1 (Since J = -1 for Dv=1)
  // Note: The term should be 2*GHI^T*H_offset. Let's use the simpler form:
  // (m-H)^T Q (m-H) = H^T Q H + GHI^T H + 1 -> This is the value on RHS
  RowVector3float H_T = hard_iron.transpose();  // 1x3
  RowVector3float GHI_T = GHI.transpose();      // 1x3
  Matrix<float, 1, 1> HtQH = H_T * Q * hard_iron;
  Matrix<float, 1, 1> GHItH = GHI_T * hard_iron;
  float scale = HtQH(0, 0) + GHItH(0, 0) + 1.0f;

  if (std::fabs(scale) < std::numeric_limits<float>::epsilon()) {
    return false;  // Scale factor is too small
  }
  Matrix3x3float M_prime = Q * (1.0f / scale);

  // Calculate S = sqrt(M'^-1) using Eigenvalue Decomposition
  float det_M_prime = M_prime.determinant();
  if (std::fabs(det_M_prime) < std::numeric_limits<float>::epsilon() * 10) {
    return false;  // Cannot invert M'
  }
  Matrix3x3float M_prime_inv = M_prime.invert();
  // Optional: check M_prime_inv for validity

  // M'^-1 = V * Lambda * V^T
  Vector3float eigenvalues;     // Stores eigenvalues (diagonal of Lambda)
  Matrix3x3float eigenvectors;  // Stores eigenvectors (columns of V)

  if (!eigenDecompositionSym3x3(M_prime_inv, eigenvalues, eigenvectors)) {
    // Eigenvalue decomposition failed (likely didn't converge)
    return false;
  }

  // Check for non-positive eigenvalues (indicates poor fit or data)
  if (eigenvalues(0, 0) <= std::numeric_limits<float>::epsilon() ||
      eigenvalues(1, 0) <= std::numeric_limits<float>::epsilon() ||
      eigenvalues(2, 0) <= std::numeric_limits<float>::epsilon()) {
    // Use epsilon tolerance instead of strict zero
    return false;  // Ellipsoid is not positive definite -> bad fit
  }

  // Calculate sqrt(Lambda)
  Matrix3x3float Lambda_sqrt_inv = Matrix3x3float::identity();  // Base name on what it represents M'^-1 = V L V^T
  Lambda_sqrt_inv(0, 0) = std::sqrt(eigenvalues(0, 0));
  Lambda_sqrt_inv(1, 1) = std::sqrt(eigenvalues(1, 0));
  Lambda_sqrt_inv(2, 2) = std::sqrt(eigenvalues(2, 0));

  // Calculate S = V * sqrt(Lambda_of_Mprime_inv) * V^T
  soft_iron = eigenvectors * Lambda_sqrt_inv * eigenvectors.transpose();

  _calibrationComplete = true;  // Set calibration complete flag

  // Note: The optional scaling based on expected_field_magnitude_gauss can still be added here if desired.

  _storeCalibrationData();
  return true;  // Successfully calculated parameters
}
#endif  // MATLAB_SIM