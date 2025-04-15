#pragma once

#include <Matrix.h>

#include <cmath>
#include <vector>

/**
 * @brief Simple struct to hold 3D vector data (x,y,z).
 */
struct Vector3f
{
  float x;
  float y;
  float z;
};

/**
 * @brief Simple struct to hold Euler yaw, pitch, and roll
 */
struct EulerAngle
{
  float yaw;
  float pitch;
  float roll;
};

constexpr size_t STATE_DIM = 10;

// These are indices into the _x state vector
constexpr int Q0_IDX = 0;
constexpr int Q1_IDX = 1;
constexpr int Q2_IDX = 2;
constexpr int Q3_IDX = 3;
constexpr int ALT_IDX = 4;
constexpr int VELZ_IDX = 5;
constexpr int BG_XIDX = 6;
constexpr int BGY_IDX = 7;
constexpr int BGZ_IDX = 8;
constexpr int BBARO_IDX = 9;

// Quaternion Helper Function Declarations
Matrix<float, 4, 1> quaternionMultiplyHamiltonProduct(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2);
Matrix<float, 3, 1> rotateVectorByQuaternion(const Matrix<float, 3, 1>& v, const Matrix<float, 4, 1>& q);
Matrix<float, 4, 1> normalizeQuaternion(Matrix<float, 4, 1>& q);
Matrix<float, 3, 3> quaternionToRotationMatrix(const Matrix<float, 4, 1>& q);
Matrix<float, 3, 1> getYawPitchRollDegreesFromQuaternion(const Matrix<float, 4, 1>& q);

class ExtendedKalmanFilter
{
 public:
  // Structure to hold configuration parameters
  struct Config
  {
    // Initial state uncertainty (variances for diagonal of P0)
    float initial_quat_uncertainty = 0.1f;
    float initial_alt_uncertainty = 1.0f;
    float initial_velz_uncertainty = 0.1f;
    float initial_gyro_bias_uncertainty = 0.01f;
    float initial_baro_bias_uncertainty = 0.5f;

    // Process noise spectral densities (variances added per second)
    float gyro_noise_density = 0.001f;    // (rad/s)^2 / Hz -> Variance = density * dt
    float gyro_bias_random_walk = 1e-5f;  // (rad/s^2)^2 / Hz -> Variance = density * dt
    float velz_Process_noise = 0.1f;      // (m/s^2)^2 / Hz -> Variance = density * dt (Simplified model)
    float baro_bias_random_walk = 1e-5f;  // (m/s)^2 / Hz -> Variance = density * dt

    // Measurement noise variances (scalar)
    float accel_noise_variance = 0.05f;  // (m/s^2)^2
    float mag_noise_variance = 0.001f;   // (normalized units)^2
    float baro_noise_variance = 1.0f;    // m^2
    float range_noise_variance = 0.01f;  // m^2

    // Other constants
    float gravity_magnitude = 9.81f;
    Matrix<float, 3, 1> mag_reference_vector = {
        {0.446f}, {0.116f}, {0.887f}};  // adjust for location, this is for Foster City, CA
  };

  ExtendedKalmanFilter(const Config& config);

  // Prediction step based on gyro (mandatory) and dt
  // Gyro values in degrees per second
  void predict(float gyro_x_deg_s, float gyro_y_deg_s, float gyro_z_deg_s, float dt);

  // Update step using accelerometer measurement (for attitude correction)
  // Accelerometer values in G's in meters / (second ^ 2)
  void updateAccelerometer(float accel_x, float accel_y, float accel_z);

  // Update step using magnetometer measurement (for yaw correction)
  // Magnetometer values in gauss values
  void updateMagnetometer(float mag_x, float mag_y, float mag_z);

  // Update step using barometer measurement (for altitude)
  // Barometer value is height difference in meters since initialization
  void updateBarometer(float alt_baro);

  // Update step using rangefinder measurement (for altitude)
  // Range reading value is height difference in meters since initialization
  void updateRangefinder(float range_reading);

  // --- Getters ---
  const Matrix<float, STATE_DIM, 1>& getState() const { return _x; }
  const Matrix<float, STATE_DIM, STATE_DIM>& getCovariance() const { return _PCovariance; }

  Matrix<float, 3, 1> getYawPitchRollDegrees(void);
  Matrix<float, 4, 1> getAttitudeQuaternion();
  float getAltitude() const;
  float getVerticalVelocity() const;
  Matrix<float, 3, 1> getGyroBias();
  float getBarometerBias() const;

 private:
  // State vector: [q0, q1, q2, q3, alt, vel_z, bias_gx, bias_gy, bias_gz, bias_baro]
  Matrix<float, STATE_DIM, 1> _x;

  // Process noise covariance matrix (diagonal approximation based on config)
  Matrix<float, STATE_DIM, STATE_DIM> _Q_base;  // Base variances per second

  // State covariance matrix
  Matrix<float, STATE_DIM, STATE_DIM> _PCovariance;

  // Measurement noise variances (stored from config)
  float _R_accel;
  float _R_mag;
  float _R_baro;
  float _R_range_finder;

  // Configuration parameters
  Config _config;

  // Gravity vector (NED frame assumed: North, East, Down)
  Matrix<float, 3, 1> _gravity_world = {{0.0f}, {0.0f}, {_config.gravity_magnitude}};

  // --- Private Helper Methods ---
  // Calculates the state transition Jacobian F for the prediction step
  Matrix<float, STATE_DIM, STATE_DIM> _calculateF(
      const Matrix<float, 4, 1>& q,  // Current quaternion state
      float omega_x,
      float omega_y,
      float omega_z,  // Bias-corrected rates (rad/s)
      float dt);

  // Calculates the process noise matrix Q for a given dt
  Matrix<float, STATE_DIM, STATE_DIM> _calculateQ(
      const Matrix<float, 4, 1>& q,  // Current quaternion estimate needed for Qqq
      float dt);
};