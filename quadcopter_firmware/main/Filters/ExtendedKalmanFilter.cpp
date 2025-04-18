#include "ExtendedKalmanFilter.h"

#include <Logger.h>

#include <cmath>
#include <stdexcept>  // For potential errors

// Small value to prevent division by zero
constexpr float EPSILON = 1e-9f;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

// Constant for converting degrees to radians
#ifndef DEG_TO_RADS
#define DEG_TO_RADS(x) x * 0.017453292519943295769236907684886f
#endif  // DEG_TO_RADS

#ifndef RAD_TO_DEGS
#define RAD_TO_DEGS(x) x * 57.295779513082320876798154814105f
#endif  // RAD_TO_DEGS

// Quaternion Helper Function Implementations
// (Place these here or in a separate utility file included by both .h and .cpp)

Matrix<float, 4, 1> quaternionMultiplyHamiltonProduct(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2)
{
  Matrix<float, 4, 1> q_out;
  q_out(0, 0) = q1(0, 0) * q2(0, 0) - q1(1, 0) * q2(1, 0) - q1(2, 0) * q2(2, 0) - q1(3, 0) * q2(3, 0);  // w
  q_out(1, 0) = q1(0, 0) * q2(1, 0) + q1(1, 0) * q2(0, 0) + q1(2, 0) * q2(3, 0) - q1(3, 0) * q2(2, 0);  // x
  q_out(2, 0) = q1(0, 0) * q2(2, 0) - q1(1, 0) * q2(3, 0) + q1(2, 0) * q2(0, 0) + q1(3, 0) * q2(1, 0);  // y
  q_out(3, 0) = q1(0, 0) * q2(3, 0) + q1(1, 0) * q2(2, 0) - q1(2, 0) * q2(1, 0) + q1(3, 0) * q2(0, 0);  // z
  return q_out;
}

Matrix<float, 4, 1> normalizeQuaternion(Matrix<float, 4, 1>& q)
{
  float norm = sqrt(q(0, 0) * q(0, 0) + q(1, 0) * q(1, 0) + q(2, 0) * q(2, 0) + q(3, 0) * q(3, 0));
  if (norm > EPSILON) {
    q /= norm;
  } else {
    // Avoid division by zero, reset to identity
    q(0, 0) = 1.0f;
    q(1, 0) = 0.0f;
    q(2, 0) = 0.0f;
    q(3, 0) = 0.0f;
  }
  return q;
}

Matrix<float, 3, 3> quaternionToRotationMatrix(const Matrix<float, 4, 1>& q)
{
  float q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
  float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
  float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
  float q1q2 = q1 * q2, q1q3 = q1 * q3;
  float q2q3 = q2 * q3;

  Matrix<float, 3, 3> R;
  R(0, 0) = q0q0 + q1q1 - q2q2 - q3q3;
  R(0, 1) = 2.0f * (q1q2 - q0q3);
  R(0, 2) = 2.0f * (q1q3 + q0q2);
  R(1, 0) = 2.0f * (q1q2 + q0q3);
  R(1, 1) = q0q0 - q1q1 + q2q2 - q3q3;
  R(1, 2) = 2.0f * (q2q3 - q0q1);
  R(2, 0) = 2.0f * (q1q3 - q0q2);
  R(2, 1) = 2.0f * (q2q3 + q0q1);
  R(2, 2) = q0q0 - q1q1 - q2q2 + q3q3;
  return R;  // This is rotation from Body to World. World to Body is transpose.
}

Matrix<float, 3, 1> getYawPitchRollDegreesFromQuaternion(const Matrix<float, 4, 1>& q)
{
  Matrix<float, 4, 1> q_norm = q;
  q_norm = normalizeQuaternion(q_norm);

  // Extract quaternion components
  // Assuming q = [q0, q1, q2, q3] where q0 is scalar (qw), q1=qx, q2=qy, q3=qz
  const float q0 = q_norm(0, 0);  // qw
  const float q1 = q_norm(1, 0);  // qx
  const float q2 = q_norm(2, 0);  // qy
  const float q3 = q_norm(3, 0);  // qz

  Matrix<float, 3, 1> ypr_deg;         // Output matrix [Yaw, Pitch, Roll] in degrees
  float yaw_rad, pitch_rad, roll_rad;  // Angles in radians

  // Pre-calculate intermediate values for efficiency
  const float q0q0 = q0 * q0;
  const float q1q1 = q1 * q1;
  const float q2q2 = q2 * q2;
  const float q3q3 = q3 * q3;

  const float q0q1 = q0 * q1;
  const float q0q2 = q0 * q2;
  const float q0q3 = q0 * q3;
  const float q1q2 = q1 * q2;
  const float q1q3 = q1 * q3;
  const float q2q3 = q2 * q3;

  // Pitch (y-axis rotation) calculation and gimbal lock check
  // sin(pitch) = 2 * (q0*q2 - q3*q1)
  float sinp_arg = 2.0f * (q0q2 - q1q3);

  // Use a small epsilon for floating point comparison to avoid issues near +/-1
  const float epsilon = 1e-6f;  // Adjust if necessary

  if (fabsf(sinp_arg) >= (1.0f - epsilon)) {
    // Gimbal lock: Pitch is +/- 90 degrees
    pitch_rad = copysignf(M_PI_2, sinp_arg);  // +/- pi/2 radians

    // In gimbal lock, yaw and roll are coupled.
    // Conventionally, set roll to 0 and calculate yaw.
    roll_rad = 0.0f;
    yaw_rad = copysignf(2.0f * atan2f(q1, q0), sinp_arg);  // Radians
  } else {
    // Normal case: Calculate pitch, roll, yaw in radians
    pitch_rad = asinf(sinp_arg);

    // Roll (x-axis rotation) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    roll_rad = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);

    // Yaw (z-axis rotation) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    yaw_rad = atan2f(2.0f * (q0q3 + q1q2), q0q0 + q1q1 - q2q2 - q3q3);

    float denom = q0q0 + q1q1 - q2q2 - q3q3;
    yaw_rad = atan2f(2.0f * (q0q3 + q1q2), denom);
  }

  // Convert radians to degrees before storing
  ypr_deg(0, 0) = yaw_rad * RAD_TO_DEG;
  ypr_deg(1, 0) = pitch_rad * RAD_TO_DEG;
  ypr_deg(2, 0) = roll_rad * RAD_TO_DEG;

  return ypr_deg;
}

Matrix<float, 3, 1> rotateVectorByQuaternion(const Matrix<float, 3, 1>& v, const Matrix<float, 4, 1>& q)
{
  Matrix<float, 3, 3> R = quaternionToRotationMatrix(q);
  // We need world to body rotation for accel/mag prediction
  // Or body to world for rotating accel measurements
  // Let's assume quaternionToRotationMatrix gives Body->World
  // For predicting accel/mag (gravity/mag_ref in world -> body), use transpose
  // For prediction of vel_z from body accel, use R
  // Here, let's assume we rotate world vector v into body frame: use R^T
  return R.transpose() * v;
}

// EKF Implementation

ExtendedKalmanFilter::ExtendedKalmanFilter(const Config& config) : _config(config)
{
  // Initialize state vector _x
  _x.zeros();
  _x(Q0_IDX, 0) = 1.0f;   // Identity quaternion (w=1, x=0, y=0, z=0)
  _x(ALT_IDX, 0) = 0.0f;  // Start at relative altitude 0
  _x(VELZ_IDX, 0) = 0.0f;
  // Biases are initialized to zero

  // Initialize covariance matrix _PCovariance
  _PCovariance.zeros();
  _PCovariance(Q0_IDX, Q0_IDX) = _config.initial_quat_uncertainty;  // Simplified: Treat quat uncertainty element-wise
  _PCovariance(Q1_IDX, Q1_IDX) = _config.initial_quat_uncertainty;
  _PCovariance(Q2_IDX, Q2_IDX) = _config.initial_quat_uncertainty;
  _PCovariance(Q3_IDX, Q3_IDX) = _config.initial_quat_uncertainty;
  _PCovariance(ALT_IDX, ALT_IDX) = _config.initial_alt_uncertainty;
  _PCovariance(VELZ_IDX, VELZ_IDX) = _config.initial_velz_uncertainty;
  _PCovariance(BG_XIDX, BG_XIDX) = _config.initial_gyro_bias_uncertainty;
  _PCovariance(BGY_IDX, BGY_IDX) = _config.initial_gyro_bias_uncertainty;
  _PCovariance(BGZ_IDX, BGZ_IDX) = _config.initial_gyro_bias_uncertainty;
  _PCovariance(BBARO_IDX, BBARO_IDX) = _config.initial_baro_bias_uncertainty;

  // Initialize base process noise Q_base (variances per second)
  _Q_base.zeros();
  // Simplified: Assume process noise affects quaternion attitude directly (tune this)
  float quat_PCovariancerocess_noise = config.gyro_noise_density;  // Simplified link
  _Q_base(Q0_IDX, Q0_IDX) = quat_PCovariancerocess_noise;
  _Q_base(Q1_IDX, Q1_IDX) = quat_PCovariancerocess_noise;
  _Q_base(Q2_IDX, Q2_IDX) = quat_PCovariancerocess_noise;
  _Q_base(Q3_IDX, Q3_IDX) = quat_PCovariancerocess_noise;
  _Q_base(ALT_IDX, ALT_IDX) = 0.0f;                         // Altitude process noise comes via velocity state
  _Q_base(VELZ_IDX, VELZ_IDX) = config.velz_Process_noise;  // Uncertainty in vertical velocity prediction model
  _Q_base(BG_XIDX, BG_XIDX) = config.gyro_bias_random_walk;
  _Q_base(BGY_IDX, BGY_IDX) = config.gyro_bias_random_walk;
  _Q_base(BGZ_IDX, BGZ_IDX) = config.gyro_bias_random_walk;
  _Q_base(BBARO_IDX, BBARO_IDX) = config.baro_bias_random_walk;

  // Store measurement noise variances
  _R_accel = _config.accel_noise_variance;
  _R_mag = _config.mag_noise_variance;
  _R_baro = _config.baro_noise_variance;
  _R_range_finder = _config.range_noise_variance;

  // Set gravity vector based on config
  _gravity_world(2, 0) = _config.gravity_magnitude;
}

void ExtendedKalmanFilter::predict(
    float gyro_x_deg_s,
    float gyro_y_deg_s,
    float gyro_z_deg_s,
    float accel_x_m_s2,
    float accel_y_m_s2,
    float accel_z_m_s2,
    float dt)
{
  // ── 0.  Sanity check ───────────────────────────────────────────────────────
  if (dt <= 0.0f) return;

  // (Optional) warn if dt is unusually large
  // if (dt > 0.1f) LOG_WARN("EKF dt = %.3f s", dt);

  // ── 1.  Convert gyro to rad/s and pull current state -----------------------
  float gx = DEG_TO_RADS(gyro_x_deg_s);
  float gy = DEG_TO_RADS(gyro_y_deg_s);
  float gz = DEG_TO_RADS(gyro_z_deg_s);

  Matrix<float, STATE_DIM, 1> x_prev = _x;
  Matrix<float, STATE_DIM, STATE_DIM> P_prev = _PCovariance;

  Matrix<float, 4, 1> q_prev = x_prev.slice<4, 1>(Q0_IDX, 0);
  Matrix<float, 3, 1> b_g = x_prev.slice<3, 1>(BG_XIDX, 0);

  // ── 2.  Attitude propagation ----------------------------------------------
  Matrix<float, 3, 1> omega = {{gx - b_g(0, 0)}, {gy - b_g(1, 0)}, {gz - b_g(2, 0)}};

  Matrix<float, 3, 1> dtheta = omega * dt;
  float dtheta_norm_sq = dtheta(0, 0) * dtheta(0, 0) + dtheta(1, 0) * dtheta(1, 0) + dtheta(2, 0) * dtheta(2, 0);

  Matrix<float, 4, 1> delta_q;
  constexpr float ANGLE_TOL_SQ = 1e-12f;
  if (dtheta_norm_sq > ANGLE_TOL_SQ) {
    float dtheta_norm = std::sqrt(dtheta_norm_sq);
    float half_ang = 0.5f * dtheta_norm;
    float s = std::sin(half_ang);
    float c = std::cos(half_ang);

    Matrix<float, 3, 1> axis = dtheta * (1.0f / dtheta_norm);
    delta_q(0, 0) = c;
    delta_q(1, 0) = axis(0, 0) * s;
    delta_q(2, 0) = axis(1, 0) * s;
    delta_q(3, 0) = axis(2, 0) * s;
  } else  // small‑angle first‑order approximation
  {
    delta_q(0, 0) = 1.0f;
    delta_q(1, 0) = 0.5f * dtheta(0, 0);
    delta_q(2, 0) = 0.5f * dtheta(1, 0);
    delta_q(3, 0) = 0.5f * dtheta(2, 0);
    delta_q = normalizeQuaternion(delta_q);
  }

  Matrix<float, 4, 1> q_new = quaternionMultiplyHamiltonProduct(q_prev, delta_q);
  q_new = normalizeQuaternion(q_new);

  // ── 3.  IMU‑derived vertical acceleration (NED: +Z is Down) ---------------
  Matrix<float, 3, 1> a_body = {{accel_x_m_s2}, {accel_y_m_s2}, {accel_z_m_s2}};
  float a_z = _verticalAccelerationWorld(a_body, q_new);  // m/s², Down +

  // ── 4.  Integrate to vertical velocity and altitude -----------------------
  float velz_prev = x_prev(VELZ_IDX, 0);  // m/s (Down +)
  float alt_prev = x_prev(ALT_IDX, 0);    // m   (Down +)

  float velz_new = velz_prev + a_z * dt;
  float alt_new = alt_prev + velz_prev * dt + 0.5f * a_z * dt * dt;

  // ── 5.  Write propagated state --------------------------------------------
  _x.setSlice<4, 1>(Q0_IDX, 0, q_new);
  _x(VELZ_IDX, 0) = velz_new;
  _x(ALT_IDX, 0) = alt_new;
  // (bias states remain unchanged in the deterministic model)

  // ── 6.  Linearised transition matrix F ------------------------------------
  Matrix<float, STATE_DIM, STATE_DIM> F = _calculateF(q_prev, omega(0, 0), omega(1, 0), omega(2, 0), dt);

  // ── 7.  Base process noise matrix Q ---------------------------------------
  Matrix<float, STATE_DIM, STATE_DIM> Q = _calculateQ(q_prev, dt);

  // Additional continuous‑acceleration noise mapped into (alt, velz)
  const float sigma2_accel = _config.accel_noise_variance;  // (m/s²)²
  float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;

  Q(ALT_IDX, ALT_IDX) += 0.25f * sigma2_accel * dt4;
  Q(ALT_IDX, VELZ_IDX) += 0.5f * sigma2_accel * dt3;
  Q(VELZ_IDX, ALT_IDX) += 0.5f * sigma2_accel * dt3;
  Q(VELZ_IDX, VELZ_IDX) += sigma2_accel * dt2;

  // ── 8.  Covariance propagation -------------------------------------------
  _PCovariance = F * P_prev * F.transpose() + Q;
  // Enforce symmetry (numerical hygiene)
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;
}

Matrix<float, STATE_DIM, STATE_DIM> ExtendedKalmanFilter::_calculateF(
    const Matrix<float, 4, 1>& q,
    float omega_x,
    float omega_y,
    float omega_z,  // Bias-corrected rates (rad/s)
    float dt)
{
  // Initialize F as the Identity matrix (STATE_DIM x STATE_DIM).
  Matrix<float, STATE_DIM, STATE_DIM> F_term = Matrix<float, STATE_DIM, STATE_DIM>::identity();

  // --- Calculate Fqq block: d(quat_new) / d(quat_old) ---
  // Fqq = I(4x4) + 0.5 * Omega(omega) * dt

  // Construct Omega matrix using initializer list
  Matrix<float, 4, 4> Omega = {
      {0.0f,    -omega_x, -omega_y, -omega_z},
      {omega_x, 0.0f,     omega_z,  -omega_y},
      {omega_y, -omega_z, 0.0f,     omega_x },
      {omega_z, omega_y,  -omega_x, 0.0f    }
  };

  // Calculate Fqq = I + Omega * (0.5*dt)
  Matrix<float, 4, 4> Fqq = Matrix<float, 4, 4>::identity() + Omega * (0.5f * dt);

  // Place Fqq into the main F matrix
  F_term.setSlice<4, 4>(Q0_IDX, Q0_IDX, Fqq);

  // --- Calculate Fqbg block: d(quat_new) / d(gyro_bias) ---
  // Fqbg = -0.5 * dt * Xi(q)

  // Extract quaternion components first
  float q0 = q(0, 0);
  float q1 = q(1, 0);
  float q2 = q(2, 0);
  float q3 = q(3, 0);

  // Construct Xi matrix using initializer list
  Matrix<float, 4, 3> Xi = {
      {-q1, -q2, -q3},
      {q0,  -q3, q2 },
      {q3,  q0,  -q1},
      {-q2, q1,  q0 }
  };

  // Calculate Fqbg = Xi * (-0.5*dt)
  Matrix<float, 4, 3> Fqbg = Xi * (-0.5f * dt);

  // Place Fqbg into the main F matrix
  F_term.setSlice<4, 3>(Q0_IDX, BG_XIDX, Fqbg);

  // --- Set other non-identity elements ---
  // d(alt_new) / d(velz_old) = dt
  F_term(ALT_IDX, VELZ_IDX) = dt;

  // All other elements remain as per Identity matrix initialization

  return F_term;
}

Matrix<float, STATE_DIM, STATE_DIM> ExtendedKalmanFilter::_calculateQ(
    const Matrix<float, 4, 1>& q,  // Current quaternion estimate needed for Qqq
    float dt)
{
  // Initialize Q matrix (STATE_DIM x STATE_DIM).
  // Assumes default constructor zeros it, or we use zeros().
  Matrix<float, STATE_DIM, STATE_DIM> Q;

  // --- Calculate Noise Variances for the interval dt ---

  // Gyro noise affecting attitude state (variance = density * dt)
  // Units: (rad/s)^2/Hz * s = rad^2
  float var_gyro = _config.gyro_noise_density * dt;

  // Vertical velocity process noise (from continuous white noise accel model)
  // Units: (m/s^2)^2/Hz
  float Pn_accel = _config.velz_Process_noise;

  // Gyro bias random walk variance (variance = density * dt)
  // Units: (rad/s^2)^2/Hz * s = (rad/s)^2
  float var_gyro_bias = _config.gyro_bias_random_walk * dt;

  // Barometer bias random walk variance (variance = density * dt)
  // Config comment unit "(m/s)^2 / Hz" seems odd, should likely be m^2/Hz.
  // Assuming the value corresponds to the variance rate needed: m^2/s
  // Units: (m^2/s) * s = m^2
  float var_baro_bias = _config.baro_bias_random_walk * dt;

  // --- Populate Diagonal Blocks of Q ---

  // 1. Qqq: Attitude noise block (4x4)
  //    Maps gyro noise variance into quaternion state variance.
  //    Qqq = (0.25 * var_gyro) * (I - q*q^T)
  //    Avoid calculation if noise density is zero.
  if (var_gyro > 1e-12f) {
    // Extract quaternion components first (or pass q to _calculateQ)
    float q0 = q(0, 0);
    float q1 = q(1, 0);
    float q2 = q(2, 0);
    float q3 = q(3, 0);

    // Construct Xi matrix
    Matrix<float, 4, 3> Xi = {
        {-q1, -q2, -q3},
        {q0,  -q3, q2 },
        {q3,  q0,  -q1},
        {-q2, q1,  q0 }
    };

    // Gyro noise covariance matrix (3x3 diagonal)
    Matrix<float, 3, 3> GyroNoiseCov = Matrix<float, 3, 3>::identity() * var_gyro;

    // Calculate Qqq
    Matrix<float, 4, 4> Qqq = Xi * GyroNoiseCov * Xi.transpose() * 0.25f;

    // Set this 4x4 slice into the main Q matrix
    Q.setSlice<4, 4>(Q0_IDX, Q0_IDX, Qqq);
  }

  // 2. Altitude/Velocity Noise Block (2x2)
  //    Derived from integrating white noise acceleration Pn_accel over dt.
  //    Avoid calculation if noise density is zero.
  if (Pn_accel > 1e-12f)  // Use a small tolerance
  {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    // Using direct element access based on indices
    // Assumes ALT_IDX and VELZ_IDX are consecutive or handled correctly by operator()
    Q(ALT_IDX, ALT_IDX) = (dt3 / 3.0f) * Pn_accel;
    Q(ALT_IDX, VELZ_IDX) = (dt2 / 2.0f) * Pn_accel;
    Q(VELZ_IDX, ALT_IDX) = (dt2 / 2.0f) * Pn_accel;  // Symmetric
    Q(VELZ_IDX, VELZ_IDX) = dt * Pn_accel;
  }

  // 3. Gyro Bias Noise Block (Diagonal 3x3)
  //    Represents the random walk of the gyro biases.
  //    Avoid calculation if noise density is zero.
  if (var_gyro_bias > 1e-12f)  // Use a small tolerance
  {
    Q(BG_XIDX, BG_XIDX) = var_gyro_bias;
    Q(BGY_IDX, BGY_IDX) = var_gyro_bias;
    Q(BGZ_IDX, BGZ_IDX) = var_gyro_bias;
  }

  // 4. Barometer Bias Noise (Scalar)
  //    Represents the random walk of the barometer bias.
  //    Avoid calculation if noise density is zero.
  if (var_baro_bias > 1e-12f)  // Use a small tolerance
  {
    Q(BBARO_IDX, BBARO_IDX) = var_baro_bias;
  }

  // All off-diagonal blocks (except alt/vel block) remain zero

  return Q;
}

void ExtendedKalmanFilter::updateAccelerometer(float accel_x, float accel_y, float accel_z)
{
  // --- EKF Update Step using Accelerometer ---

  float accel_norm = std::sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
  float expected_gravity = _config.gravity_magnitude;
  float accel_error = std::fabs(accel_norm - expected_gravity);
  if (accel_error > 5.5f) {  // Tune threshold (e.g., 1.5 m/s^2)
    LOG_ERROR("Skipping accel update due to high acceleration: %.2f", accel_error);
    return;
  }

  // 1. Get predicted state (current _x) and predicted covariance (_PCovariance)
  Matrix<float, STATE_DIM, 1> x_pred = _x;
  Matrix<float, STATE_DIM, STATE_DIM> P_pred = _PCovariance;

  // 2. Extract current attitude quaternion using slice (creates a copy)
  Matrix<float, 4, 1> q = x_pred.slice<4, 1>(Q0_IDX, 0);
  // Note: It might be prudent to normalize q here before using it in calculations,
  // although the state update *should* keep it near-normalized.
  // Matrix<float, 4, 1> q_normalized_temp = normalizeQuaternion(q); // Assuming normalizeQuaternion takes non-const ref
  // but returns the value we need q = q_normalized_temp; // Use the normalized version for calculations if desired

  // 3. Define the measurement vector z
  Matrix<float, 3, 1> z;  // Default constructor zeros the matrix
  z(0, 0) = accel_x;
  z(1, 0) = accel_y;
  z(2, 0) = accel_z;

  // 4. Calculate the predicted measurement h(x)
  // Rotate the negative world gravity vector [-0, -0, -g].
  Matrix<float, 3, 3> R_bw = quaternionToRotationMatrix(q);  // Get Body-to-World matrix
  Matrix<float, 3, 3> R_wb = R_bw.transpose();               // Calculate World-to-Body via transpose
  Matrix<float, 3, 1> h = R_wb * _gravity_world;             // Use the correct R_wb

  // 5. Calculate the measurement residual (innovation) y = z - h(x)
  Matrix<float, 3, 1> y = z - h;  // Matrix subtraction uses operator-

  // 6. Calculate the measurement Jacobian H = dh/dx | x = x_pred
  Matrix<float, 3, STATE_DIM> H;  // Default constructor zeros the matrix

  float q0 = q(0, 0);  // Access elements of the quaternion copy
  float q1 = q(1, 0);
  float q2 = q(2, 0);
  float q3 = q(3, 0);
  float g = _config.gravity_magnitude;

  // Populate the 3x4 Jacobian block related to quaternion states
  // using operator() for element access.
  // ∂h/∂q0
  H(0, Q0_IDX) = -2.0f * g * q2;
  H(1, Q0_IDX) = 2.0f * g * q1;
  H(2, Q0_IDX) = 2.0f * g * q0;
  // ∂h/∂q1
  H(0, Q1_IDX) = 2.0f * g * q3;
  H(1, Q1_IDX) = 2.0f * g * q0;
  H(2, Q1_IDX) = -2.0f * g * q1;
  // ∂h/∂q2
  H(0, Q2_IDX) = -2.0f * g * q0;
  H(1, Q2_IDX) = 2.0f * g * q3;
  H(2, Q2_IDX) = -2.0f * g * q2;
  // ∂h/∂q3
  H(0, Q3_IDX) = 2.0f * g * q1;
  H(1, Q3_IDX) = 2.0f * g * q2;
  H(2, Q3_IDX) = 2.0f * g * q3;
  // Other columns of H remain zero.

  // 7. Define the measurement noise covariance matrix R (3x3)
  Matrix<float, 3, 3> R_mat;  // Default zeroed
  R_mat(0, 0) = _R_accel;
  R_mat(1, 1) = _R_accel;
  R_mat(2, 2) = _R_accel;

  // 8. Calculate the innovation covariance S = H * P_pred * H^T + R
  // Uses operator* for matrix multiplication, transpose() method, and operator+
  Matrix<float, 3, 3> S = H * P_pred * H.transpose() + R_mat;

  // 9. Calculate the Kalman Gain K = P_pred * H^T * S^-1
  // Uses transpose(), operator*, and invert() method
  Matrix<float, STATE_DIM, 3> K = P_pred * H.transpose() * S.invert();

  // 10. Update the state estimate: x_updated = x_pred + K * y
  // Uses operator* and operator+
  _x = x_pred + K * y;

  // 11. Update the state covariance: P_updated = (I - K * H) * P_pred
  // Uses identity() static method, operator-, operator*,
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  // Make sure R_mat (or R for 1D updates) holds the appropriate measurement noise covariance matrix
  // calculated earlier in the specific update function.
  // For updateAccelerometer/Magnetometer, use R_mat (3x3)
  // For updateBarometer/Rangefinder, use R (1x1)
  // Let's assume the variable is called 'R_noise_matrix' in general below:
  // Matrix<float, M, M> R_noise_matrix = ...; // Defined earlier in the function (e.g., R_mat or R)
  _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();

  // Belt-and-braces: Enforce symmetry AFTER Joseph form update
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // --- Optional: Use Joseph form for Covariance Update (more stable) ---
  // Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  // _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();
  // --- End Optional ---

  // --- Ensure Covariance Symmetry (recommended after update) ---
  // _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f; // Uses transpose, +, * scalar
  // --- End Symmetry Enforcement ---

  // 12. Renormalize the quaternion part of the updated state vector _x
  //     using slice() to get a copy and setSlice() to write it back.
  //     Assumes normalizeQuaternion returns the normalized value.
  Matrix<float, 4, 1> q_updated = _x.slice<4, 1>(Q0_IDX, 0);  // Get copy of updated quat
  Matrix<float, 4, 1> q_normalized =
      normalizeQuaternion(q_updated);  // Normalize the copy
                                       // Assumes normalizeQuaternion takes Matrix& but we rely on the return value.
                                       // If it modifies q_updated in place and returns void/ref, this still works.
  _x.setSlice<4, 1>(Q0_IDX, 0, q_normalized);  // Set the normalized values back into _x
}

void ExtendedKalmanFilter::updateMagnetometer(float mag_x, float mag_y, float mag_z)
{
  // --- EKF Update Step using Magnetometer ---

  // 1. Get predicted state and covariance
  Matrix<float, STATE_DIM, 1> x_pred = _x;
  Matrix<float, STATE_DIM, STATE_DIM> P_pred = _PCovariance;

  // 2. Extract current attitude quaternion
  Matrix<float, 4, 1> q = x_pred.slice<4, 1>(Q0_IDX, 0);

  // 3. Construct and Normalize the measurement vector z
  Matrix<float, 3, 1> z_raw = {{mag_x}, {mag_y}, {mag_z}};  // Raw magnetometer reading

  // Calculate the norm (magnitude) of the raw measurement
  float norm_z = std::sqrt(z_raw(0, 0) * z_raw(0, 0) + z_raw(1, 0) * z_raw(1, 0) + z_raw(2, 0) * z_raw(2, 0));

  // Avoid division by zero or near-zero for stability
  const float min_norm = 1e-6f;
  if (norm_z < min_norm) {
    // Magnetometer reading is too weak or zero, cannot reliably use it. Skip update.
    // Optionally add logging here.
    return;
  }

  // Normalize the measurement vector
  Matrix<float, 3, 1> z = z_raw * (1.0f / norm_z);

  // 4. Get the reference magnetic field vector (from config, assumed normalized)
  const Matrix<float, 3, 1>& m_ref = _config.mag_reference_vector;

  // 5. Calculate the predicted measurement h(x)
  //    h(x) = Rotate reference vector from world frame to body frame
  Matrix<float, 3, 3> R_bw = quaternionToRotationMatrix(q);  // Get Body-to-World matrix
  Matrix<float, 3, 3> R_wb = R_bw.transpose();               // Calculate World-to-Body via transpose
  Matrix<float, 3, 1> h = R_wb * m_ref;                      // Use the correct R_wb
  // Note: If m_ref is normalized and R_wb is a valid rotation, h is also normalized.

  // 6. Calculate the measurement residual (innovation) y = z - h
  Matrix<float, 3, 1> y = z - h;

  // 7. Calculate the measurement Jacobian H = dh/dx | x = x_pred (3 x STATE_DIM)
  //    Only depends on quaternion state variables (q0, q1, q2, q3).
  Matrix<float, 3, STATE_DIM> H;  // Default constructor zeros the matrix

  float q0 = q(0, 0);
  float q1 = q(1, 0);
  float q2 = q(2, 0);
  float q3 = q(3, 0);
  float mx = m_ref(0, 0);
  float my = m_ref(1, 0);
  float mz = m_ref(2, 0);

  // H = [ H_q | 0_{3x6} ] where H_q = [ d(R*m_ref)/dq0, d(R*m_ref)/dq1, ... ]
  // Column for q0: d(R*m_ref)/dq0 = (dR/dq0) * m_ref
  H(0, Q0_IDX) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
  H(1, Q0_IDX) = 2.0f * (q3 * mx + q0 * my - q1 * mz);
  H(2, Q0_IDX) = 2.0f * (-q2 * mx + q1 * my + q0 * mz);

  // Column for q1: d(R*m_ref)/dq1 = (dR/dq1) * m_ref
  H(0, Q1_IDX) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
  H(1, Q1_IDX) = 2.0f * (q2 * mx - q1 * my - q0 * mz);
  H(2, Q1_IDX) = 2.0f * (q3 * mx + q0 * my - q1 * mz);

  // Column for q2: d(R*m_ref)/dq2 = (dR/dq2) * m_ref
  H(0, Q2_IDX) = 2.0f * (-q2 * mx + q1 * my + q0 * mz);
  H(1, Q2_IDX) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
  H(2, Q2_IDX) = 2.0f * (-q0 * mx + q3 * my - q2 * mz);

  // Column for q3: d(R*m_ref)/dq3 = (dR/dq3) * m_ref
  H(0, Q3_IDX) = 2.0f * (-q3 * mx - q0 * my + q1 * mz);
  H(1, Q3_IDX) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
  H(2, Q3_IDX) = 2.0f * (q1 * mx + q2 * my + q3 * mz);

  // Columns ALT_IDX through BBARO_IDX remain zero.

  // 8. Define the measurement noise covariance matrix R (3x3)
  //    Uses _R_mag, assumes uncorrelated noise on normalized axes.
  Matrix<float, 3, 3> R_mat;  // Default zeroed
  R_mat(0, 0) = _R_mag;
  R_mat(1, 1) = _R_mag;
  R_mat(2, 2) = _R_mag;

  // 9. Calculate the innovation covariance S = H * P_pred * H^T + R
  Matrix<float, 3, 3> S = H * P_pred * H.transpose() + R_mat;

  // 10. Calculate the Kalman Gain K = P_pred * H^T * S^-1
  //     Ensure S.invert() is numerically stable.
  Matrix<float, STATE_DIM, 3> K = P_pred * H.transpose() * S.invert();

  // 11. Update the state estimate: x_updated = x_pred + K * y
  _x = x_pred + K * y;

  // 12. Update the state covariance: P_updated = (I - K * H) * P_pred
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  // Make sure R_mat (or R for 1D updates) holds the appropriate measurement noise covariance matrix
  // calculated earlier in the specific update function.
  // For updateAccelerometer/Magnetometer, use R_mat (3x3)
  // For updateBarometer/Rangefinder, use R (1x1)
  // Let's assume the variable is called 'R_noise_matrix' in general below:
  // Matrix<float, M, M> R_noise_matrix = ...; // Defined earlier in the function (e.g., R_mat or R)
  _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();

  // Belt-and-braces: Enforce symmetry AFTER Joseph form update
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // --- Optional: Joseph form P = (I-KH)P(I-KH)' + KRK' ---
  // Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  // _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();
  // --- End Optional ---

  // --- Optional: Ensure Covariance Symmetry ---
  // _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;
  // --- End Optional ---

  // 13. Renormalize the quaternion part of the updated state vector _x
  Matrix<float, 4, 1> q_updated = _x.slice<4, 1>(Q0_IDX, 0);
  Matrix<float, 4, 1> q_normalized = normalizeQuaternion(q_updated);  // Assumes normalizeQuaternion works as needed
  _x.setSlice<4, 1>(Q0_IDX, 0, q_normalized);
}

void ExtendedKalmanFilter::updateBarometer(float alt_baro)
{
  Matrix<float, 1, STATE_DIM> H_baro;  // 1x10
  H_baro(0, ALT_IDX) = 1.0f;
  H_baro(0, BBARO_IDX) = 1.0f;

  float h_hat = _x(ALT_IDX, 0) + _x(BBARO_IDX, 0);
  float y = alt_baro - h_hat;

  Matrix<float, 1, 1> S = H_baro * _PCovariance * H_baro.transpose();
  S(0, 0) += _R_baro;

  if (S(0, 0) <= 1e-9f) {
    return;  // Skip update if S is near zero
  }

  Matrix<float, STATE_DIM, 1> K = _PCovariance * H_baro.transpose() * (1.0f / S(0, 0));
  _x = _x + K * y;

  // Use Joseph form for covariance update
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H_baro;
  Matrix<float, 1, 1> R_baro_mat = {{_R_baro}};
  _PCovariance = I_KH * _PCovariance * I_KH.transpose() + K * R_baro_mat * K.transpose();

  // Enforce symmetry
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // Normalize quaternion
  Matrix<float, 4, 1> q = _x.slice<4, 1>(Q0_IDX, 0);
  q = normalizeQuaternion(q);
  _x.setSlice<4, 1>(Q0_IDX, 0, q);
}

void ExtendedKalmanFilter::updateRangefinder(float range_reading)
{
  Matrix<float, 1, STATE_DIM> H_range;  // 1x10
  H_range(0, ALT_IDX) = 1.0f;

  float h_hat = _x(ALT_IDX, 0);
  float y = range_reading - h_hat;

  Matrix<float, 1, 1> S = H_range * _PCovariance * H_range.transpose();
  S(0, 0) += _R_range_finder;

  if (S(0, 0) <= 1e-9f) {
    return;  // Skip update if S is near zero
  }

  Matrix<float, STATE_DIM, 1> K = _PCovariance * H_range.transpose() * (1.0f / S(0, 0));
  _x = _x + K * y;

  // Use Joseph form for covariance update
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H_range;
  Matrix<float, 1, 1> R_range_mat = {{_R_range_finder}};
  _PCovariance = I_KH * _PCovariance * I_KH.transpose() + K * R_range_mat * K.transpose();

  // Enforce symmetry
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // Normalize quaternion
  Matrix<float, 4, 1> q = _x.slice<4, 1>(Q0_IDX, 0);
  q = normalizeQuaternion(q);
  _x.setSlice<4, 1>(Q0_IDX, 0, q);
}

float ExtendedKalmanFilter::_verticalAccelerationWorld(
    const Matrix<float, 3, 1>& a_body, const Matrix<float, 4, 1>& q) const
{
  // Body → World
  Matrix<float, 3, 3> R_bw = quaternionToRotationMatrix(q);
  Matrix<float, 3, 1> a_world = R_bw * a_body;

  // In NED, +Z is down;  _gravity_world = [0,0,+g]^T
  return a_world(2, 0) - _config.gravity_magnitude;  // m/s² (net vertical)
}

// Getters Implementation

Matrix<float, 3, 1> ExtendedKalmanFilter::getYawPitchRollDegrees(void)
{
  auto quaternions = getAttitudeQuaternion();
  return getYawPitchRollDegreesFromQuaternion(quaternions);
}

Matrix<float, 4, 1> ExtendedKalmanFilter::getAttitudeQuaternion()
{
  auto subMatrix = _x.slice<4, 1>(Q0_IDX, 0);
  return subMatrix;
}

float ExtendedKalmanFilter::getAltitude() const { return _x(ALT_IDX, 0); }

float ExtendedKalmanFilter::getVerticalVelocity() const { return _x(VELZ_IDX, 0); }

Matrix<float, 3, 1> ExtendedKalmanFilter::getGyroBias() { return _x.slice<3, 1>(BG_XIDX, 0); }

float ExtendedKalmanFilter::getBarometerBias() const { return _x(BBARO_IDX, 0); }