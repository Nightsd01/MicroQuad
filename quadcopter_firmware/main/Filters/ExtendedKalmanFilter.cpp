#include "ExtendedKalmanFilter.h"

#include <Logger.h>

#include <cmath>
#include <stdexcept>  // For potential errors

#include "Constants.h"

// Small value to prevent division by zero
constexpr float EPSILON = 1e-9f;

// Quaternion Helper Function Implementations
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
  }

  // Convert radians to degrees before storing
  ypr_deg(0, 0) = RAD_TO_DEGS(yaw_rad);
  ypr_deg(1, 0) = RAD_TO_DEGS(pitch_rad);
  ypr_deg(2, 0) = RAD_TO_DEGS(roll_rad);

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

  // Set magnetic reference vector (NED frame, unit vector)
  // Assuming declination 0, inclination ~60 degrees (typical mid-latitude)
  // Set magnetic reference vector from config and normalize
  _mag_ref_world = _config.mag_reference_vector;
  float mag_norm = std::sqrt(_mag_ref_world.dot(_mag_ref_world));
  if (mag_norm > EPSILON) {
    _mag_ref_world /= mag_norm;
  } else {
    LOG_ERROR("Mag reference vector is zero, setting to horizontal north");
    _mag_ref_world(0, 0) = 1.0f;  // Horizontal north as fallback
    _mag_ref_world(1, 0) = 0.0f;
    _mag_ref_world(2, 0) = 0.0f;
  }
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
  if (dt <= 0.0f) return;

  // Convert gyro to rad/s
  float gx = DEG_TO_RADS(gyro_x_deg_s);
  float gy = DEG_TO_RADS(gyro_y_deg_s);
  float gz = DEG_TO_RADS(gyro_z_deg_s);

  // Copy old state/cov
  Matrix<float, STATE_DIM, 1> x_prev = _x;
  Matrix<float, STATE_DIM, STATE_DIM> P_prev = _PCovariance;

  // Extract some sub-states
  Matrix<float, 4, 1> q_prev = x_prev.slice<4, 1>(Q0_IDX, 0);
  Matrix<float, 3, 1> b_g = x_prev.slice<3, 1>(BG_XIDX, 0);

  // Body-frame acceleration vector
  Matrix<float, 3, 1> a_body = {{accel_x_m_s2}, {accel_y_m_s2}, {accel_z_m_s2}};

  // Bias-corrected angular rates
  Matrix<float, 3, 1> omega = {{gx - b_g(0, 0)}, {gy - b_g(1, 0)}, {gz - b_g(2, 0)}};

  // --- 1) Attitude propagation ---
  Matrix<float, 3, 1> dtheta = omega * dt;
  float dtheta_norm_sq = dtheta.dot(dtheta);

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
  } else {
    // small-angle approximation
    delta_q(0, 0) = 1.0f;
    delta_q(1, 0) = 0.5f * dtheta(0, 0);
    delta_q(2, 0) = 0.5f * dtheta(1, 0);
    delta_q(3, 0) = 0.5f * dtheta(2, 0);
    normalizeQuaternion(delta_q);
  }

  Matrix<float, 4, 1> q_new = quaternionMultiplyHamiltonProduct(q_prev, delta_q);
  q_new = normalizeQuaternion(q_new);

  // --- 2) Vertical acceleration in world (NED) ---
  float a_z = _verticalAccelerationWorld(a_body, q_new);  // +Z down

  // --- 3) Integrate alt & vel ---
  float velz_prev = x_prev(VELZ_IDX, 0);
  float alt_prev = x_prev(ALT_IDX, 0);

  float velz_new = velz_prev + a_z * dt;
  float alt_new = alt_prev + velz_prev * dt + 0.5f * a_z * dt * dt;

  // Write back to state
  _x.setSlice<4, 1>(Q0_IDX, 0, q_new);
  _x(VELZ_IDX, 0) = velz_new;
  _x(ALT_IDX, 0) = alt_new;

  // --- 4) Build F with new partial derivatives ---
  Matrix<float, STATE_DIM, STATE_DIM> F =
      _calculateF(q_prev, omega(0, 0), omega(1, 0), omega(2, 0), a_body, dt);  // <-- CHANGED

  // --- 5) Build Q ---
  Matrix<float, STATE_DIM, STATE_DIM> Q = _calculateQ(q_prev, dt);

  // --- 6) Covariance propagation ---
  _PCovariance = F * P_prev * F.transpose() + Q;
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;  // enforce symmetry
}

static Matrix<float, 1, 4> partialAzWrtQuaternion(const Matrix<float, 4, 1>& q, const Matrix<float, 3, 1>& abody)
{
  // For a_z = the 3rd component of (R_bw(q) * abody), ignoring the constant -g
  float q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
  float ax = abody(0, 0), ay = abody(1, 0), az = abody(2, 0);

  // By explicit partial derivatives of the 3rd row of R_bw * abody
  // R_bw(2,0) = 2(q1q3 - q0q2)
  // R_bw(2,1) = 2(q2q3 + q0q1)
  // R_bw(2,2) = (q0^2 - q1^2 - q2^2 + q3^2)
  // => a_z = R_bw(2,0)*ax + R_bw(2,1)*ay + R_bw(2,2)*az

  Matrix<float, 1, 4> d;
  d.zeros();

  // wrt q0
  d(0, 0) = -2.f * q2 * ax + 2.f * q1 * ay + 2.f * q0 * az;
  // wrt q1
  d(0, 1) = 2.f * q3 * ax + 2.f * q0 * ay - 2.f * q1 * az;
  // wrt q2
  d(0, 2) = -2.f * q0 * ax + 2.f * q3 * ay - 2.f * q2 * az;
  // wrt q3
  d(0, 3) = 2.f * q1 * ax + 2.f * q2 * ay + 2.f * q3 * az;

  return d;
}

Matrix<float, STATE_DIM, STATE_DIM> ExtendedKalmanFilter::_calculateF(
    const Matrix<float, 4, 1>& q,
    float omega_x,
    float omega_y,
    float omega_z,
    const Matrix<float, 3, 1>& a_body,  // <-- ADDED
    float dt)
{
  Matrix<float, STATE_DIM, STATE_DIM> F_term = Matrix<float, STATE_DIM, STATE_DIM>::identity();

  // --- Fqq block (attitude) ---
  Matrix<float, 4, 4> Omega = {
      {0.0f,    -omega_x, -omega_y, -omega_z},
      {omega_x, 0.0f,     omega_z,  -omega_y},
      {omega_y, -omega_z, 0.0f,     omega_x },
      {omega_z, omega_y,  -omega_x, 0.0f    }
  };

  Matrix<float, 4, 4> Fqq = Matrix<float, 4, 4>::identity() + Omega * (0.5f * dt);
  F_term.setSlice<4, 4>(Q0_IDX, Q0_IDX, Fqq);

  // --- Fqbg: derivative of quaternion wrt gyro biases ---
  float q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
  Matrix<float, 4, 3> Xi = {
      {-q1, -q2, -q3},
      {q0,  -q3, q2 },
      {q3,  q0,  -q1},
      {-q2, q1,  q0 }
  };
  Matrix<float, 4, 3> Fqbg = Xi * (-0.5f * dt);
  F_term.setSlice<4, 3>(Q0_IDX, BG_XIDX, Fqbg);

  // --- F for alt and velz wrt alt, velz ---
  F_term(ALT_IDX, VELZ_IDX) = dt;  // d(alt) / d(velz) = dt

  // --- NEW: partial derivatives of (velz, alt) wrt quaternion ---
  //    velz_new = velz_old + a_z(q)*dt
  // => partial wrt q = dt * d(a_z)/dq
  //    alt_new = alt_old + velz_old*dt + 0.5*a_z(q)*dt^2
  // => partial wrt q = 0.5*dt^2 * d(a_z)/dq
  Matrix<float, 1, 4> dAz_dq = partialAzWrtQuaternion(q, a_body);

  // place them into the correct rows:
  for (int i = 0; i < 4; ++i) {
    F_term(VELZ_IDX, Q0_IDX + i) = dt * dAz_dq(0, i);             // d(velz)/dq
    F_term(ALT_IDX, Q0_IDX + i) = 0.5f * dt * dt * dAz_dq(0, i);  // d(alt)/dq
  }

  return F_term;
}

Matrix<float, 3, 4> ExtendedKalmanFilter::_computeVectorJacobian(
    const Matrix<float, 4, 1>& q, const Matrix<float, 3, 1>& ref_vec)
{
  Matrix<float, 3, 4> H;
  float q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
  float vx = ref_vec(0, 0), vy = ref_vec(1, 0), vz = ref_vec(2, 0);

  // Row 0 (hx partials)
  H(0, 0) = 2 * q0 * vx + 2 * q3 * vy - 2 * q2 * vz;
  H(0, 1) = 2 * q1 * vx + 2 * q2 * vy + 2 * q3 * vz;
  H(0, 2) = -2 * q2 * vx + 2 * q1 * vy - 2 * q0 * vz;
  H(0, 3) = -2 * q3 * vx + 2 * q0 * vy + 2 * q1 * vz;

  // Row 1 (hy partials)
  H(1, 0) = -2 * q3 * vx + 2 * q0 * vy + 2 * q1 * vz;
  H(1, 1) = 2 * q2 * vx - 2 * q1 * vy + 2 * q0 * vz;
  H(1, 2) = 2 * q1 * vx + 2 * q2 * vy + 2 * q3 * vz;
  H(1, 3) = -2 * q0 * vx - 2 * q3 * vy + 2 * q2 * vz;

  // Row 2 (hz partials)
  H(2, 0) = 2 * q2 * vx - 2 * q1 * vy + 2 * q0 * vz;
  H(2, 1) = 2 * q3 * vx - 2 * q0 * vy - 2 * q1 * vz;
  H(2, 2) = 2 * q0 * vx + 2 * q3 * vy - 2 * q2 * vz;
  H(2, 3) = 2 * q1 * vx + 2 * q2 * vy + 2 * q3 * vz;

  return H;
}

Matrix<float, STATE_DIM, STATE_DIM> ExtendedKalmanFilter::_calculateQ(
    const Matrix<float, 4, 1>& q,  // Current quaternion estimate needed for Qqq
    float dt)
{
  // Initialize Q matrix (STATE_DIM x STATE_DIM).
  // Assumes default constructor zeros it, or we use zeros().
  Matrix<float, STATE_DIM, STATE_DIM> Q;

  // --- Calculate Noise Variances for the interval dt ---

  // Gyro noise affecting attitude state (variance = density^2 * dt)
  float var_gyro = std::pow(_config.gyro_noise_density, 2) * dt;

  // Vertical velocity process noise (from continuous white noise accel model)
  float Pn_accel = _config.velz_Process_noise;

  // Gyro bias random walk variance (variance = density^2 * dt)
  float var_gyro_bias = std::pow(_config.gyro_bias_random_walk, 2) * dt;

  // Barometer bias random walk variance (variance = density^2 * dt)
  float var_baro_bias = std::pow(_config.baro_bias_random_walk, 2) * dt;

  // --- Populate Diagonal Blocks of Q ---

  // 1. Qqq: Attitude noise block (4x4)
  if (var_gyro > 1e-12f) {
    float q0 = q(0, 0);
    float q1 = q(1, 0);
    float q2 = q(2, 0);
    float q3 = q(3, 0);

    Matrix<float, 4, 3> Xi = {
        {-q1, -q2, -q3},
        {q0,  -q3, q2 },
        {q3,  q0,  -q1},
        {-q2, q1,  q0 }
    };

    Matrix<float, 3, 3> GyroNoiseCov = Matrix<float, 3, 3>::identity() * var_gyro;

    Matrix<float, 4, 4> Qqq = Xi * GyroNoiseCov * Xi.transpose() * 0.25f;

    Q.setSlice<4, 4>(Q0_IDX, Q0_IDX, Qqq);
  }

  // 2. Altitude/Velocity Noise Block (2x2)
  if (Pn_accel > 1e-12f) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    Q(ALT_IDX, ALT_IDX) = (dt3 / 3.0f) * Pn_accel;
    Q(ALT_IDX, VELZ_IDX) = (dt2 / 2.0f) * Pn_accel;
    Q(VELZ_IDX, ALT_IDX) = (dt2 / 2.0f) * Pn_accel;  // Symmetric
    Q(VELZ_IDX, VELZ_IDX) = dt * Pn_accel;
  }

  // 3. Gyro Bias Noise Block (Diagonal 3x3)
  if (var_gyro_bias > 1e-12f) {
    Q(BG_XIDX, BG_XIDX) = var_gyro_bias;
    Q(BGY_IDX, BGY_IDX) = var_gyro_bias;
    Q(BGZ_IDX, BGZ_IDX) = var_gyro_bias;
  }

  // 4. Barometer Bias Noise (Scalar)
  if (var_baro_bias > 1e-12f) {
    Q(BBARO_IDX, BBARO_IDX) = var_baro_bias;
  }

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
  Matrix<float, 3, 1> z = {{accel_x}, {accel_y}, {accel_z}};

  // 4. Calculate the predicted measurement h(x)
  Matrix<float, 3, 3> R_bw = quaternionToRotationMatrix(q);  // Body-to-World
  Matrix<float, 3, 3> R_wb = R_bw.transpose();               // World-to-Body
  Matrix<float, 3, 1> h = R_wb * _gravity_world;

  // 5. Calculate the measurement residual (innovation) y = z - h(x)
  Matrix<float, 3, 1> y = z - h;

  // 6. Calculate the measurement Jacobian H = dh/dx | x = x_pred
  Matrix<float, 3, STATE_DIM> H;  // zeros

  float q0 = q(0, 0);
  float q1 = q(1, 0);
  float q2 = q(2, 0);
  float q3 = q(3, 0);
  float g = _config.gravity_magnitude;

  H(0, Q0_IDX) = -2.0f * g * q2;
  H(1, Q0_IDX) = 2.0f * g * q1;
  H(2, Q0_IDX) = 2.0f * g * q0;
  H(0, Q1_IDX) = 2.0f * g * q3;
  H(1, Q1_IDX) = 2.0f * g * q0;
  H(2, Q1_IDX) = -2.0f * g * q1;
  H(0, Q2_IDX) = -2.0f * g * q0;
  H(1, Q2_IDX) = 2.0f * g * q3;
  H(2, Q2_IDX) = -2.0f * g * q2;
  H(0, Q3_IDX) = 2.0f * g * q1;
  H(1, Q3_IDX) = 2.0f * g * q2;
  H(2, Q3_IDX) = 2.0f * g * q3;

  // 7. Measurement noise R
  Matrix<float, 3, 3> R_mat;
  R_mat(0, 0) = _R_accel;
  R_mat(1, 1) = _R_accel;
  R_mat(2, 2) = _R_accel;

  // 8. Innovation covariance S
  Matrix<float, 3, 3> S = H * P_pred * H.transpose() + R_mat;

  // 9. Kalman Gain K
  Matrix<float, STATE_DIM, 3> K = P_pred * H.transpose() * S.invert();

  // 10. Update state
  _x = x_pred + K * y;

  // 11. Update covariance (Joseph form)
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();

  // Enforce symmetry
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // 12. Renormalize quaternion
  Matrix<float, 4, 1> q_updated = _x.slice<4, 1>(Q0_IDX, 0);
  q_updated = normalizeQuaternion(q_updated);
  _x.setSlice<4, 1>(Q0_IDX, 0, q_updated);
}

void ExtendedKalmanFilter::updateMagnetometer(float mag_x, float mag_y, float mag_z)
{
  // --- EKF Update Step using Magnetometer (vector-based) ---

  // 1. Copy predicted state and covariance
  Matrix<float, STATE_DIM, 1> x_pred = _x;
  Matrix<float, STATE_DIM, STATE_DIM> P_pred = _PCovariance;

  // 2. Normalize measurement vector (to unit vector)
  Matrix<float, 3, 1> z = {{mag_x}, {mag_y}, {mag_z}};
  float mag_norm = std::sqrt(z.dot(z));
  if (mag_norm < EPSILON) {
    return;  // Skip if near-zero
  }
  z /= mag_norm;

  // 3. Extract quaternion
  Matrix<float, 4, 1> q = x_pred.slice<4, 1>(Q0_IDX, 0);
  q = normalizeQuaternion(q);  // Ensure normalized

  // 4. Predicted measurement h (world mag ref rotated to body frame, normalized)
  Matrix<float, 3, 3> R_bw = quaternionToRotationMatrix(q);
  Matrix<float, 3, 3> R_wb = R_bw.transpose();
  Matrix<float, 3, 1> h = R_wb * _mag_ref_world;
  float h_norm = std::sqrt(h.dot(h));
  if (h_norm < EPSILON) {
    return;
  }
  h /= h_norm;

  // 5. Innovation y
  Matrix<float, 3, 1> y = z - h;

  // 6. Jacobian H
  Matrix<float, 3, STATE_DIM> H;  // zeros
  Matrix<float, 3, 4> Hq =
      _computeVectorJacobian(q, _mag_ref_world / h_norm);  // Note: scale doesn't matter since linear
  H.setSlice<3, 4>(0, Q0_IDX, Hq);

  // 7. Measurement noise R
  Matrix<float, 3, 3> R_mat;
  R_mat(0, 0) = _R_mag;
  R_mat(1, 1) = _R_mag;
  R_mat(2, 2) = _R_mag;

  // 8. Innovation covariance S
  Matrix<float, 3, 3> S = H * P_pred * H.transpose() + R_mat;

  // 9. Kalman Gain K
  Matrix<float, STATE_DIM, 3> K = P_pred * H.transpose() * S.invert();

  // 10. Update state
  _x = x_pred + K * y;

  // 11. Update covariance (Joseph form)
  Matrix<float, STATE_DIM, STATE_DIM> I = Matrix<float, STATE_DIM, STATE_DIM>::identity();
  Matrix<float, STATE_DIM, STATE_DIM> I_KH = I - K * H;
  _PCovariance = I_KH * P_pred * I_KH.transpose() + K * R_mat * K.transpose();

  // Enforce symmetry
  _PCovariance = (_PCovariance + _PCovariance.transpose()) * 0.5f;

  // 12. Renormalize quaternion
  Matrix<float, 4, 1> q_new = _x.slice<4, 1>(Q0_IDX, 0);
  q_new = normalizeQuaternion(q_new);
  _x.setSlice<4, 1>(Q0_IDX, 0, q_new);
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