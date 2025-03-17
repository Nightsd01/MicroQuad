#include "MahonyAHRS.h"

#include <algorithm>  // for std::max if needed

// For convenience, define M_PI if not present
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------------------------
/**
 * @brief Helper function to apply a 3x3 misalignment matrix to a 3D vector.
 *
 * @param v    Input vector (x,y,z)
 * @param mat  3x3 misalignment matrix in row-major order
 * @return     Transformed vector mat * v
 */
static Vector3f applyMisalignment(const Vector3f& v, const float mat[3][3])
{
  return {
      .x = mat[0][0] * v.x + mat[0][1] * v.y + mat[0][2] * v.z,
      .y = mat[1][0] * v.x + mat[1][1] * v.y + mat[1][2] * v.z,
      .z = mat[2][0] * v.x + mat[2][1] * v.y + mat[2][2] * v.z};
}

//------------------------------------------------------------------------------
MahonyAHRS::MahonyAHRS(float kp, float ki)
    : _twoKp(2.0f * kp),
      _twoKi(2.0f * ki),
      _q0(1.0f),
      _q1(0.0f),
      _q2(0.0f),
      _q3(0.0f),
      _integralFBx(0.0f),
      _integralFBy(0.0f),
      _integralFBz(0.0f)
{
  // Initialize each misalignment matrix to identity by default
  const float identity[3][3] = {
      {1.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 1.0f},
  };
  setGyroMisalignment(identity);
  setAccelMisalignment(identity);
  setMagMisalignment(identity);
}

//------------------------------------------------------------------------------
void MahonyAHRS::setGyroMisalignment(const float mat[3][3]) { std::memcpy(_gyroMisalign, mat, 9 * sizeof(float)); }

//------------------------------------------------------------------------------
void MahonyAHRS::setAccelMisalignment(const float mat[3][3]) { std::memcpy(_accelMisalign, mat, 9 * sizeof(float)); }

//------------------------------------------------------------------------------
void MahonyAHRS::setMagMisalignment(const float mat[3][3]) { std::memcpy(_magMisalign, mat, 9 * sizeof(float)); }

//------------------------------------------------------------------------------
void MahonyAHRS::update(const Vector3f& gyroDPS, const Vector3f& accelG, const Vector3f& magData, float dt)
{
  //-----------------------------------
  // 1. Apply misalignment correction
  //-----------------------------------
  Vector3f gyroCorr = applyMisalignment(gyroDPS, _gyroMisalign);
  Vector3f accelCorr = applyMisalignment(accelG, _accelMisalign);
  Vector3f magCorr = applyMisalignment(magData, _magMisalign);

  //-----------------------------------
  // 2. Convert gyro from deg/s to rad/s
  //-----------------------------------
  float gx = gyroCorr.x * (M_PI / 180.0f);
  float gy = gyroCorr.y * (M_PI / 180.0f);
  float gz = gyroCorr.z * (M_PI / 180.0f);

  //-----------------------------------
  // 3. Normalize accelerometer (if valid)
  //-----------------------------------
  float ax = accelCorr.x;
  float ay = accelCorr.y;
  float az = accelCorr.z;
  float normA = std::sqrt(ax * ax + ay * ay + az * az);
  if (normA > 1.0e-6f) {
    ax /= normA;
    ay /= normA;
    az /= normA;
  } else {
    // Accel invalid, do minimal gyro integration
    float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    float qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    float qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    _q0 += qDot0 * dt;
    _q1 += qDot1 * dt;
    _q2 += qDot2 * dt;
    _q3 += qDot3 * dt;

    float nq = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 /= nq;
    _q1 /= nq;
    _q2 /= nq;
    _q3 /= nq;
    return;
  }

  //-----------------------------------
  // 4. Normalize magnetometer (if valid)
  //-----------------------------------
  float mx = magCorr.x;
  float my = magCorr.y;
  float mz = magCorr.z;
  float normM = std::sqrt(mx * mx + my * my + mz * mz);
  if (normM > 1.0e-6f) {
    mx /= normM;
    my /= normM;
    mz /= normM;
  } else {
    // Mag invalid => 6DOF correction only (gyro+accel)
    // Compute error between estimated gravity and measured gravity
    float vx = 2.0f * (_q1 * _q3 - _q0 * _q2);
    float vy = 2.0f * (_q0 * _q1 + _q2 * _q3);
    float vz = (_q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3);

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      _integralFBx += _twoKi * ex * dt;
      _integralFBy += _twoKi * ey * dt;
      _integralFBz += _twoKi * ez * dt;

      gx += _integralFBx;
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f;
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Proportional feedback
    gx += _twoKp * ex;
    gy += _twoKp * ey;
    gz += _twoKp * ez;

    // Integrate
    float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    float qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    float qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    _q0 += qDot0 * dt;
    _q1 += qDot1 * dt;
    _q2 += qDot2 * dt;
    _q3 += qDot3 * dt;

    float nq = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 /= nq;
    _q1 /= nq;
    _q2 /= nq;
    _q3 /= nq;
    return;
  }

  //-----------------------------------
  // 5. Estimated direction of gravity
  //-----------------------------------
  float vx = 2.0f * (_q1 * _q3 - _q0 * _q2);
  float vy = 2.0f * (_q0 * _q1 + _q2 * _q3);
  float vz = (_q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3);

  //-----------------------------------
  // 6. Reference direction of Earth's magnetic field
  //    (Following typical Mahony/Madgwick approach)
  //-----------------------------------
  float q0q0 = _q0 * _q0;
  float q1q1 = _q1 * _q1;
  float q2q2 = _q2 * _q2;
  float q3q3 = _q3 * _q3;
  float q0q1 = _q0 * _q1;
  float q0q2 = _q0 * _q2;
  float q0q3 = _q0 * _q3;
  float q1q2 = _q1 * _q2;
  float q1q3 = _q1 * _q3;
  float q2q3 = _q2 * _q3;

  // Compute intermediate values
  float hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
  float hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
  float hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

  // We define bx, bz as the horizontal and vertical components
  float bx = std::sqrt(hx * hx + hy * hy);
  float bz = hz;

  // Estimated direction of mag field in sensor frame
  float wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  float wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  float wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

  //-----------------------------------
  // 7. Compute combined error (accel + mag)
  //-----------------------------------
  // Accel error = cross(acc, v)
  float ex_acc = (ay * vz - az * vy);
  float ey_acc = (az * vx - ax * vz);
  float ez_acc = (ax * vy - ay * vx);

  // Mag error = cross(mag, w)
  float ex_mag = (my * wz - mz * wy);
  float ey_mag = (mz * wx - mx * wz);
  float ez_mag = (mx * wy - my * wx);

  // Total error
  float ex = ex_acc + ex_mag;
  float ey = ey_acc + ey_mag;
  float ez = ez_acc + ez_mag;

  //-----------------------------------
  // 8. Apply integral feedback if enabled
  //-----------------------------------
  if (_twoKi > 0.0f) {
    _integralFBx += _twoKi * ex * dt;
    _integralFBy += _twoKi * ey * dt;
    _integralFBz += _twoKi * ez * dt;

    gx += _integralFBx;
    gy += _integralFBy;
    gz += _integralFBz;
  } else {
    _integralFBx = 0.0f;
    _integralFBy = 0.0f;
    _integralFBz = 0.0f;
  }

  //-----------------------------------
  // 9. Apply proportional feedback
  //-----------------------------------
  gx += _twoKp * ex;
  gy += _twoKp * ey;
  gz += _twoKp * ez;

  //-----------------------------------
  // 10. Integrate the rate of change of the quaternion
  //-----------------------------------
  float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
  float qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
  float qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
  float qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

  _q0 += qDot0 * dt;
  _q1 += qDot1 * dt;
  _q2 += qDot2 * dt;
  _q3 += qDot3 * dt;

  //-----------------------------------
  // 11. Normalize the quaternion
  //-----------------------------------
  float nq = std::sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
  if (nq > 1.0e-6f) {
    _q0 /= nq;
    _q1 /= nq;
    _q2 /= nq;
    _q3 /= nq;
  }
}

//------------------------------------------------------------------------------
void MahonyAHRS::getYawPitchRoll(float& yawDeg, float& pitchDeg, float& rollDeg) const
{
  // Convert quaternion to Euler angles in radians:
  // Yaw (Z), Pitch (Y), Roll (X) in aerospace convention
  float ysqr = _q2 * _q2;

  // Roll (x-axis rotation)
  float t0 = +2.0f * (_q0 * _q1 + _q2 * _q3);
  float t1 = +1.0f - 2.0f * (_q1 * _q1 + ysqr);
  float roll = std::atan2(t0, t1);

  // Pitch (y-axis rotation)
  float t2 = +2.0f * (_q0 * _q2 - _q3 * _q1);
  t2 = (t2 > 1.0f) ? 1.0f : t2;
  t2 = (t2 < -1.0f) ? -1.0f : t2;
  float pitch = std::asin(t2);

  // Yaw (z-axis rotation)
  float t3 = +2.0f * (_q0 * _q3 + _q1 * _q2);
  float t4 = +1.0f - 2.0f * (ysqr + _q3 * _q3);
  float yaw = std::atan2(t3, t4);

  // Convert radians to degrees
  rollDeg = roll * (180.0f / M_PI);
  pitchDeg = pitch * (180.0f / M_PI);
  yawDeg = yaw * (180.0f / M_PI);

  // Optionally, normalize yaw to [0..360)
  while (yawDeg < 0.0f) yawDeg += 360.0f;
  while (yawDeg >= 360.0f) yawDeg -= 360.0f;
}
