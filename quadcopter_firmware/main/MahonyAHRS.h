#ifndef MAHONY_AHRS_HPP
#define MAHONY_AHRS_HPP

#include <cmath>
#include <cstring>  // for std::memcpy

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

/**
 * @brief MahonyAHRS class for 9DOF sensor fusion (gyro + accel + mag),
 *        with optional misalignment correction for each sensor.
 */
class MahonyAHRS
{
 public:
  /**
   * @param kp  Proportional gain (e.g., 0.5)
   * @param ki  Integral gain (e.g., 0.0)
   */
  MahonyAHRS(float kp = 0.5f, float ki = 0.0f);

  /**
   * @brief Sets the misalignment matrix for gyro data.
   *        Must be a 3x3 float array, row-major:
   *        M = [ [m00, m01, m02],
   *              [m10, m11, m12],
   *              [m20, m21, m22] ]
   */
  void setGyroMisalignment(const float mat[3][3]);

  /**
   * @brief Sets the misalignment matrix for accelerometer data.
   */
  void setAccelMisalignment(const float mat[3][3]);

  /**
   * @brief Sets the misalignment matrix for magnetometer data.
   */
  void setMagMisalignment(const float mat[3][3]);

  /**
   * @brief Update the filter with new 9DOF sensor readings.
   *
   * @param gyroDPS   Gyroscope reading in deg/s (x,y,z)
   * @param accelG    Accelerometer reading in g (x,y,z)
   * @param mag       Magnetometer reading (x,y,z) in e.g. µT or gauss
   * @param dt        Delta time (seconds) since last update
   *
   * @note The filter internally:
   *       1) Applies the misalignment matrices.
   *       2) Converts gyro to rad/s.
   *       3) Normalizes accel & mag (if valid).
   *       4) Computes the Mahony correction for both gravity & mag.
   *       5) Integrates the quaternion.
   *       6) Normalizes the quaternion.
   */
  void update(const Vector3f& gyroDPS, const Vector3f& accelG, const Vector3f& mag, float dt);

  /**
   * @brief Get the current estimated orientation as yaw, pitch, roll in degrees.
   * @param[out] yawDeg   Yaw in degrees
   * @param[out] pitchDeg Pitch in degrees
   * @param[out] rollDeg  Roll in degrees
   */
  void getYawPitchRoll(float& yawDeg, float& pitchDeg, float& rollDeg) const;

 private:
  // Internal parameters
  float _twoKp;  ///< 2 * proportional gain
  float _twoKi;  ///< 2 * integral gain

  // Quaternion of sensor frame relative to Earth frame
  float _q0, _q1, _q2, _q3;

  // Integral error terms (gyro bias correction)
  float _integralFBx;
  float _integralFBy;
  float _integralFBz;

  // Misalignment matrices (3x3) for each sensor
  float _gyroMisalign[3][3];
  float _accelMisalign[3][3];
  float _magMisalign[3][3];
};

#endif  // MAHONY_AHRS_HPP
