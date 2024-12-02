#pragma once

template <typename T>
class KalmanFilter
{
 public:
  KalmanFilter(float q, float r, T x, float p, float k) : _q(q), _r(r), _x(x), _p(p), _k(k) {};

  T applyFilter(const T& measurement);

 private:
  float _q;  // Process noise covariance
  float _r;  // Measurement noise covariance
  T _x;      // Value
  float _p;  // Estimation error covariance
  float _k;  // Kalman gain
};