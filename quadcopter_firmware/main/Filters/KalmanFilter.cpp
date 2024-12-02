#include "KalmanFilter.h"

template <typename T>
T KalmanFilter<T>::applyFilter(const T &measurement)
{
  // Prediction update
  _p = _p + _q;

  // Measurement update
  _k = _p / (_p + _r);
  _x = _x + _k * (measurement - _x);
  _p = (1.0f - _k) * _p;

  return _x;
}

template class KalmanFilter<float>;