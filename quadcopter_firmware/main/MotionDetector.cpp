#include "MotionDetector.h"

#include <Logger.h>

#include <cmath>  // For std::sqrt, std::fabs

// --- Constructor Implementations ---

// Constructor taking an explicit Config object
MotionDetector::MotionDetector(const Config& config)
    : _config(config),  // Copy config
      _gyro_filter_x(_config.median_filter_window_size),
      _gyro_filter_y(_config.median_filter_window_size),
      _gyro_filter_z(_config.median_filter_window_size),
      _accel_filter_x(_config.median_filter_window_size),
      _accel_filter_y(_config.median_filter_window_size),
      _accel_filter_z(_config.median_filter_window_size),
      _gyro_motion_threshold_dps_sq(_config.gyro_motion_threshold_dps * _config.gyro_motion_threshold_dps)
{
  // Common initialization logic (can also be put in a private init function)
  isInMotion = false;
  isUpright = false;
  _motion_start_detect_micros = 0;
  _stationary_start_detect_micros = 0;
  _upright_start_detect_micros = 0;
  _not_upright_start_detect_micros = 0;
  _sample_count = 0;
  _filters_initialized = false;
}

// Default constructor: Delegates to the specific constructor using a default Config
MotionDetector::MotionDetector() : MotionDetector(Config())
{
  // The body is often empty when using constructor delegation,
  // as the target constructor does all the work.
}

// --- Method Implementations (checkMotionCondition, checkUprightCondition, imuUpdate remain the same) ---

bool MotionDetector::checkMotionCondition(float gx, float gy, float gz, float ax, float ay, float az)
{
  float gyro_mag_sq = (gx * gx) + (gy * gy) + (gz * gz);
  bool high_gyro = (gyro_mag_sq > _gyro_motion_threshold_dps_sq);

  float accel_mag = std::sqrt((ax * ax) + (ay * ay) + (az * az));
  bool high_accel_dev =
      (std::fabs(accel_mag - _config.accel_stationary_magnitude_g) > _config.accel_mag_deviation_threshold_g);

  return high_gyro || high_accel_dev;
}

bool MotionDetector::checkUprightCondition(float ax, float ay, float az)
{
  // ... (implementation is unchanged) ...
  bool xy_ok = (std::fabs(ax) <= _config.accel_upright_xy_threshold_g) &&
               (std::fabs(ay) <= _config.accel_upright_xy_threshold_g);

  bool z_ok = (az >= _config.accel_upright_z_min_g) && (az <= _config.accel_upright_z_max_g);

  return xy_ok && z_ok;
}

void MotionDetector::imuUpdate(const imu_update_t& data, uint64_t current_micros)
{
  // ... (implementation is unchanged) ...

  // 1. Add data to filters
  _gyro_filter_x.addValue(data.gyro_x);
  _gyro_filter_y.addValue(data.gyro_y);
  _gyro_filter_z.addValue(data.gyro_z);
  _accel_filter_x.addValue(data.accel_x);
  _accel_filter_y.addValue(data.accel_y);
  _accel_filter_z.addValue(data.accel_z);

  _sample_count++;

  // 2. Check filter initialization
  if (!_filters_initialized && _sample_count >= _config.median_filter_window_size) {
    _filters_initialized = true;
    _stationary_start_detect_micros = current_micros;   // Assume starting stationary
    _not_upright_start_detect_micros = current_micros;  // Assume starting not upright
  }

  if (!_filters_initialized) {
    LOG_ERROR_PERIODIC_MILLIS(1000, "MotionDetector: Filters not initialized yet");
    return;
  }

  // 3. Get filtered values
  float filt_gx = _gyro_filter_x.getMedian();
  float filt_gy = _gyro_filter_y.getMedian();
  float filt_gz = _gyro_filter_z.getMedian();
  float filt_ax = _accel_filter_x.getMedian();
  float filt_ay = _accel_filter_y.getMedian();
  float filt_az = _accel_filter_z.getMedian();

  // 4. Check current conditions based on filtered data
  bool motion_condition_met = checkMotionCondition(filt_gx, filt_gy, filt_gz, filt_ax, filt_ay, filt_az);
  bool upright_condition_met = checkUprightCondition(filt_ax, filt_ay, filt_az);

  // Initialize timers on the very first valid filtered data check if not already set
  // (Removed redundant init logic here, handled at _filters_initialized transition)

  // 5. Apply Motion Detection Hysteresis (Time-based)
  if (motion_condition_met) {
    if (_motion_start_detect_micros == 0) _motion_start_detect_micros = current_micros;
    _stationary_start_detect_micros = 0;
    if (_motion_start_detect_micros != 0 &&  // Ensure timer has started
        current_micros - _motion_start_detect_micros >= _config.motion_confirm_duration_us) {
      isInMotion = true;
    }
  } else {
    if (_stationary_start_detect_micros == 0) _stationary_start_detect_micros = current_micros;
    _motion_start_detect_micros = 0;
    if (_stationary_start_detect_micros != 0 &&  // Ensure timer has started
        current_micros - _stationary_start_detect_micros >= _config.stationary_confirm_duration_us) {
      isInMotion = false;
    }
  }

  // 6. Apply Upright Detection Hysteresis (Time-based)
  if (upright_condition_met) {
    if (_upright_start_detect_micros == 0) _upright_start_detect_micros = current_micros;
    _not_upright_start_detect_micros = 0;
    if (_upright_start_detect_micros != 0 &&  // Ensure timer has started
        current_micros - _upright_start_detect_micros >= _config.upright_confirm_duration_us) {
      isUpright = true;
    }
  } else {
    if (_not_upright_start_detect_micros == 0) _not_upright_start_detect_micros = current_micros;
    _upright_start_detect_micros = 0;
    if (_not_upright_start_detect_micros != 0 &&  // Ensure timer has started
        current_micros - _not_upright_start_detect_micros >= _config.not_upright_confirm_duration_us) {
      isUpright = false;
    }
  }
}