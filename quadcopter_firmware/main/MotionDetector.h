#pragma once

#include <cstddef>  // For size_t
#include <cstdint>  // For uint64_t

#include "IMU.h"           // Include your IMU data structure definition
#include "MedianFilter.h"  // Include your MedianFilter implementation

class MotionDetector
{
 public:
  // --- Public State Variables ---
  bool isInMotion = false;  // Is the device currently considered in motion?
  bool isUpright = false;   // Is the device considered upright (Z-axis aligned with gravity)?

  // --- Configuration Struct (Keep default initializers here) ---
  struct Config
  {
    // Filter settings
    size_t median_filter_window_size = 5;

    // Motion Detection Thresholds (apply to filtered data)
    float gyro_motion_threshold_dps = 1.5f;         // Rotation rate threshold
    float accel_stationary_magnitude_g = 1.0f;      // Expected gravity
    float accel_mag_deviation_threshold_g = 0.05f;  // Allowed deviation from 1G when stationary

    // Upright Detection Thresholds (apply to filtered data)
    float accel_upright_xy_threshold_g = 0.15f;  // Max allowed G on X/Y plane
    float accel_upright_z_min_g = 0.85f;         // Min Z G when upright
    float accel_upright_z_max_g = 1.15f;         // Max Z G when upright

    // Hysteresis Durations (in microseconds)
    uint64_t motion_confirm_duration_us = 200000;       // 0.2 seconds
    uint64_t stationary_confirm_duration_us = 1000000;  // 1.0 seconds
    uint64_t upright_confirm_duration_us = 500000;      // 0.5 seconds
    uint64_t not_upright_confirm_duration_us = 500000;  // 0.5 seconds

    // NOTE: No explicit default constructor needed here if members have initializers (C++11 onwards)
  };

  // --- Constructor Overloads ---
  explicit MotionDetector(const Config& config);  // Constructor taking explicit config
  MotionDetector();                               // Default constructor (will use default Config)

  // --- Update Function ---
  void imuUpdate(const imu_update_t& data, uint64_t current_micros);

 private:
  // --- Private Members (remain the same) ---
  Config _config;

  MedianFilter<float> _gyro_filter_x;
  MedianFilter<float> _gyro_filter_y;
  MedianFilter<float> _gyro_filter_z;
  MedianFilter<float> _accel_filter_x;
  MedianFilter<float> _accel_filter_y;
  MedianFilter<float> _accel_filter_z;

  uint64_t _motion_start_detect_micros = 0;
  uint64_t _stationary_start_detect_micros = 0;
  uint64_t _upright_start_detect_micros = 0;
  uint64_t _not_upright_start_detect_micros = 0;

  int _sample_count = 0;
  bool _filters_initialized = false;
  float _gyro_motion_threshold_dps_sq;

  bool checkMotionCondition(float gx, float gy, float gz, float ax, float ay, float az);
  bool checkUprightCondition(float ax, float ay, float az);
};