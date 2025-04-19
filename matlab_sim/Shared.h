#pragma once 

#include <cstdint>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define DEG_TO_RAD(x) (x * (M_PI / 180.0))

struct imu_update_t
{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float heading;
};

void setFakeTime(uint64_t time);

uint64_t millis(void);

uint64_t micros(void);