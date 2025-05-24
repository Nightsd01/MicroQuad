#pragma once

#include <Matrix.h>

#ifndef MATLAB_SIM
#include "Arduino.h"
#include "QuadcopterController.h"

#define LED_DATA_PIN GPIO_NUM_38
#define BATTERY_SENSE_PIN GPIO_NUM_9
#define BATTERY_STAT1_PIN GPIO_NUM_37
#define BATTERY_STAT2_PIN GPIO_NUM_36
#define BATTERY_PG_PIN GPIO_NUM_35
#define BATTERY_CHARGE_ENABLE_PIN GPIO_NUM_2
#define SPI0_CS_PIN GPIO_NUM_10
#define SPI0_MOSI_PIN GPIO_NUM_11
#define SPI0_SCLK_PIN GPIO_NUM_12
#define SPI0_MISO_PIN GPIO_NUM_13
#define IMU_INT_PIN GPIO_NUM_3

const gpio_num_t MOTOR_PINS[NUM_MOTORS] = {GPIO_NUM_8, GPIO_NUM_7, GPIO_NUM_5, GPIO_NUM_6};
const gpio_num_t MOTOR_TELEM_PINS[NUM_MOTORS] = {GPIO_NUM_34, GPIO_NUM_33, GPIO_NUM_4, GPIO_NUM_21};

#endif  // MATLAB_SIM

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

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif  // MAX

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif  // MIN

// declination angle for Foster City, CA
#define DECLINATION_ANGLE_DEG (12.0 + (55.0 / 60.0)) / (180 / PI)

#define STANDARD_GRAVITY 9.81f  // or 9.80665f for higher precision

#define MAGNETIC_REFERENCE_VECTOR {{0.446f}, {0.116f}, {0.887f}}  // adjust for location, this is for Foster City, CA