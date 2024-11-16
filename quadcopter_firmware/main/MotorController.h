#pragma once

#define MIN_THROTTLE_RANGE 48
#define MAX_THROTTLE_RANGE 2047

#include <dshot_esc_encoder.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

class MotorController
{
 public:
  MotorController(gpio_num_t pin);

  // Range from MIN_THROTTLE_RANGE to MAX_THROTTLE_RANGE
  void setSpeed(uint16_t speed);

  // Permanently stops the motor until device reset. Use with emergency mode
  void halt(void);

 private:
  dshot_esc_throttle_t _throttle;
  rmt_channel_handle_t _escChan;
  rmt_encoder_handle_t _encoder;
  rmt_transmit_config_t _txConfig;
  bool _isHalted = false;
  void _writeRawValue(uint16_t value);
};