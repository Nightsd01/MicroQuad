#pragma once

#define MIN_THROTTLE_RANGE 48
#define MAX_THROTTLE_RANGE 2047

#include <dshot_esc_encoder.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

enum MotorIODirection
{
  unset = 0,
  transmit = 1,
  receive = 2
};

class MotorController
{
 public:
  MotorController(gpio_num_t pin);

  ~MotorController();

  // Range from MIN_THROTTLE_RANGE to MAX_THROTTLE_RANGE
  void setSpeed(uint16_t speed);

  // Permanently stops the motor until device reset. Use with emergency mode
  void halt(void);

  void disconnectRMT(void);

  void connectRMT(void);

  bool isConnected;

  // We need to make these members public so that they can be statically accessed by the ISR callbacks
  void _handleRxCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata);
  void _handleTxComplete(void);

 private:
  gpio_num_t _pin;
  dshot_esc_throttle_t _throttle;
  rmt_channel_handle_t _escChan;
  rmt_encoder_handle_t _encoder;
  rmt_transmit_config_t _txConfig;
  rmt_channel_handle_t _escRxChan;
  bool _isHalted = false;
  void _writeRawValue(uint16_t value);
  static const size_t RMT_RX_BUFFER_SIZE = 48;  // Adjust based on your needs
  rmt_symbol_word_t _rxBuffer[RMT_RX_BUFFER_SIZE];
  rmt_receive_config_t _rxConfig;
  MotorIODirection _currentDirection = MotorIODirection::unset;

  bool _rxInProgress = false;
  bool _txInProgress = false;
  uint16_t _lastSpeed = 0;
  bool _receivedSpeedUpdate = false;
  void _switchMotorIODirection(MotorIODirection direction);
};