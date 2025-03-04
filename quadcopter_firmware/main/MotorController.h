#pragma once

#define MIN_THROTTLE_RANGE 48
#define MAX_THROTTLE_RANGE 2047

#include <dshot_esc_encoder.h>

#include "driver/rmt_common.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

/** Telemetry data structure for DShot telemetry (voltage, current, etc.) */
struct TelemetryData
{
  float voltage;        // ESC voltage in Volts (0.25 V resolution)
  float current;        // ESC current in Amps (1 A resolution)
  uint8_t temperature;  // ESC temperature in °C
  uint16_t rpm;         // Motor eRPM (electrical RPM) from telemetry
  uint8_t errorFlags;   // Error/Status flags: bit0=error, bit1=warning, bit2=alert
};

class MotorController
{
 public:
  MotorController(int motorNum, gpio_num_t pin, gpio_num_t telemPin = (gpio_num_t)-1, bool enableTelem = false);

  // Range from MIN_THROTTLE_RANGE to MAX_THROTTLE_RANGE
  void setSpeed(uint16_t speed);

  // Permanently stops the motor until device reset. Use with emergency mode
  void halt(void);

  void disconnectRMT(void);

  void connectRMT(void);

  bool isConnected;

  TelemetryData telemetry;

  // Callback for RMT transmission completion (to handle telemetry)
  bool _handleRmtTxCompleteCallback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *event_data);
  bool _handleRmtRxCompleteCallback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata);

  void *_rxBuffer;
  gpio_num_t _pin;
  gpio_num_t _telemPin;
  bool _telemEnabled;
  dshot_esc_throttle_t _throttle;
  rmt_channel_handle_t _escChan;
  rmt_channel_handle_t _escRxChan;
  rmt_encoder_handle_t _encoder;
  rmt_transmit_config_t _txConfig;

  // RMT channel IDs for TX (and RX if using separate channel for telemetry)
  int _rmtChannelTx;
  int _rmtChannelRx;  // RMT channel for telemetry receive (if needed)

  bool _isHalted = false;
  void _writeRawValue(uint16_t value);

  // Helper to process a raw 16-bit telemetry packet and update telemetry struct
  void _processTelemetryPacket(uint16_t packet);
};