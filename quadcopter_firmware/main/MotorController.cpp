#include "MotorController.h"

#include <Logger.h>

#include "Logger.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000  // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000  // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif

static IRAM_ATTR bool _rmtTxCompleteCallback(
    rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *event_data, void *user_data)
{
  auto *mc = static_cast<MotorController *>(user_data);
  return mc->_handleRmtTxCompleteCallback(channel, event_data);
}

static IRAM_ATTR bool _rmtRxCompleteCallback(
    rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data)
{
  auto *mc = static_cast<MotorController *>(user_data);
  return mc->_handleRmtRxCompleteCallback(channel, event_data);
}

#define RMT_RX_BUF_SIZE 64
MotorController::MotorController(int motorNum, gpio_num_t pin, gpio_num_t telemPin, bool enableTelem)
{
  LOG_INFO("Initializing motor on GPIO %d", pin);
  _pin = pin;
  _telemPin = telemPin;
  _telemEnabled = enableTelem;
  _rmtChannelTx = motorNum;
  _rmtChannelRx = motorNum + 4;
  _rxBuffer = malloc(RMT_RX_BUF_SIZE);
  if (!_rxBuffer) {
    LOG_ERROR("Failed to allocate RMT RX buffer");
  }
  connectRMT();
  LOG_INFO("Successfully initialized motor on GPIO %d", pin);
}

void MotorController::setSpeed(uint16_t speed)
{
  if (_isHalted) {
    return;
  }

  if (speed < MIN_THROTTLE_RANGE || speed > MAX_THROTTLE_RANGE) {
    LOG_WARN("Speed %d is out of range [%d, %d]", speed, MIN_THROTTLE_RANGE, MAX_THROTTLE_RANGE);
    return;
  }

  _writeRawValue(speed);
}

void MotorController::halt(void)
{
  _isHalted = true;

  _writeRawValue(0);
}

void MotorController::disconnectRMT(void)
{
  LOG_INFO("Disconnecting RMT TX channel for GPIO %d", _pin);

  // Disable the RMT channel
  ESP_ERROR_CHECK(rmt_disable(_escChan));

  // Delete the RMT encoder
  ESP_ERROR_CHECK(rmt_del_encoder(_encoder));

  // Delete the RMT channel
  ESP_ERROR_CHECK(rmt_del_channel(_escChan));

  isConnected = false;
}

void MotorController::connectRMT(void)
{
  LOG_INFO("Creating RMT TX channel for GPIO %d", _pin);

  // Configure TX channel
  rmt_tx_channel_config_t tx_chan_config = {
      .gpio_num = _pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_escChan));

  // Set up the dshot encoder
  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = 300000,  // DSHOT300
      .post_delay_us = 50,
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &_encoder));

  rmt_tx_event_callbacks_t txCallbacks = {
      .on_trans_done = _rmtTxCompleteCallback,
      // There are other optional callbacks if needed
  };

  // Register TX event callbacks for this channel, passing "this" as user data
  ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(_escChan, &txCallbacks, this));

  // Enable the TX channel
  ESP_ERROR_CHECK(rmt_enable(_escChan));  // RMT event callback structures

  // If telemetry is enabled, create an RX channel on the telemetry pin
  // or on the same pin if you're using a shared signal line.
  if (_telemEnabled && _telemPin != -1) {
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = _telemPin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .mem_block_symbols = 64,  // enough symbols to capture 16 bits + margins
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &_escRxChan));

    rmt_rx_event_callbacks_t s_rxCbs = {
        .on_recv_done = _rmtRxCompleteCallback,
        // Likewise, other callbacks are optional
    };

    // Register RX event callbacks
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_escRxChan, &s_rxCbs, this));

    // Enable the RX channel (we'll start/stop as needed)
    ESP_ERROR_CHECK(rmt_enable(_escRxChan));
  }

  // Start an infinite transmission of throttle=0 so the motor stays armed
  // or do a single send if you prefer. This ensures there's always a signal.
  _txConfig = {.loop_count = -1};  // infinite loop
  _throttle = {.throttle = 0, .telemetry_req = _telemEnabled};
  ESP_ERROR_CHECK(rmt_transmit(_escChan, _encoder, &_throttle, sizeof(_throttle), &_txConfig));

  isConnected = true;
}

// Callback function invoked when an RMT DShot300 packet transmission is complete
bool IRAM_ATTR
MotorController::_handleRmtTxCompleteCallback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *event_data)
{
  if (!_telemEnabled) {
    return false;  // no telemetry
  }
  if (channel != _escChan) {
    return false;  // not our channel
  }
  // Start RX so we can capture the telemetry reply.
  // Non-blocking: the RX driver captures in the background and
  // will call _rmtRxCompleteCallback() once data is ready.
  if (_escRxChan) {
    ESP_ERROR_CHECK(rmt_enable(_escRxChan));
    rmt_receive_config_t recv_config = {
        .signal_range_min_ns = 0,
        .signal_range_max_ns = 0,
    };

    // Provide a pointer to our buffer and its size in bytes:
    ESP_ERROR_CHECK(rmt_receive(_escRxChan, _rxBuffer, sizeof(_rxBuffer), &recv_config));
  }

  return false;  // Return whether an RTOS yield is needed from ISR; typically false
}

// RX callback: called when we detect a complete frame or idle
bool IRAM_ATTR
MotorController::_handleRmtRxCompleteCallback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata)
{
  if (channel != _escRxChan) {
    return false;  // not our channel
  }

  // edata->received_symbols is an array of RMT items
  // edata->num_symbols is how many items were captured
  if ((edata->received_symbols == nullptr) || (edata->num_symbols == 0)) {
    LOG_ERROR("No telemetry data received");
    // Next time TX fires, we'll try again
    return false;
  }

  // Convert RMT items to 16-bit raw telemetry
  uint16_t rawTelemetry = 0;
  const int BIT_COUNT = 16;
  int itemsCount = edata->num_symbols;
  auto *items = edata->received_symbols;

  for (int i = 0; i < BIT_COUNT && i < itemsCount; ++i) {
    // DShot: '1' => higher duration0 than duration1, '0' => shorter
    bool bitValue = (items[i].duration0 > items[i].duration1);
    rawTelemetry = (rawTelemetry << 1) | (bitValue ? 1 : 0);
  }
  ESP_ERROR_CHECK(rmt_disable(_escRxChan));

  // Process telemetry in user code
  _processTelemetryPacket(rawTelemetry);

  return false;  // no need to yield from ISR
}

void MotorController::_writeRawValue(uint16_t value)
{
  if (!isConnected) {
    // this is expected if the RMT channel is being used for something else
    // so no need to log an error/warning here
    return;
  }
  _throttle.throttle = value;
  ESP_ERROR_CHECK(rmt_transmit(_escChan, _encoder, &_throttle, sizeof(_throttle), &_txConfig));
  ESP_ERROR_CHECK(rmt_disable(_escChan));
  ESP_ERROR_CHECK(rmt_enable(_escChan));
}

void MotorController::_processTelemetryPacket(uint16_t packet)
{
  // Separate the 4-bit CRC from the 12-bit data
  uint8_t receivedCrc = packet & 0x0F;
  uint16_t data12 = packet >> 4;  // 12-bit data (includes type and value)
  // Compute expected CRC from data bits&#8203;:contentReference[oaicite:5]{index=5}
  uint8_t calcCrc = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0x0F;
  if (calcCrc != receivedCrc) {
    LOG_ERROR("Telemetry CRC mismatch: recv=0x%X calc=0x%X", receivedCrc, calcCrc);
    return;
  }

  // Identify frame type and parse accordingly (Extended DShot Telemetry spec)
  uint8_t frameType = (data12 >> 8) & 0x0F;  // top 4 bits determine type
  uint8_t value = data12 & 0xFF;             // lower 8 bits of data field
  switch (frameType) {
    case 0x2:  // 0b0010: Temperature frame&#8203;:contentReference[oaicite:6]{index=6}
      telemetry.temperature = value;
      break;
    case 0x4:  // 0b0100: Voltage frame (0.25V units)&#8203;:contentReference[oaicite:7]{index=7}
      telemetry.voltage = value * 0.25f;
      break;
    case 0x6:  // 0b0110: Current frame (1A units)&#8203;:contentReference[oaicite:8]{index=8}
      telemetry.current = value * 1.0f;
      break;
    case 0x8:  // 0b1000: Debug frame 1 (not used in our case)
      // We can ignore or store debug value if needed
      break;
    case 0xA:  // 0b1010: Debug frame 2 (not used)
      break;
    case 0xC:  // 0b1100: Stress level frame (since EDT v2.0)
      // Not explicitly requested, but we can store max stress level if needed
      // e.g., telemetryData.maxStress = value;
      break;
    case 0xE:  // 0b1110: Status frame (error/warning/alert flags)&#8203;:contentReference[oaicite:9]{index=9}
    {
      // Bits: [7]=alert, [6]=warning, [5]=error, [3:0]=max stress (if v2.0)
      bool alert = (value & 0x80) != 0;
      bool warning = (value & 0x40) != 0;
      bool error = (value & 0x20) != 0;
      telemetry.errorFlags = 0;
      if (error) telemetry.errorFlags |= 0x01;
      if (warning) telemetry.errorFlags |= 0x02;
      if (alert) telemetry.errorFlags |= 0x04;
      // (Optional: parse max stress level: value & 0x0F)
      break;
    }
    default:
      // If frameType doesn’t match Extended Telemetry, treat as standard eRPM
      // Standard DShot telemetry eRPM frame: 3-bit exponent (bits 11-9 of data12) and 9-bit
      // mantissa&#8203;:contentReference[oaicite:10]{index=10}.
      uint8_t exp = data12 >> 9;           // exponent (upper 3 bits of 12-bit data)
      uint16_t mantissa = data12 & 0x1FF;  // lower 9 bits
      uint32_t eRPM =
          mantissa * (1 << exp);       // eRPM = mantissa << exponent&#8203;:contentReference[oaicite:11]{index=11}
      telemetry.rpm = (uint16_t)eRPM;  // store eRPM (note: actual RPM depends on motor pole count)
      break;
  }

  LOG_INFO_PERIODIC_MILLIS(
      1000,
      "Motor %d Telemetry: V=%.2fV, I=%.2fA, T=%d°C, RPM=%d, Flags=0x%02X",
      _rmtChannelTx,
      telemetry.voltage,
      telemetry.current,
      telemetry.temperature,
      telemetry.rpm,
      telemetry.errorFlags);
}