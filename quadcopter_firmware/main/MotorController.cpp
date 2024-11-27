#include "MotorController.h"

#include <Arduino.h>
#include <Logger.h>

#include "esp32s3/rom/ets_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000  // 32MHz resolution, DSHot protocol needs a relatively high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000  // 40MHz resolution, DSHot protocol needs a relatively high resolution
#endif

// This is invoked when a telemetry packet is received from the ESC
static bool IRAM_ATTR
_onTelemetryReceivedCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
  AsyncController::main.executeAsync([&]() { LOG_INFO("RX complete"); });
  esp_log_level_set("*", ESP_LOG_NONE);
  // Get a reference to the MotorController object
  MotorController *controller = reinterpret_cast<MotorController *>(user_ctx);
  controller->_handleRxCallback(rx_chan, edata);
  esp_log_level_set("*", ESP_LOG_INFO);
  return true;
}

// This is invoked when a telemetry packet is received from the ESC
static bool _onTxCompleteCallback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
  AsyncController::main.executeAsync([&]() { LOG_INFO("TX complete"); });
  esp_log_level_set("*", ESP_LOG_NONE);
  // Get a reference to the MotorController object
  MotorController *controller = reinterpret_cast<MotorController *>(user_ctx);
  controller->_handleTxComplete();
  esp_log_level_set("*", ESP_LOG_INFO);
  return true;
}

MotorController::~MotorController()
{
  LOG_ERROR("Deallocating motor controller on pin %d", _pin);
  ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
}

void MotorController::_switchMotorIODirection(MotorIODirection direction)
{
  if (direction == MotorIODirection::transmit) {
    if (_currentDirection == MotorIODirection::receive) {
      ESP_ERROR_CHECK(rmt_disable(_escRxChan));
    }
    _currentDirection = MotorIODirection::transmit;
    // Configure TX channel if not already configured
    if (_escChan == nullptr) {
      rmt_tx_channel_config_t tx_chan_config = {
          .gpio_num = _pin,
          .clk_src = RMT_CLK_SRC_DEFAULT,
          .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
          .mem_block_symbols = 48,
          .trans_queue_depth = 1,
      };
      ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_escChan));

      // Register TX callback
      rmt_tx_event_callbacks_t tx_callbacks = {
          .on_trans_done = _onTxCompleteCallback,
      };
      ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(_escChan, &tx_callbacks, this));

      ESP_ERROR_CHECK(rmt_enable(_escChan));
    }
  } else {
    if (_currentDirection == MotorIODirection::transmit) {
      ESP_ERROR_CHECK(rmt_disable(_escChan));
    }
    _currentDirection = MotorIODirection::receive;
    // Configure RX channel if not already configured
    if (_escRxChan == nullptr) {
      rmt_rx_channel_config_t rx_chan_config = {
          .gpio_num = _pin,
          .clk_src = RMT_CLK_SRC_DEFAULT,
          .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
          .mem_block_symbols = RMT_RX_BUFFER_SIZE,
      };
      ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &_escRxChan));

      // Register RX callback
      rmt_rx_event_callbacks_t rx_callbacks = {
          .on_recv_done = _onTelemetryReceivedCallback,
      };
      ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_escRxChan, &rx_callbacks, this));
      ESP_ERROR_CHECK(rmt_enable(_escRxChan));
    }
  }
}

void MotorController::_handleTxComplete(void)
{
  // Enable RX for telemetry after TX completes
  _rxInProgress = true;
  _switchMotorIODirection(MotorIODirection::receive);
  esp_err_t rx_err = rmt_receive(_escRxChan, _rxBuffer, sizeof(_rxBuffer), &_rxConfig);
  if (rx_err != ESP_OK) {
    LOG_ERROR_ASYNC_ON_MAIN("Failed to enable RX: %s", esp_err_to_name(rx_err));
  }
  _txInProgress = false;
  AsyncController::main.executeAsync([&]() { LOG_INFO("Waiting for RX complete"); });
}

void MotorController::_handleRxCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata)
{
  _switchMotorIODirection(MotorIODirection::transmit);
  if (!edata || edata->num_symbols < 21) {
    _rxInProgress = false;
    if (_receivedSpeedUpdate) {
      _receivedSpeedUpdate = false;
      _writeRawValue(_lastSpeed);
    }
    return;
  }

  // Process telemetry data...
  uint32_t raw_data = 0;
  for (int i = 0; i < 21; i++) {
    const rmt_symbol_word_t *symbol = &edata->received_symbols[i];
    if (symbol->duration0 > (DSHOT_ESC_RESOLUTION_HZ / 400000)) {
      raw_data = (raw_data << 1) | 1;
    } else {
      raw_data = (raw_data << 1) | 0;
    }
  }

  uint16_t received_crc = raw_data & 0xF;
  uint16_t data = (raw_data >> 4) & 0xFFFF;

  _rxInProgress = false;

  if (_receivedSpeedUpdate) {
    _receivedSpeedUpdate = false;
    _writeRawValue(_lastSpeed);
  }
}

MotorController::MotorController(gpio_num_t pin)
{
  _pin = pin;
  LOG_INFO("Initializing motor on GPIO %d", pin);
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
  LOG_INFO("Disconnecting RMT TX and RX channels for GPIO %d", _pin);

  if (_escRxChan != nullptr) {
    ESP_ERROR_CHECK(rmt_disable(_escRxChan));
    ESP_ERROR_CHECK(rmt_del_channel(_escRxChan));
    _escRxChan = nullptr;
  }
  if (_escChan != nullptr) {
    ESP_ERROR_CHECK(rmt_disable(_escChan));
    ESP_ERROR_CHECK(rmt_del_channel(_escChan));
    _escChan = nullptr;
  }

  // Delete the RMT encoder
  ESP_ERROR_CHECK(rmt_del_encoder(_encoder));

  isConnected = false;
  _currentDirection = MotorIODirection::unset;
}

void MotorController::connectRMT(void)
{
  LOG_INFO("Creating RMT TX channel for GPIO %d", _pin);
  _escChan = nullptr;
  _escRxChan = nullptr;

  _switchMotorIODirection(MotorIODirection::transmit);

  LOG_INFO("Installing DShot ESC Encoder for GPIO %d", _pin);
  _encoder = nullptr;
  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = 300000,
      .post_delay_us = 50,
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &_encoder));

  _txConfig = {
      .loop_count = 0,
  };
  _rxConfig = {
      .signal_range_min_ns = 1250,  // Optimized for DShot300
      .signal_range_max_ns = 3750,  // Optimized for DShot300
  };
  _throttle = {
      .throttle = 0,
      .telemetry_req = true,
  };

  isConnected = true;
}

void MotorController::_writeRawValue(uint16_t value)
{
  if (!isConnected) {
    LOG_WARN_PERIODIC_MILLIS(500, "Cannot write value, is not connected to GPIO %d", _pin);
    return;
  }

  if (_txInProgress || _rxInProgress) {
    LOG_INFO_PERIODIC_MILLIS(
        100,
        "TX (%d) or RX (%d) in progress, skipping write for pin %d",
        _txInProgress,
        _rxInProgress,
        _pin);
    _lastSpeed = value;
    _receivedSpeedUpdate = true;
    return;
  }

  LOG_INFO_PERIODIC_MILLIS(100, "Writing value %d to GPIO %d", value, _pin);

  // Update throttle value and request telemetry
  _throttle.throttle = value;
  _throttle.telemetry_req = true;

  // Just transmit - the callback will handle enabling RX
  _txInProgress = true;

  esp_err_t tx_err = rmt_transmit(_escChan, _encoder, &_throttle, sizeof(_throttle), &_txConfig);
  if (tx_err != ESP_OK) {
    _txInProgress = false;
    LOG_ERROR_PERIODIC_MILLIS(500, "Failed to transmit to ESC on GPIO %d: %s", _pin, esp_err_to_name(tx_err));
    return;
  }
}