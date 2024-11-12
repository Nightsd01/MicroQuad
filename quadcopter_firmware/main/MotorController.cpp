#include "MotorController.h"

#include <Logger.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000  // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000  // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif

MotorController::MotorController(gpio_num_t pin)
{
  LOG_INFO("Initializing motor on GPIO %d", pin);
  _escChan = NULL;
  rmt_tx_channel_config_t tx_chan_config = {
      .gpio_num = pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,  // select a clock that can provide needed resolution
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,  // set the number of transactions that can be pending in the background
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_escChan));

  LOG_INFO("Installing DShot ESC Encoder for GPIO %d", pin);
  _encoder = NULL;
  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = 300000,  // DSHOT300 protocol
      .post_delay_us = 50,  // extra delay between each frame
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &_encoder));

  LOG_INFO("Enabling RMT TX channel for GPIO %d", pin);
  ESP_ERROR_CHECK(rmt_enable(_escChan));
  _txConfig = {
      .loop_count = -1,  // infinite loop
  };
  _throttle = {
      .throttle = 0,
      .telemetry_req = false,  // telemetry is not supported in this example
  };
  ESP_ERROR_CHECK(rmt_transmit(_escChan, _encoder, &_throttle, sizeof(_throttle), &_txConfig));
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

void MotorController::_writeRawValue(uint16_t value)
{
  _throttle.throttle = value;
  ESP_ERROR_CHECK(rmt_transmit(_escChan, _encoder, &_throttle, sizeof(_throttle), &_txConfig));
  ESP_ERROR_CHECK(rmt_disable(_escChan));
  ESP_ERROR_CHECK(rmt_enable(_escChan));
}