#include "LEDController.h"

#define RMT_CLK_DIV 1              // RMT clock divider
#define RMT_TICK_DURATION_NS 12.5  // Duration of one RMT clock tick in nanoseconds

#define T0H_NS 400   // Width of a 0 bit's high level
#define T0L_NS 850   // Width of a 0 bit's low level
#define T1H_NS 800   // Width of a 1 bit's high level
#define T1L_NS 450   // Width of a 1 bit's low level
#define RESET_US 50  // Reset code duration in microseconds

#define T0H_TICKS (uint16_t)(T0H_NS / RMT_TICK_DURATION_NS)
#define T0L_TICKS (uint16_t)(T0L_NS / RMT_TICK_DURATION_NS)
#define T1H_TICKS (uint16_t)(T1H_NS / RMT_TICK_DURATION_NS)
#define T1L_TICKS (uint16_t)(T1L_NS / RMT_TICK_DURATION_NS)
#define RESET_TICKS (uint32_t)(RESET_US * 1000 / RMT_TICK_DURATION_NS)

LEDController::LEDController(gpio_num_t dataPin) : _dataPin(dataPin) {}

void LEDController::_connectRMT(void)
{
  // Configure the RMT channel for TX
  rmt_tx_channel_config_t tx_config = {
      .gpio_num = _dataPin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 80000000, // RMT runs at 80MHz
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,
      .flags =
          {
                  .invert_out = false,
                  .with_dma = false,
                  .io_loop_back = false,
                  .io_od_mode = false,
                  },
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &_rmt_channel));
  ESP_ERROR_CHECK(rmt_enable(_rmt_channel));  // Enable the channel

  // Create a copy encoder (raw encoder)
  rmt_copy_encoder_config_t encoder_config = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &_encoder));

  _connectedRMT = true;
}

void LEDController::disconnectRMT(void)
{
  // Disable the RMT channel for the LED
  ESP_ERROR_CHECK(rmt_disable(_rmt_channel));

  // Delete the LED encoder
  ESP_ERROR_CHECK(rmt_del_encoder(_encoder));

  // Delete the RMT channel for the LED
  ESP_ERROR_CHECK(rmt_del_channel(_rmt_channel));

  _connectedRMT = false;
}

void LEDController::showRGB(uint8_t red, uint8_t green, uint8_t blue)
{
  _connectRMT();
  rmt_symbol_word_t items[25];  // 24 items for bits, 1 for reset
  uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;

  // Prepare RMT items for each bit
  for (int i = 0; i < 24; i++) {
    uint8_t bit = (grb >> (23 - i)) & 0x01;
    if (bit) {
      // '1' bit
      items[i].level0 = 1;
      items[i].duration0 = T1H_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = T1L_TICKS;
    } else {
      // '0' bit
      items[i].level0 = 1;
      items[i].duration0 = T0H_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = T0L_TICKS;
    }
  }

  // Add reset code at the end
  items[24].level0 = 0;
  items[24].duration0 = RESET_TICKS;
  items[24].level1 = 0;
  items[24].duration1 = 0;

  // Transmit configuration
  rmt_transmit_config_t tx_config = {
      .loop_count = 0,  // Send once
  };

  // Transmit the items
  ESP_ERROR_CHECK(rmt_transmit(_rmt_channel, _encoder, items, sizeof(items), &tx_config));

  // Wait for transmission to complete
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(_rmt_channel, 30));
}