#pragma once
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

class LEDController
{
 public:
  LEDController(int dataPin);
  void showRGB(uint8_t red, uint8_t green, uint8_t blue);

 private:
  int _dataPin;
  rmt_channel_handle_t _rmt_channel;  // Handle for RMT channel
  rmt_encoder_handle_t _encoder;      // Handle for RMT encoder
};