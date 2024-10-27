#pragma once
#include <Arduino.h>

#include "freertos/FreeRTOS.h"

class LEDController {
 public:
  LEDController(int dataPin);
  void showRGB(uint8_t red, uint8_t green, uint8_t blue);

 private:
  int _dataPin;
  int _highStartPulseCycles;
  int _highEndPulseCycles;
  int _lowStartPulseCycles;
  int _lowEndPulseCycles;
};