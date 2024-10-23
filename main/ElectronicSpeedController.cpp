#include "ElectronicSpeedController.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

ElectronicSpeedController::ElectronicSpeedController(uint8_t pin, uint8_t channel, esp_err_t *error)
{
  // Configure the LEDC PWM timer (shared among ESCs)
  static bool ledc_timer_initialized = false;
  if (!ledc_timer_initialized) {
      ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK,
      };
      esp_err_t err = ledc_timer_config(&ledc_timer);
      *error = err;
      if (err != ESP_OK) {
        return;
      }
      ledc_timer_initialized = true;
  }

  // Configure the LEDC PWM channel for this ESC
  _ledcChannel.speed_mode     = LEDC_LOW_SPEED_MODE;
  _ledcChannel.channel        = (ledc_channel_t)channel;
  _ledcChannel.timer_sel      = LEDC_TIMER_0;
  _ledcChannel.intr_type      = LEDC_INTR_DISABLE;
  _ledcChannel.gpio_num       = pin;
  _ledcChannel.duty           = 0;
  _ledcChannel.hpoint         = 0;

  *error = ledc_channel_config(&_ledcChannel);
  _initializedSuccessfully = (*error == ESP_OK);
}

esp_err_t ElectronicSpeedController::setMicrosecondsPulses(uint32_t value)
{
  if (!_initializedSuccessfully) {
    return ESP_ERR_INVALID_STATE;
  }

  // Pulse width limits (typically 1000 to 2000 µs for ESCs)
  const uint32_t minPulseWidthMicros = 1000;   // Minimum pulse width
  const uint32_t maxPulseWidthMicros = 2000;   // Maximum pulse width

  // Clamp the pulse width to the valid range
  uint32_t pulseWidthMicros = MIN(MAX(value, minPulseWidthMicros), maxPulseWidthMicros);

  // Calculate the duty cycle value
  uint32_t duty = (pulseWidthMicros * ((1 << LEDC_TIMER_12_BIT) - 1)) / (1000000 / 50); // 50 Hz frequency

  // Set the duty cycle
  esp_err_t err = ledc_set_duty(_ledcChannel.speed_mode, _ledcChannel.channel, duty);
  if (err != ESP_OK) {
      return err;
  }

  // Update the duty cycle to the hardware
  err = ledc_update_duty(_ledcChannel.speed_mode, _ledcChannel.channel);
  if (err != ESP_OK) {
      return err;
  }

  return ESP_OK;
}