#include "driver/ledc.h"
#include "esp_err.h"

class ElectronicSpeedController {
  public: 
    ElectronicSpeedController(uint8_t pin, uint8_t channel, esp_err_t *error);
    esp_err_t setMicrosecondsPulses(uint32_t value);

  private:
    ledc_channel_config_t _ledcChannel;
    bool _initializedSuccessfully;
};