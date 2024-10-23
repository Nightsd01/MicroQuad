#include "led_controller.h"

#include "driver/gpio.h"
#include "driver/dedic_gpio.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_struct.h"
#include "freertos/task.h"

LEDController::LEDController(int dataPin) 
{
    _dataPin = dataPin;
    esp_rom_gpio_pad_select_gpio((gpio_num_t)dataPin);
    gpio_set_direction((gpio_num_t)dataPin, GPIO_MODE_OUTPUT);
    GPIO.out1_w1tc.val = ((uint32_t)1 << dataPin);
    portDISABLE_INTERRUPTS();
    const uint32_t beginCycles = xthal_get_ccount();
    delayMicroseconds(10);
    const uint32_t endCycles = xthal_get_ccount();
    portENABLE_INTERRUPTS();
    const uint32_t cyclesPerMicrosecond = (endCycles - beginCycles) / 10;
    _highStartPulseCycles = (uint32_t)((0.8f * cyclesPerMicrosecond) / 10);
    _highEndPulseCycles = (uint32_t)((0.45f * cyclesPerMicrosecond) / 10);
    _lowStartPulseCycles = (uint32_t)((0.4f * cyclesPerMicrosecond) / 10);
    _lowEndPulseCycles = (uint32_t)((0.85f * cyclesPerMicrosecond) / 10);
}

static inline __attribute__((always_inline))
void IRAM_ATTR _writeColor(
    uint8_t red, 
    uint8_t green, 
    uint8_t blue, 
    uint32_t highStartPulseCycles, 
    uint32_t highEndPulseCycles, 
    uint32_t lowStartPulseCycles, 
    uint32_t lowEndPulseCycles, uint8_t pin
) {
    const uint8_t bytes[3] = {green, red, blue};
    portDISABLE_INTERRUPTS();
    for (int i = 0; i < 3; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            const bool isHigh = (bytes[i] >> bit) & 1;
            uint32_t cycles = isHigh ? highStartPulseCycles : lowStartPulseCycles;
            GPIO.out1_w1ts.val = ((uint32_t)1 << pin);
            while(cycles--) {
                __asm__ volatile ("nop");
            }
            cycles = isHigh ? highEndPulseCycles : lowEndPulseCycles;
            GPIO.out1_w1tc.val = ((uint32_t)1 << pin);
            while(cycles--) {
                __asm__ volatile ("nop");
            }
        }
    }
    GPIO.out1_w1tc.val = ((uint32_t)1 << pin);
    portENABLE_INTERRUPTS();
}

void LEDController::showRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint32_t pin = _dataPin;
    const uint32_t highStartPulseCycles = _highStartPulseCycles;
    const uint32_t highEndPulseCycles = _highEndPulseCycles;
    const uint32_t lowStartPulseCycles = _lowStartPulseCycles;
    const uint32_t lowEndPulseCycles = _lowEndPulseCycles;
    _writeColor(
        red, 
        green, 
        blue,
        highStartPulseCycles, 
        highEndPulseCycles, 
        lowStartPulseCycles, 
        lowEndPulseCycles, 
        pin
    );
}