idf_component_register(
    SRCS "main.cpp" "IMU.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino
    REQUIRES MicroQuad
    REQUIRES ESP32Servo
    REQUIRES bt
    REQUIRES FastLED
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DFASTLED_ESP32_I2S")