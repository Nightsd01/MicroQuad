idf_component_register(
    SRCS "BLEController.cpp" "main.cpp" "IMU.cpp" "led_controller.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino
    REQUIRES MicroQuad
    REQUIRES bt
    REQUIRES Fusion
    REQUIRES icm42688P
    REQUIRES qmc5883l
    REQUIRES ESP32Servo
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DCONFIG_MPU_SPI")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC CONFIG_MPU_CHIP_MODEL="MPU6000")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DENABLE_EMERGENCY_MODE")
