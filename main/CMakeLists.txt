idf_component_register(
    SRCS "main.cpp" "IMU.cpp" "led_controller.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino
    REQUIRES MicroQuad
    REQUIRES bt
    REQUIRES Fusion
    REQUIRES icm42688P
    REQUIRES ESP32Servo
    REQUIRES qmc5883l
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DCONFIG_MPU_SPI")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC CONFIG_MPU_CHIP_MODEL="MPU6000")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DENABLE_EMERGENCY_MODE")
