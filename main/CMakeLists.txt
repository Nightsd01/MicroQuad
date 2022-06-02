idf_component_register(
    SRCS "main.cpp"
    SRCS "diagnostics.cpp"
    SRCS "debug_util.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino
    REQUIRES MicroQuad
    REQUIRES MPU6050
    REQUIRES i2cdev
    REQUIRES ESP32Servo
)
