idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino
    REQUIRES MicroQuad
    REQUIRES MPU6050
    REQUIRES i2cdev
    REQUIRES ESP32Servo
)
