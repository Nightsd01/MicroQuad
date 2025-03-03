# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS other_components/esp32-arduino-lib-builder/components/arduino_tinyusb)
project(AccelerometerTest)
set(CONFIG_MPU_SPI 1)
set(ONFIG_MPU6000 1)
set(SPIBUS_COMPONENT_TRUE 1)
set(CONFIG_MPU_ENABLE_DMP 1)
