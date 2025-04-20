cmake_minimum_required(VERSION 3.5)

option(BUILD_HOST_LIBRARY "Build a host static library for MATLAB SIM" OFF)

if (BUILD_HOST_LIBRARY)
  ##### HOST STATIC LIBRARY BUILD (for MATLAB) #####

  # Define a simpler project for the host build
  project(QuadcopterFirmwareHostLib LANGUAGES C CXX)
  message(STATUS "Configuring HOST build for MATLAB simulation library.")

  # Define the static library target
  # This will be the 'libfirmware_host.a' (or .lib) you link in MEX
  add_library(firmware_host STATIC)

  # --- Define Sources and Includes for the HOST library ---
  # This is the trickiest part: You need to specify ALL sources (.c, .cpp)
  # and include directories needed by the code paths used when MATLAB_SIM is defined.
  # This means code in 'main', relevant 'components', etc., but EXCLUDING
  # ESP32-specific drivers or code guarded by '#ifndef MATLAB_SIM'.

  # Example: List source files explicitly or use GLOB (GLOB is simpler but less precise)
  file(GLOB_RECURSE HOST_LIB_SOURCES
    "main/*.c"
    "main/*.cpp"
    "components/MicroQuad/*.c"
    "components/MicroQuad/*.cpp"
  )
  target_sources(firmware_host PRIVATE ${HOST_LIB_SOURCES})

  # Example: Specify include directories
  target_include_directories(firmware_host PUBLIC
    "."                          # Project root
    "main"                       # Main include directory
    "components/Matrix/include" # Example: Component include dir
    "components/MicroQuad/include"
  )

  # --- Define MATLAB_SIM ---
  # This ensures the '#ifndef MATLAB_SIM' guards work correctly for the host build
  target_compile_definitions(firmware_host PUBLIC MATLAB_SIM)

  # --- <<< REQUIRE C++20 >>> ---
  set_target_properties(firmware_host PROPERTIES
      CXX_STANDARD 20              # Specify C++20 standard
      CXX_STANDARD_REQUIRED YES    # Require this standard exactly
  )
  message(STATUS "C++ standard for firmware_host set to C++20.")

  # --- Add any other necessary host compile flags ---
  # target_compile_options(firmware_host PRIVATE -Wall -Wextra ...)

  message(STATUS "Host library 'firmware_host' defined.")
else () 
  ##### ESP32 FIRMWARE BUILD (Original Logic) #####
  message(STATUS "Configuring ESP32 firmware build.")

  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
  set(EXTRA_COMPONENT_DIRS other_components/esp32-arduino-lib-builder/components/arduino_tinyusb)
  project(AccelerometerTest)
  set(CONFIG_MPU_SPI 1)
  set(ONFIG_MPU6000 1)
  set(SPIBUS_COMPONENT_TRUE 1)
  set(CONFIG_MPU_ENABLE_DMP 1)

  message(STATUS "ESP32 project '${PROJECT_NAME}' configured.")
endif()
