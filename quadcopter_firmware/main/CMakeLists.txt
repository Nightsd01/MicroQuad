file(GLOB_RECURSE app_sources *.cpp)

# Get all subdirectories under the current directory
file(GLOB APP_ALL_SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)

# Exclude certain directories
set(EXCLUDED_DIRS ".git" "build")

# Function to check if a directory is in the excluded list
function(is_excluded dir result)
    set(${result} "FALSE" PARENT_SCOPE)
    foreach(excluded_dir ${EXCLUDED_DIRS})
        if("${dir}" STREQUAL "${excluded_dir}")
            set(${result} "TRUE" PARENT_SCOPE)
            return()
        endif()
    endforeach()
endfunction()

# Filter directories
set(INCLUDE_DIRS ".")
foreach(dir ${APP_ALL_SUBDIRS})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
        is_excluded("${dir}" excluded)
        if(NOT excluded)
            list(APPEND INCLUDE_DIRS "${dir}")
        endif()
    endif()
endforeach()

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES MicroQuad
    REQUIRES bt
    REQUIRES icm42688P
    REQUIRES qmc5883l
    REQUIRES ESP32Servo
    REQUIRES DShot
    REQUIRES nvs_flash
    REQUIRES Matrix
    REQUIRES Adafruit_BME280
    REQUIRES esp-dsp
    REQUIRES VL53L1X
    REQUIRES TinyGPSPlus
    REQUIRES esp-nimble-cpp
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DENABLE_EMERGENCY_MODE")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DUSE_ACCELERATION")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DDEBUG_EKF")
