# MedianFilter/test/CMakeLists.txt

cmake_minimum_required(VERSION 3.14)
project(MedianFilterNativeTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compiler-specific flags to ensure standard library is found
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

# --- Testing Setup (GTest via FetchContent) ---
enable_testing()
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(googletest)

# --- Define the Test Executable ---
include(GoogleTest) # Provides gtest_discover_tests

add_executable(median_filter_tests
    test_median_filter.cpp
    ../main/Filters/MedianFilter.cpp
)

# --- Link Dependencies ---

# Add the include directories for MedianFilter headers
target_include_directories(median_filter_tests PRIVATE
    ../main
    ../main/Filters
)

# Define MATLAB_SIM to avoid ESP-specific code
target_compile_definitions(median_filter_tests PRIVATE MATLAB_SIM)

# Link the test executables against GTest
target_link_libraries(median_filter_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
)

# --- Discover Tests ---
gtest_discover_tests(median_filter_tests)

message(STATUS "MedianFilter test project configured. Headers sourced from: ${CMAKE_CURRENT_SOURCE_DIR}/../main")
message(STATUS "Run tests from build directory using 'ctest' or './median_filter_tests'") 