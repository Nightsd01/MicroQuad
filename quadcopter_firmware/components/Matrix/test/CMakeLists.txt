# Matrix/test/CMakeLists.txt

cmake_minimum_required(VERSION 3.14)
project(MatrixNativeTests LANGUAGES CXX) # Project name for the test build

set(CMAKE_CXX_STANDARD 17) # Or your desired C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Testing Setup (GTest via FetchContent) ---
# This setup is local to this test project build
enable_testing()
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip # Or specific commit/tag
)
FetchContent_MakeAvailable(googletest)

# --- Define the Test Executable ---
include(GoogleTest) # Provides gtest_discover_tests

add_executable(matrix_tests
    src/MatrixTests.cpp
    src/MatrixOperatorTests.cpp
)

# --- Link Dependencies ---

# Add the include directory from the parent ('Matrix/include')
# The path '../include' is relative to this CMakeLists.txt file (in Matrix/test/)
target_include_directories(matrix_tests PRIVATE
    ../include
    include
)

# Link the test executable against GTest
target_link_libraries(matrix_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
)

# --- Discover Tests ---
gtest_discover_tests(matrix_tests)

message(STATUS "Native test project configured. Headers sourced from: ${CMAKE_CURRENT_SOURCE_DIR}/../include")
message(STATUS "Run tests from build directory using 'ctest' or './matrix_tests'")