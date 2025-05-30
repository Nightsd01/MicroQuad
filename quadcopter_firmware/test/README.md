# Filter Unit Tests

This directory contains unit tests for the MedianFilter and ExtendedKalmanFilter components using Google Test (gtest), following the same pattern as the Matrix component tests.

## Overview

This setup uses **Google Test with CMake** for comprehensive C++ unit testing. Tests run on your computer (Linux/macOS/Windows) for fast development cycles, with no ESP-IDF dependencies or ESP32 hardware required.

## Prerequisites

1. CMake 3.14 or higher
2. C++ compiler with C++20 support (g++, clang++, etc.)
3. Make utility
4. Internet connection (for downloading Google Test)

## Running Tests

### Option 1: Use the provided script (Recommended)
```bash
cd quadcopter_firmware
./run_tests.sh
```

### Option 2: Manual execution with Make
```bash
cd quadcopter_firmware/test
make test              # Run all tests
make test-median       # Run MedianFilter tests only
```

### Option 3: Manual execution with CMake
```bash
cd quadcopter_firmware/test
mkdir build && cd build
cmake ..
make
ctest --output-on-failure
```

### Option 4: Run test executables directly
```bash
cd quadcopter_firmware/test
make test-direct
# or after building:
# ./build/median_filter_tests
```

## Test Structure

```
test/
├── CMakeLists.txt                  # CMake configuration with gtest
├── test_median_filter.cpp          # MedianFilter unit tests
├── test_extended_kalman_filter.cpp # ExtendedKalmanFilter unit tests
├── Logger.h                        # Mock logger for EKF tests
├── Makefile                        # Convenience wrapper for CMake
├── README.md                       # This file
└── build/                          # Generated build directory
    ├── median_filter_tests         # MedianFilter test executable
```

The tests reference the original source files in `../main/Filters/` - **no duplication needed!**

## Current Test Coverage

### MedianFilter Tests (Organized by Test Suites)

**MedianFilterConstructorTests:**
- ✅ Valid window sizes
- ✅ Invalid window size (zero)
- ✅ Window size one

**MedianFilterBasicTests:**
- ✅ Single value handling
- ✅ Two values (even count median)
- ✅ Three values (odd count median)
- ✅ Empty filter exception handling

**MedianFilterSlidingWindowTests:**
- ✅ Basic sliding window functionality
- ✅ Window size one behavior
- ✅ Complex sliding window sequences

**MedianFilterDataTypeTests:**
- ✅ Integer data types
- ✅ Floating-point precision

**MedianFilterEdgeCaseTests:**
- ✅ Duplicate values
- ✅ Negative values
- ✅ All same values

**MedianFilterComprehensiveTests:**
- ✅ Random value sequences
- ✅ Stress testing
- ✅ Large window sizes

**MedianFilterBugRegressionTests:**
- ✅ Sliding window bug scenario (fixed delayed removal issue)
- ✅ Delayed removal edge cases
- ✅ Heap balancing after removal
- ✅ Alternating high/low values
- ✅ Many delayed removals stress test

**Total MedianFilter Tests: 23 comprehensive test cases organized in 7 test suites**

## Google Test Assertions

Common gtest assertions available:
- `EXPECT_EQ(expected, actual)` - Exact equality
- `EXPECT_FLOAT_EQ(expected, actual)` - Float equality with tolerance
- `EXPECT_NEAR(actual, expected, tolerance)` - Custom tolerance
- `EXPECT_TRUE(condition)` / `EXPECT_FALSE(condition)`
- `EXPECT_THROW(statement, exception_type)`
- `EXPECT_GE(val1, val2)` / `EXPECT_LE(val1, val2)` - Greater/less than or equal
- `SUCCEED()` - Mark test as passed
- `FAIL() << "message"` - Force test failure
