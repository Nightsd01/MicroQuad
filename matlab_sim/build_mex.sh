#!/bin/bash

# STEP 1: Build the shared quadcopter firmware library
cd ../quadcopter_firmware
cmake -S . -B build_host -DBUILD_HOST_LIBRARY=ON
cmake --build build_host
if [ $? -ne 0 ]; then
  echo "Failed to build the quadcopter firmware library!"
  exit 1
fi
cd ../matlab_sim


# STEP 2: Build the MEX interface
LIB_DIR="../quadcopter_firmware/build_host/"
MICROQUAD_INCLUDE_DIR="components/MicroQuad/include"
MATRIX_INCLUDE_DIR="components/Matrix/include"

echo "Using Library Dir: ${LIB_DIR}"

# Ensure existence of necessary include dirs
if [ ! -d "${MICROQUAD_INCLUDE_DIR}" ]; then
    echo "ERROR: Specific include directory not found: ${MICROQUAD_INCLUDE_DIR}"
    echo "Check the path and ensure symlinks are correct."
    exit 1
fi

if [ ! -d "${MATRIX_INCLUDE_DIR}" ]; then
    echo "ERROR: Specific include directory not found: ${MATRIX_INCLUDE_DIR}"
    echo "Check the path and ensure symlinks are correct."
    exit 1
fi

mex mex_gateway.cpp Shared.cpp \
    -DMATLAB_SIM \
    -I./main \
    -I./main/Filters \
    -I./components \
    -I${MICROQUAD_INCLUDE_DIR} \
    -I${MATRIX_INCLUDE_DIR} \
    -L"${LIB_DIR}" \
    -lfirmware_host \
    CXXFLAGS='$CXXFLAGS -std=c++20'

# --- Optional: Add error checking ---
MEX_EXIT_CODE=$? # Capture the exit code of the mex command
if [ ${MEX_EXIT_CODE} -ne 0 ]; then
  echo "MEX compilation failed with exit code ${MEX_EXIT_CODE}!"
  exit 1 # Exit the script with a non-zero code to indicate failure
else
  echo "MEX compilation successful."
fi

# Exit with success code
exit 0