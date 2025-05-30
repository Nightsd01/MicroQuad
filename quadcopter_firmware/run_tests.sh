#!/bin/bash

# MedianFilter Test Runner Script
# Runs comprehensive unit tests for the MedianFilter component

set -e  # Exit on any error

echo "🧪 MedianFilter Test Runner"
echo "=========================="

# Check if we're in the right directory
if [ ! -d "test" ]; then
    echo "❌ Error: test directory not found. Please run this script from the quadcopter_firmware directory."
    exit 1
fi

# Navigate to test directory
cd test

echo "📁 Working directory: $(pwd)"

# Clean previous build
echo "🧹 Cleaning previous build..."
make clean

# Build tests
echo "🔨 Building tests..."
make build

# Run tests
echo "🚀 Running MedianFilter tests..."
make test

echo ""
echo "✅ All tests completed successfully!"
echo ""
echo "📊 Test Summary:"
echo "   - MedianFilter: 23 comprehensive test cases"
echo "   - Coverage: Constructor, basic operations, sliding window, edge cases, and regression tests"
echo ""
echo "🎯 To run specific tests:"
echo "   make test-median    # Run MedianFilter tests only"
echo "   make test-direct    # Run test executable directly" 