# Filter Tests with GTest
# Uses CMake for building with Google Test

BUILD_DIR = build

.PHONY: all build test clean configure test-median

all: build

configure:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

build: configure
	cd $(BUILD_DIR) && make

test: build
	cd $(BUILD_DIR) && ctest --output-on-failure

# Run specific test suites
test-median: build
	cd $(BUILD_DIR) && ./median_filter_tests

# Alternative: run the test executables directly
test-direct: build
	cd $(BUILD_DIR) && ./median_filter_tests

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Available targets:"
	@echo "  all        - Build all tests (default)"
	@echo "  configure  - Configure CMake build"
	@echo "  build      - Build the test executables"
	@echo "  test       - Run all tests using ctest"
	@echo "  test-median- Run MedianFilter tests only"
	@echo "  test-direct- Run test executables directly"
	@echo "  clean      - Remove build directory"
	@echo "  help       - Show this help" 