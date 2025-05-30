#include <gtest/gtest.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "MedianFilter.h"

// Helper function to calculate expected median
template<typename T>
T calculateExpectedMedian(const std::vector<T>& values) {
    if (values.empty()) {
        throw std::runtime_error("Empty vector");
    }
    
    std::vector<T> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    
    size_t n = sorted.size();
    if (n % 2 == 1) {
        return sorted[n / 2];
    } else {
        // For even number of elements, return average of middle two
        double avg = (static_cast<double>(sorted[n/2 - 1]) + static_cast<double>(sorted[n/2])) / 2.0;
        return static_cast<T>(avg);
    }
}

// ========================================================================
// == MEDIAN FILTER CONSTRUCTOR TESTS ==
// ========================================================================

TEST(MedianFilterConstructorTests, ValidWindowSize) {
    MedianFilter<float> filter(5);
    // If we get here without exception, the test passes
    SUCCEED();
}

TEST(MedianFilterConstructorTests, InvalidWindowSizeZero) {
    EXPECT_THROW(MedianFilter<float> filter(0), std::exception);
}

TEST(MedianFilterConstructorTests, WindowSizeOne) {
    MedianFilter<float> filter(1);
    // Should work fine
    SUCCEED();
}

// ========================================================================
// == MEDIAN FILTER BASIC FUNCTIONALITY TESTS ==
// ========================================================================

TEST(MedianFilterBasicTests, SingleValue) {
    MedianFilter<float> filter(3);
    filter.addValue(42.5f);
    
    float median = filter.getMedian();
    EXPECT_FLOAT_EQ(median, 42.5f);
}

TEST(MedianFilterBasicTests, TwoValues) {
    MedianFilter<float> filter(5);
    filter.addValue(10.0f);
    filter.addValue(20.0f);
    
    float median = filter.getMedian();
    float expected = (10.0f + 20.0f) / 2.0f;
    EXPECT_FLOAT_EQ(median, expected);
}

TEST(MedianFilterBasicTests, ThreeValues) {
    MedianFilter<float> filter(5);
    filter.addValue(30.0f);
    filter.addValue(10.0f);
    filter.addValue(20.0f);
    
    float median = filter.getMedian();
    EXPECT_FLOAT_EQ(median, 20.0f);
}

TEST(MedianFilterBasicTests, EmptyFilter) {
    MedianFilter<float> filter(5);
    
    EXPECT_THROW(filter.getMedian(), std::runtime_error);
}

// ========================================================================
// == MEDIAN FILTER SLIDING WINDOW TESTS ==
// ========================================================================

TEST(MedianFilterSlidingWindowTests, BasicSlidingWindow) {
    MedianFilter<float> filter(3);  // Window size 3
    
    // Add values: [1, 2, 3] -> median should be 2
    filter.addValue(1.0f);
    filter.addValue(2.0f);
    filter.addValue(3.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 2.0f);
    
    // Add 4: [2, 3, 4] -> median should be 3
    filter.addValue(4.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);
    
    // Add 1: [3, 4, 1] -> median should be 3
    filter.addValue(1.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);
}

TEST(MedianFilterSlidingWindowTests, WindowSizeOne) {
    MedianFilter<float> filter(1);
    
    filter.addValue(42.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 42.0f);
    
    filter.addValue(100.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 100.0f);
    
    filter.addValue(-50.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), -50.0f);
}

TEST(MedianFilterSlidingWindowTests, SlidingWindowSequence) {
    MedianFilter<float> filter(5);
    
    std::vector<float> values = {1.0f, 3.0f, 2.0f, 5.0f, 4.0f, 7.0f, 6.0f, 9.0f, 8.0f};
    std::vector<float> window;
    
    for (size_t i = 0; i < values.size(); ++i) {
        filter.addValue(values[i]);
        window.push_back(values[i]);
        
        // Keep only last 5 elements
        if (window.size() > 5) {
            window.erase(window.begin());
        }
        
        float expected = calculateExpectedMedian(window);
        float actual = filter.getMedian();
        
        EXPECT_NEAR(actual, expected, 0.001f) << "Failed at step " << i;
    }
}

// ========================================================================
// == MEDIAN FILTER DATA TYPE TESTS ==
// ========================================================================

TEST(MedianFilterDataTypeTests, IntegerType) {
    MedianFilter<int16_t> filter(3);
    
    filter.addValue(100);
    filter.addValue(50);
    filter.addValue(75);
    
    int16_t median = filter.getMedian();
    EXPECT_EQ(median, 75);
}

TEST(MedianFilterDataTypeTests, FloatingPointPrecision) {
    MedianFilter<float> filter(4);
    
    filter.addValue(-10.0f);
    filter.addValue(-5.0f);
    filter.addValue(5.0f);
    filter.addValue(10.0f);
    
    float median = filter.getMedian();
    float expected = (-5.0f + 5.0f) / 2.0f;  // 0.0f
    EXPECT_FLOAT_EQ(median, 0.0f);
}

// ========================================================================
// == MEDIAN FILTER EDGE CASE TESTS ==
// ========================================================================

TEST(MedianFilterEdgeCaseTests, DuplicateValues) {
    MedianFilter<float> filter(5);
    
    filter.addValue(5.0f);
    filter.addValue(5.0f);
    filter.addValue(5.0f);
    filter.addValue(10.0f);
    
    float median = filter.getMedian();
    float expected = (5.0f + 5.0f) / 2.0f;  // Middle two values
    EXPECT_FLOAT_EQ(median, expected);
}

TEST(MedianFilterEdgeCaseTests, NegativeValues) {
    MedianFilter<float> filter(4);
    
    filter.addValue(-10.0f);
    filter.addValue(-5.0f);
    filter.addValue(5.0f);
    filter.addValue(10.0f);
    
    float median = filter.getMedian();
    float expected = (-5.0f + 5.0f) / 2.0f;  // 0.0f
    EXPECT_FLOAT_EQ(median, 0.0f);
}

TEST(MedianFilterEdgeCaseTests, AllSameValues) {
    MedianFilter<float> filter(5);
    
    for (int i = 0; i < 10; ++i) {
        filter.addValue(42.0f);
        EXPECT_FLOAT_EQ(filter.getMedian(), 42.0f);
    }
}

// ========================================================================
// == MEDIAN FILTER COMPREHENSIVE TESTS ==
// ========================================================================

TEST(MedianFilterComprehensiveTests, RandomSequence) {
    MedianFilter<float> filter(7);
    
    std::vector<float> values = {3.1f, 1.4f, 1.5f, 9.2f, 6.5f, 3.5f, 8.9f};
    std::vector<float> window;
    
    for (size_t i = 0; i < values.size(); ++i) {
        filter.addValue(values[i]);
        window.push_back(values[i]);
        
        float expected = calculateExpectedMedian(window);
        float actual = filter.getMedian();
        
        EXPECT_NEAR(actual, expected, 0.001f) << "Failed at step " << i;
    }
}

TEST(MedianFilterComprehensiveTests, StressTest) {
    MedianFilter<float> filter(50);
    
    // Add 100 values (less than the stress test to avoid too much output)
    for (int i = 0; i < 100; ++i) {
        float value = static_cast<float>(i % 100);  // Cycle 0-99
        filter.addValue(value);
        
        // Just verify we can get a median without crashing
        float median = filter.getMedian();
        EXPECT_GE(median, 0.0f);
        EXPECT_LE(median, 99.0f);
    }
}

TEST(MedianFilterComprehensiveTests, LargeWindow) {
    MedianFilter<float> filter(100);
    
    // Add values in a pattern
    for (int i = 0; i < 150; ++i) {
        filter.addValue(static_cast<float>(i % 10));  // Cycle 0-9
        
        float median = filter.getMedian();
        EXPECT_GE(median, 0.0f);
        EXPECT_LE(median, 9.0f);
    }
}

// ========================================================================
// == MEDIAN FILTER BUG REGRESSION TESTS ==
// ========================================================================

TEST(MedianFilterBugRegressionTests, SlidingWindowBugScenario) {
    // This test specifically covers the bug scenario that was failing
    MedianFilter<float> filter(3);
    
    // Step 1: Add [1, 2, 3] -> window = [1, 2, 3], median = 2
    filter.addValue(1.0f);
    filter.addValue(2.0f);
    filter.addValue(3.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 2.0f);
    
    // Step 2: Add 4 -> window = [2, 3, 4], median = 3 (was failing with 2.5)
    filter.addValue(4.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);
    
    // Step 3: Add 1 -> window = [3, 4, 1], median = 3 (was failing with 2)
    filter.addValue(1.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);
    
    // Step 4: Add 5 -> window = [4, 1, 5], median = 4
    filter.addValue(5.0f);
    EXPECT_FLOAT_EQ(filter.getMedian(), 4.0f);
}

TEST(MedianFilterBugRegressionTests, DelayedRemovalEdgeCase) {
    // Test case where delayed elements accumulate in heaps
    MedianFilter<float> filter(2);
    
    // Add sequence that causes delayed elements to build up
    filter.addValue(10.0f);  // window = [10], median = 10
    EXPECT_FLOAT_EQ(filter.getMedian(), 10.0f);
    
    filter.addValue(5.0f);   // window = [10, 5], median = 7.5
    EXPECT_FLOAT_EQ(filter.getMedian(), 7.5f);
    
    filter.addValue(15.0f);  // window = [5, 15], median = 10 (10 should be removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 10.0f);
    
    filter.addValue(1.0f);   // window = [15, 1], median = 8 (5 should be removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 8.0f);
    
    filter.addValue(20.0f);  // window = [1, 20], median = 10.5 (15 should be removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 10.5f);
}

TEST(MedianFilterBugRegressionTests, HeapBalancingAfterRemoval) {
    // Test that heap balancing works correctly after delayed removals
    MedianFilter<float> filter(4);
    
    // Build up a specific pattern
    filter.addValue(1.0f);   // [1]
    filter.addValue(2.0f);   // [1, 2]
    filter.addValue(3.0f);   // [1, 2, 3]
    filter.addValue(4.0f);   // [1, 2, 3, 4]
    EXPECT_FLOAT_EQ(filter.getMedian(), 2.5f);  // (2+3)/2
    
    // Now slide the window
    filter.addValue(5.0f);   // [2, 3, 4, 5] (1 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.5f);  // (3+4)/2
    
    filter.addValue(6.0f);   // [3, 4, 5, 6] (2 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 4.5f);  // (4+5)/2
    
    filter.addValue(1.0f);   // [4, 5, 6, 1] (3 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 4.5f);  // (4+5)/2
}

TEST(MedianFilterBugRegressionTests, AlternatingValues) {
    // Test alternating high/low values that stress the heap balancing
    MedianFilter<float> filter(3);
    
    filter.addValue(100.0f); // [100]
    EXPECT_FLOAT_EQ(filter.getMedian(), 100.0f);
    
    filter.addValue(1.0f);   // [100, 1]
    EXPECT_FLOAT_EQ(filter.getMedian(), 50.5f);
    
    filter.addValue(200.0f); // [100, 1, 200]
    EXPECT_FLOAT_EQ(filter.getMedian(), 100.0f);
    
    filter.addValue(2.0f);   // [1, 200, 2] (100 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 2.0f);
    
    filter.addValue(300.0f); // [200, 2, 300] (1 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 200.0f);
    
    filter.addValue(3.0f);   // [2, 300, 3] (200 removed)
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);
}

TEST(MedianFilterBugRegressionTests, ManyDelayedRemovals) {
    // Test scenario with many delayed removals to stress the cleanup mechanism
    MedianFilter<float> filter(5);
    
    // Fill the window
    for (int i = 1; i <= 5; ++i) {
        filter.addValue(static_cast<float>(i));
    }
    EXPECT_FLOAT_EQ(filter.getMedian(), 3.0f);  // [1,2,3,4,5] -> median = 3
    
    // Add many more values to cause lots of delayed removals
    for (int i = 6; i <= 15; ++i) {
        filter.addValue(static_cast<float>(i));
        // Window should always contain the last 5 values
        // For i=6: [2,3,4,5,6] -> median = 4
        // For i=7: [3,4,5,6,7] -> median = 5
        // etc.
        float expectedMedian = static_cast<float>(i - 2);
        EXPECT_FLOAT_EQ(filter.getMedian(), expectedMedian) 
            << "Failed at iteration i=" << i;
    }
} 