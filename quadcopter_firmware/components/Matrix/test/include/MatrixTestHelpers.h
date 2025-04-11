#pragma once

#include <gtest/gtest.h>  // Include dependencies needed by the function body

#include <type_traits>

#include "Matrix.h"  // Adjust path if needed from this header's location

template <size_t R, size_t C>
void _assertRoughlyEqual(const Matrix<float, R, C>& A, const Matrix<float, R, C>& B)  // Definition is now HERE
{
  static_assert(std::is_floating_point_v<float>, "Helper should use float");
  for (size_t r = 0; r < R; ++r) {
    for (size_t c = 0; c < C; ++c) {
      EXPECT_NEAR(A(r, c), B(r, c), 1e-5f)
          << "Element (" << r << ", " << c << ") differs. Actual: " << A(r, c) << ", Expected: " << B(r, c);
    }
  }
}

template <size_t R, size_t C>
void _assertRoughlyEqual(const Matrix<double, R, C>& A, const Matrix<float, R, C>& B)  // Definition is now HERE
{
  // ... function body ...
  for (size_t r = 0; r < R; ++r) {
    for (size_t c = 0; c < C; ++c) {
      EXPECT_NEAR(A(r, c), B(r, c), 1e-5f)  // Check tolerance/types here too
          << "Element (" << r << ", " << c << ") differs. Actual(double): " << A(r, c)
          << ", Expected(float): " << B(r, c);
    }
  }
}

// Helper function specifically for float matrices that checks for NaN equality.
// Assumes _assertRoughlyEqual handles non-NaN floats.
template <typename T, size_t R, size_t C>
void assertFloatMatrixEqualWithNaN(const Matrix<T, R, C>& actual, const Matrix<T, R, C>& expected, T tolerance = 1e-6f)
{
  // Ensure this is only used for float/double types
  static_assert(std::is_floating_point<T>::value, "This helper is for floating-point types.");

  for (size_t r = 0; r < R; ++r) {
    for (size_t c = 0; c < C; ++c) {
      // Assuming element access via operator() - adapt if different (e.g., .data[r*C + c])
      T actual_val = actual(r, c);
      T expected_val = expected(r, c);

      // 1. Handle NaN cases first (NaN != NaN)
      if (std::isnan(expected_val)) {
        // If expected is NaN, actual must also be NaN
        EXPECT_TRUE(std::isnan(actual_val))
            << "Mismatch at (" << r << "," << c << "): Expected NaN, got " << actual_val;
      }
      // 2. Handle Infinite cases (Inf == Inf, -Inf == -Inf, Inf != -Inf)
      else if (std::isinf(expected_val)) {
        // If expected is Inf, actual must be exactly the same Inf (including sign)
        EXPECT_EQ(actual_val, expected_val)
            << "Mismatch at (" << r << "," << c << "): Infinite values differ. Expected: " << expected_val
            << ", Got: " << actual_val;
      }
      // 3. Handle Finite numbers using tolerance
      else {
        // For finite numbers, use EXPECT_NEAR for tolerance-based comparison
        EXPECT_NEAR(actual_val, expected_val, std::fabs(expected_val * tolerance))  // Relative tolerance
                                                                                    // Or use absolute tolerance:
                                                                                    // EXPECT_NEAR(actual_val,
                                                                                    // expected_val, tolerance)
            << "Mismatch at (" << r << "," << c << ")";
      }
    }
  }
}