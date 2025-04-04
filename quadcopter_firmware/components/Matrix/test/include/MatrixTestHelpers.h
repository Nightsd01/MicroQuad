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