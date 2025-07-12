#include <gtest/gtest.h>
#include <stdio.h>

#include "Matrix.h"
#include "MatrixTestHelpers.h"

TEST(MatrixScalarAdditionIntTests, PlusEqual)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  A += 1;
  Matrix<int, 2, 2> expected = {
      {2, 3},
      {4, 5}
  };
  EXPECT_EQ(A, expected);
}

TEST(MatrixScalarAdditionIntTests, ScalarPlusMatrix)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 2, 2> B = 1 + A;
  Matrix<int, 2, 2> expected = {
      {2, 3},
      {4, 5}
  };
  EXPECT_EQ(B, expected);
}

TEST(MatrixScalarAdditionIntTests, MatrixPlusScalar)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 2, 2> B = A + 1;
  Matrix<int, 2, 2> expected = {
      {2, 3},
      {4, 5}
  };
  EXPECT_EQ(B, expected);
}

TEST(MatrixScalarAdditionFloatTests, PlusEqual)
{
  Matrix<float, 2, 2> A = {
      {1.5f, 2.5f},
      {3.5f, 4.5f}
  };
  A += 1.1f;
  Matrix<float, 2, 2> expected = {
      {2.6f, 3.6f},
      {4.6f, 5.6f}
  };

  // Assuming Matrix::operator== handles floats correctly or for simple cases:
  _assertRoughlyEqual(A, expected);
}

TEST(MatrixScalarAdditionFloatTests, ScalarPlusMatrix)
{
  Matrix<float, 2, 2> A = {
      {1.5f, 2.5f},
      {3.5f, 4.5f}
  };
  Matrix<float, 2, 2> B = 1.1f + A;
  Matrix<float, 2, 2> expected = {
      {2.6f, 3.6f},
      {4.6f, 5.6f}
  };
  _assertRoughlyEqual(B, expected);
}

TEST(MatrixScalarAdditionFloatTests, MatrixPlusScalar)
{
  Matrix<float, 2, 2> A = {
      {1.5f, 2.5f},
      {3.5f, 4.5f}
  };
  Matrix<float, 2, 2> B = A + 1.1f;
  Matrix<float, 2, 2> expected = {
      {2.6f, 3.6f},
      {4.6f, 5.6f}
  };
  _assertRoughlyEqual(B, expected);
}

TEST(MatrixScalarSubtractionIntTests, MinusEqual)
{
  Matrix<int, 2, 2> A = {
      {5, 6},
      {7, 8}
  };
  A -= 2;
  Matrix<int, 2, 2> expected = {
      {3, 4},
      {5, 6}
  };
  EXPECT_EQ(A, expected);
}

TEST(MatrixScalarSubtractionIntTests, MatrixMinusScalar)
{
  Matrix<int, 2, 2> A = {
      {5, 6},
      {7, 8}
  };
  Matrix<int, 2, 2> B = A - 2;
  Matrix<int, 2, 2> expected = {
      {3, 4},
      {5, 6}
  };
  EXPECT_EQ(B, expected);
}

TEST(MatrixScalarSubtractionIntTests, ScalarMinusMatrix)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  // Calculate expected: scalar - element
  Matrix<int, 2, 2> C = 10 - A;
  Matrix<int, 2, 2> expected = {
      {10 - 1, 10 - 2},
      {10 - 3, 10 - 4}
  };  // {{9, 8}, {7, 6}}
  EXPECT_EQ(C, expected);
}

TEST(MatrixScalarSubtractionFloatTests, MinusEqual)
{
  Matrix<float, 2, 2> A = {
      {5.5f, 6.5f},
      {7.5f, 8.5f}
  };
  A -= 1.1f;
  Matrix<float, 2, 2> expected = {
      {4.4f, 5.4f},
      {6.4f, 7.4f}
  };
  _assertRoughlyEqual(A, expected);
}

TEST(MatrixScalarSubtractionFloatTests, MatrixMinusScalar)
{
  Matrix<float, 2, 2> A = {
      {5.5f, 6.5f},
      {7.5f, 8.5f}
  };
  Matrix<float, 2, 2> B = A - 1.1f;
  Matrix<float, 2, 2> expected = {
      {4.4f, 5.4f},
      {6.4f, 7.4f}
  };
  _assertRoughlyEqual(B, expected);
}

TEST(MatrixScalarSubtractionFloatTests, ScalarMinusMatrix)
{
  Matrix<float, 2, 2> A = {
      {1.5f, 2.5f},
      {3.5f, 4.5f}
  };
  // Calculate expected: scalar - element
  Matrix<float, 2, 2> C = 10.0f - A;
  Matrix<float, 2, 2> expected = {
      {10.0f - 1.5f, 10.0f - 2.5f}, // {8.5f, 7.5f}
      {10.0f - 3.5f, 10.0f - 4.5f}  // {6.5f, 5.5f}
  };
  _assertRoughlyEqual(C, expected);
}

TEST(MatrixScalarMultiplicationIntTests, ScalarTimesMatrix)
{
  Matrix<int, 2, 3> A = {
      {1, 2, 3},
      {4, 5, 6}
  };
  int scalar = 3;
  Matrix<int, 2, 3> result = scalar * A;
  Matrix<int, 2, 3> expected = {
      {3,  6,  9 },
      {12, 15, 18}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixScalarMultiplicationIntTests, MatrixTimesScalar)
{
  Matrix<int, 2, 2> A = {
      {1,  -2},
      {-3, 4 }
  };
  int scalar = -2;
  Matrix<int, 2, 2> result = A * scalar;
  Matrix<int, 2, 2> expected = {
      {-2, 4 },
      {6,  -8}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixScalarMultiplicationIntTests, MatrixTimesZeroScalar)
{
  Matrix<int, 3, 1> A = {{10}, {20}, {30}};
  int scalar = 0;
  Matrix<int, 3, 1> result = A * scalar;
  Matrix<int, 3, 1> expected = {{0}, {0}, {0}};
  EXPECT_EQ(result, expected);
}

TEST(MatrixScalarMultiplicationIntTests, ZeroScalarTimesMatrix)
{
  Matrix<int, 1, 3> A = {
      {10, 20, 30}
  };
  int scalar = 0;
  Matrix<int, 1, 3> result = scalar * A;
  Matrix<int, 1, 3> expected = {
      {0, 0, 0}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixScalarMultiplicationFloatTests, ScalarTimesMatrix)
{
  Matrix<float, 2, 2> A = {
      {1.5f, -2.0f},
      {0.0f, 4.0f }
  };
  float scalar = 2.5f;
  Matrix<float, 2, 2> result = scalar * A;
  Matrix<float, 2, 2> expected = {
      {3.75f, -5.0f},
      {0.0f,  10.0f}
  };
  // Using the provided helper function (tolerance is 1e-8f inside)
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixScalarMultiplicationFloatTests, MatrixTimesScalar)
{
  Matrix<float, 2, 3> A = {
      {1.0f, 2.0f, 3.0f},
      {4.0f, 5.0f, 6.0f}
  };
  float scalar = -0.5f;
  Matrix<float, 2, 3> result = A * scalar;
  Matrix<float, 2, 3> expected = {
      {-0.5f, -1.0f, -1.5f},
      {-2.0f, -2.5f, -3.0f}
  };
  // Using the provided helper function (tolerance is 1e-8f inside)
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixScalarMultiplicationFloatTests, MatrixTimesZeroScalar)
{
  Matrix<float, 2, 2> A = {
      {1.1f, 2.2f},
      {3.3f, 4.4f}
  };
  float scalar = 0.0f;
  Matrix<float, 2, 2> result = A * scalar;
  Matrix<float, 2, 2> expected = {
      {0.0f, 0.0f},
      {0.0f, 0.0f}
  };
  // Using the provided helper function (tolerance is 1e-8f inside)
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixMatrixMultiplicationIntTests, SquareMatrices)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 2, 2> B = {
      {5, 6},
      {7, 8}
  };
  Matrix<int, 2, 2> expected = {
      {19, 22},
      {43, 50}
  };
  Matrix<int, 2, 2> result = A * B;
  EXPECT_EQ(result, expected);
}

TEST(MatrixMatrixMultiplicationIntTests, RectangularMatrices)
{
  Matrix<int, 2, 3> A = {
      {1, 2, 3},
      {4, 5, 6}
  };  // 2x3
  Matrix<int, 3, 2> B = {
      {7,  8 },
      {9,  10},
      {11, 12}
  };  // 3x2
  Matrix<int, 2, 2> expected = {
      {58,  64 },
      {139, 154}
  };  // 2x2 result
  Matrix<int, 2, 2> result = A * B;
  EXPECT_EQ(result, expected);
}

TEST(MatrixMatrixMultiplicationIntTests, MultiplyByIdentity)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 2, 2> I = Matrix<int, 2, 2>::identity();
  Matrix<int, 2, 2> result_AI = A * I;
  Matrix<int, 2, 2> result_IA = I * A;
  EXPECT_EQ(result_AI, A) << "A * I should equal A";
  EXPECT_EQ(result_IA, A) << "I * A should equal A";
}

TEST(MatrixMatrixMultiplicationIntTests, RowVectorTimesMatrix)
{
  Matrix<int, 1, 3> A = {
      {1, 2, 3}
  };  // 1x3 (row vector)
  Matrix<int, 3, 2> B = {
      {1, 0},
      {0, 1},
      {2, 2}
  };  // 3x2
  Matrix<int, 1, 2> expected = {
      {7, 8}
  };  // 1x2 result
  Matrix<int, 1, 2> result = A * B;
  EXPECT_EQ(result, expected);
}

TEST(MatrixMatrixMultiplicationIntTests, MatrixTimesColumnVector)
{
  Matrix<int, 2, 3> A = {
      {1, 2, 3},
      {4, 5, 6}
  };                            // 2x3
  Matrix<int, 3, 1> B = {{10}, {0}, {-1}};   // 3x1 (column vector)
  Matrix<int, 2, 1> expected = {{7}, {34}};  // 2x1 result
  Matrix<int, 2, 1> result = A * B;
  EXPECT_EQ(result, expected);
}

TEST(MatrixMatrixMultiplicationFloatTests, SquareMatrices)
{
  Matrix<float, 2, 2> A = {
      {1.0f, -1.0f},
      {2.0f, 0.5f }
  };
  Matrix<float, 2, 2> B = {
      {0.5f,  2.0f},
      {-1.0f, 0.0f}
  };
  Matrix<float, 2, 2> expected = {
      {1.5f, 2.0f},
      {0.5f, 4.0f}
  };

  Matrix<float, 2, 2> result = A * B;
  // Using the provided helper function (tolerance is 1e-8f inside)
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixMatrixMultiplicationFloatTests, RectangularMatrices)
{
  Matrix<float, 2, 3> A = {
      {1.0f, 0.0f, -1.0f},
      {0.0f, 1.0f, 2.0f }
  };  // 2x3
  Matrix<float, 3, 2> B = {
      {1.0f, 2.0f},
      {3.0f, 4.0f},
      {5.0f, 6.0f}
  };  // 3x2
  Matrix<float, 2, 2> expected = {
      {-4.0f, -4.0f},
      {13.0f, 16.0f}
  };  // 2x2 result
  Matrix<float, 2, 2> result = A * B;
  // Using the provided helper function (tolerance is 1e-8f inside)
  _assertRoughlyEqual(result, expected);
}

#include <gtest/gtest.h>

#include <cmath>   // For std::numeric_limits<float>::infinity(), ::quiet_NaN(), std::isnan
#include <limits>  // For std::numeric_limits

#include "Matrix.h"  // Include your Matrix class header

// Assume _assertRoughlyEqual is defined elsewhere, e.g.:
// template <typename T, size_t R, size_t C>
// void _assertRoughlyEqual(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B, T tolerance = 1e-6f);
// NOTE: We will add explicit NaN checks as _assertRoughlyEqual might fail for NaN.

// --- Test Suite for Integer Negation ---
TEST(MatrixNegationIntTests, NegateMixedSigns)
{
  Matrix<int, 2, 2> A = {
      {1,  -2},
      {-3, 4 }
  };
  Matrix<int, 2, 2> result = -A;
  Matrix<int, 2, 2> expected = {
      {-1, 2 },
      {3,  -4}
  };
  EXPECT_EQ(result, expected);  // Exact comparison for integers
}

TEST(MatrixNegationIntTests, NegateZeros)
{
  Matrix<int, 2, 3> A = {
      {1, 0,  -2},
      {0, -0, 0 }  // Integer -0 is just 0
  };
  Matrix<int, 2, 3> result = -A;
  Matrix<int, 2, 3> expected = {
      {-1, 0, 2},
      {0,  0, 0}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixNegationIntTests, NegatePositiveOnly)
{
  Matrix<int, 1, 4> A = {
      {10, 20, 30, 40}
  };
  Matrix<int, 1, 4> result = -A;
  Matrix<int, 1, 4> expected = {
      {-10, -20, -30, -40}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixNegationIntTests, NegateNegativeOnly)
{
  Matrix<int, 3, 1> A = {{-5}, {-15}, {-25}};
  Matrix<int, 3, 1> result = -A;
  Matrix<int, 3, 1> expected = {{5}, {15}, {25}};
  EXPECT_EQ(result, expected);
}

TEST(MatrixNegationIntTests, NegateIntMinMax)
{
  // Test with INT_MAX and INT_MIN
  // Note: Negating INT_MIN in standard C++ is technically undefined behavior
  // for signed integers if representation is two's complement and no padding bits,
  // as it cannot be represented as a positive int.
  // However, element-wise negation T b = -a; usually works as expected by the hardware
  // (often resulting in INT_MIN again due to wrap-around/representation).
  // We test the *expected behavior* based on typical implementations.
  int int_max = std::numeric_limits<int>::max();
  int int_min = std::numeric_limits<int>::min();

  Matrix<int, 2, 2> A = {
      {int_max, int_min},
      {0,       1      }
  };
  Matrix<int, 2, 2> result = -A;

  // Expected: -INT_MAX is representable. -INT_MIN often results in INT_MIN.
  Matrix<int, 2, 2> expected = {
      {-int_max, int_min}, // Assuming -INT_MIN yields INT_MIN
      {0,        -1     }
  };
  // If your system behaves differently for -INT_MIN (e.g., traps or wraps differently), adjust 'expected'.
  EXPECT_EQ(result, expected);
}

TEST(MatrixNegationFloatTests, NegateMixedSigns)
{
  Matrix<float, 2, 2> A = {
      {1.5f,  -2.5f},
      {-3.0f, 4.0f }
  };
  Matrix<float, 2, 2> result = -A;
  Matrix<float, 2, 2> expected = {
      {-1.5f, 2.5f },
      {3.0f,  -4.0f}
  };
  _assertRoughlyEqual(result, expected);  // Use your existing helper
}

TEST(MatrixNegationFloatTests, NegateZeros)
{
  Matrix<float, 2, 2> A = {
      {1.0f,  0.0f },
      {-0.0f, -2.0f}  // Include negative zero
  };
  Matrix<float, 2, 2> result = -A;
  Matrix<float, 2, 2> expected = {
      {-1.0f, -0.0f}, // Negating 0.0f gives -0.0f
      {0.0f,  2.0f }  // Negating -0.0f gives 0.0f
  };
  _assertRoughlyEqual(result, expected);
  // Optionally, add specific checks for the sign of zero if critical
  EXPECT_EQ(std::signbit(result(0, 1)), true);   // Expect -0.0f
  EXPECT_EQ(std::signbit(result(1, 0)), false);  // Expect +0.0f
}

TEST(MatrixNegationFloatTests, NegatePositiveOnly)
{
  Matrix<float, 1, 3> A = {
      {1.1f, 2.2f, 3.3f}
  };
  Matrix<float, 1, 3> result = -A;
  Matrix<float, 1, 3> expected = {
      {-1.1f, -2.2f, -3.3f}
  };
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixNegationFloatTests, NegateNegativeOnly)
{
  Matrix<float, 3, 1> A = {{-0.5f}, {-1.5f}, {-2.5f}};
  Matrix<float, 3, 1> result = -A;
  Matrix<float, 3, 1> expected = {{0.5f}, {1.5f}, {2.5f}};
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixNegationFloatTests, NegateInfinities)
{
  float pinf = std::numeric_limits<float>::infinity();
  float ninf = -pinf;
  Matrix<float, 2, 2> A = {
      {pinf, ninf},
      {1.0f, 0.0f}
  };
  Matrix<float, 2, 2> result = -A;
  Matrix<float, 2, 2> expected = {
      {ninf,  pinf },
      {-1.0f, -0.0f}  // Note: -(-0.0f) is 0.0f, -0.0f remains -0.0f
  };
  // Use the improved helper function
  assertFloatMatrixEqualWithNaN(result, expected, 1e-5f);  // Adjust tolerance if needed
}

TEST(MatrixNegationFloatTests, NegateNaN)
{
  float nan = std::numeric_limits<float>::quiet_NaN();
  Matrix<float, 2, 3> A = {
      {1.0f, nan,   -2.0f},
      {nan,  -3.0f, 0.0f }
  };
  Matrix<float, 2, 3> result = -A;
  Matrix<float, 2, 3> expected = {
      {-1.0f, nan,  2.0f },
      {nan,   3.0f, -0.0f}
  };

  // Use the improved helper function
  assertFloatMatrixEqualWithNaN(result, expected);
}

TEST(MatrixNegationFloatTests, NegateDifferentDims)
{
  Matrix<float, 2, 1> A = {{10.0f}, {-20.0f}};
  Matrix<float, 2, 1> result = -A;
  Matrix<float, 2, 1> expected = {{-10.0f}, {20.0f}};
  _assertRoughlyEqual(result, expected);
}

// --- Test Suite for Matrix Division ---

// Element-wise division tests for integers
TEST(MatrixDivisionIntTests, ElementWiseDivision)
{
  Matrix<int, 2, 2> A = {
      {10, 20},
      {30, 40}
  };
  Matrix<int, 2, 2> B = {
      {2, 4},
      {5, 8}
  };
  Matrix<int, 2, 2> result = A / B;
  Matrix<int, 2, 2> expected = {
      {5, 5},
      {6, 5}  // Integer division: 30/5=6, 40/8=5
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixDivisionIntTests, ElementWiseDivisionWithNegatives)
{
  Matrix<int, 2, 2> A = {
      {-10, 20 },
      {30,  -40}
  };
  Matrix<int, 2, 2> B = {
      {2,  -4},
      {-5, 8 }
  };
  Matrix<int, 2, 2> result = A / B;
  Matrix<int, 2, 2> expected = {
      {-5, -5},
      {-6, -5}
  };
  EXPECT_EQ(result, expected);
}

// Scalar division tests for integers
TEST(MatrixDivisionIntTests, MatrixDividedByScalar)
{
  Matrix<int, 2, 3> A = {
      {10, 20, 30},
      {40, 50, 60}
  };
  int scalar = 10;
  Matrix<int, 2, 3> result = A / scalar;
  Matrix<int, 2, 3> expected = {
      {1, 2, 3},
      {4, 5, 6}
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixDivisionIntTests, ScalarDividedByMatrix)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {4, 5}
  };
  int scalar = 20;
  Matrix<int, 2, 2> result = scalar / A;
  Matrix<int, 2, 2> expected = {
      {20, 10},
      {5,  4 }  // Integer division: 20/4=5, 20/5=4
  };
  EXPECT_EQ(result, expected);
}

TEST(MatrixDivisionIntTests, DivideEquals)
{
  Matrix<int, 2, 2> A = {
      {100, 200},
      {300, 400}
  };
  A /= 10;
  Matrix<int, 2, 2> expected = {
      {10, 20},
      {30, 40}
  };
  EXPECT_EQ(A, expected);
}

// Element-wise division tests for floats
TEST(MatrixDivisionFloatTests, ElementWiseDivision)
{
  Matrix<float, 2, 2> A = {
      {10.0f, 20.0f},
      {30.0f, 40.0f}
  };
  Matrix<float, 2, 2> B = {
      {2.0f, 4.0f},
      {5.0f, 8.0f}
  };
  Matrix<float, 2, 2> result = A / B;
  Matrix<float, 2, 2> expected = {
      {5.0f, 5.0f},
      {6.0f, 5.0f}
  };
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixDivisionFloatTests, ElementWiseDivisionWithNegatives)
{
  Matrix<float, 2, 2> A = {
      {-10.0f, 20.0f },
      {30.0f,  -40.0f}
  };
  Matrix<float, 2, 2> B = {
      {2.0f,  -4.0f},
      {-5.0f, 8.0f }
  };
  Matrix<float, 2, 2> result = A / B;
  Matrix<float, 2, 2> expected = {
      {-5.0f, -5.0f},
      {-6.0f, -5.0f}
  };
  _assertRoughlyEqual(result, expected);
}

// Scalar division tests for floats
TEST(MatrixDivisionFloatTests, MatrixDividedByScalar)
{
  Matrix<float, 2, 3> A = {
      {1.0f, 2.0f, 3.0f},
      {4.0f, 5.0f, 6.0f}
  };
  float scalar = 2.0f;
  Matrix<float, 2, 3> result = A / scalar;
  Matrix<float, 2, 3> expected = {
      {0.5f, 1.0f, 1.5f},
      {2.0f, 2.5f, 3.0f}
  };
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixDivisionFloatTests, ScalarDividedByMatrix)
{
  Matrix<float, 2, 2> A = {
      {1.0f, 2.0f},
      {4.0f, 0.5f}
  };
  float scalar = 1.0f;
  Matrix<float, 2, 2> result = scalar / A;
  Matrix<float, 2, 2> expected = {
      {1.0f,  0.5f},
      {0.25f, 2.0f}
  };
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixDivisionFloatTests, DivideEquals)
{
  Matrix<float, 2, 2> A = {
      {10.0f, 20.0f},
      {30.0f, 40.0f}
  };
  A /= 10.0f;
  Matrix<float, 2, 2> expected = {
      {1.0f, 2.0f},
      {3.0f, 4.0f}
  };
  _assertRoughlyEqual(A, expected);
}

// Special cases for floating point division
TEST(MatrixDivisionFloatTests, DivisionByZero)
{
  Matrix<float, 2, 2> A = {
      {1.0f, -1.0f},
      {0.0f, 2.0f }
  };
  Matrix<float, 2, 2> B = {
      {1.0f, 0.0f},
      {0.0f, 2.0f}
  };
  Matrix<float, 2, 2> result = A / B;

  // Division by zero produces infinity (or -infinity)
  EXPECT_FLOAT_EQ(result(0, 0), 1.0f);
  EXPECT_TRUE(std::isinf(result(0, 1)));
  EXPECT_TRUE(result(0, 1) < 0);          // Should be negative infinity
  EXPECT_TRUE(std::isnan(result(1, 0)));  // 0/0 = NaN
  EXPECT_FLOAT_EQ(result(1, 1), 1.0f);
}

TEST(MatrixDivisionFloatTests, DivisionWithInfinity)
{
  float inf = std::numeric_limits<float>::infinity();
  Matrix<float, 2, 2> A = {
      {inf,  10.0f},
      {-inf, 0.0f }
  };
  Matrix<float, 2, 2> B = {
      {2.0f, inf},
      {inf,  inf}
  };
  Matrix<float, 2, 2> result = A / B;

  EXPECT_TRUE(std::isinf(result(0, 0)));  // inf/2 = inf
  EXPECT_FLOAT_EQ(result(0, 1), 0.0f);    // 10/inf = 0
  EXPECT_TRUE(std::isnan(result(1, 0)));  // -inf/inf = NaN
  EXPECT_FLOAT_EQ(result(1, 1), 0.0f);    // 0/inf = 0
}

// Test chained operations with division
TEST(MatrixDivisionFloatTests, ChainedOperations)
{
  Matrix<float, 2, 2> A = {
      {10.0f, 20.0f},
      {30.0f, 40.0f}
  };
  Matrix<float, 2, 2> B = {
      {2.0f, 4.0f},
      {5.0f, 8.0f}
  };
  Matrix<float, 2, 2> C = {
      {1.0f, 1.0f},
      {2.0f, 2.0f}
  };

  // Test (A + B) / C
  Matrix<float, 2, 2> result1 = (A + B) / C;
  Matrix<float, 2, 2> expected1 = {
      {12.0f, 24.0f},
      {17.5f, 24.0f}
  };
  _assertRoughlyEqual(result1, expected1);

  // Test A / B + C
  Matrix<float, 2, 2> result2 = A / B + C;
  Matrix<float, 2, 2> expected2 = {
      {6.0f, 6.0f},
      {8.0f, 7.0f}
  };
  _assertRoughlyEqual(result2, expected2);

  // Test (A * 2.0f) / (B - 1.0f)
  Matrix<float, 2, 2> result3 = (A * 2.0f) / (B - 1.0f);
  Matrix<float, 2, 2> expected3 = {
      {20.0f, 40.0f / 3.0f},
      {15.0f, 80.0f / 7.0f}
  };
  _assertRoughlyEqual(result3, expected3);
}

// Test division with different matrix dimensions
TEST(MatrixDivisionFloatTests, DifferentDimensions)
{
  // 1x3 matrix
  Matrix<float, 1, 3> A = {
      {6.0f, 8.0f, 10.0f}
  };
  Matrix<float, 1, 3> B = {
      {2.0f, 4.0f, 5.0f}
  };
  Matrix<float, 1, 3> result = A / B;
  Matrix<float, 1, 3> expected = {
      {3.0f, 2.0f, 2.0f}
  };
  _assertRoughlyEqual(result, expected);

  // 3x1 matrix
  Matrix<float, 3, 1> C = {{12.0f}, {15.0f}, {18.0f}};
  Matrix<float, 3, 1> D = {{3.0f}, {5.0f}, {6.0f}};
  Matrix<float, 3, 1> result2 = C / D;
  Matrix<float, 3, 1> expected2 = {{4.0f}, {3.0f}, {3.0f}};
  _assertRoughlyEqual(result2, expected2);
}