#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "Matrix.h"
#include "MatrixTestHelpers.h"

TEST(MatrixIntTests, EqualityTest)
{
  Matrix<int, 2, 2> A = {
      {0, 0},
      {0, 1}
  };
  Matrix<int, 2, 2> B = {
      {0, 0},
      {0, 1}
  };
  EXPECT_TRUE(A == B);
}

TEST(MatrixIntTests, InequalityTest)
{
  Matrix<int, 2, 2> A = {
      {0, 0},
      {0, 0}
  };
  Matrix<int, 2, 2> B = {
      {0, 0},
      {0, 1}
  };
  EXPECT_TRUE(A != B);
}

// ========================================================================
// == MATRIX CONSTRUCTOR TESTS (INTEGER) ==
// ========================================================================

TEST(MatrixConstructorIntTests, DefaultConstructor)
{
  Matrix<int, 2, 2> A = {
      {0, 0},
      {0, 0}
  };
  const auto B = Matrix<int, 2, 2>();
  EXPECT_EQ(A, B);
}

TEST(MatrixConstructorIntTests, ScalarInitializedConstructor)
{
  Matrix<int, 2, 2> A = {
      {3, 3},
      {3, 3}
  };
  const auto B = Matrix<int, 2, 2>(3);
  EXPECT_EQ(A, B);
}

// ========================================================================
// == MATRIX CONSTRUCTOR TESTS (FLOAT) ==
// ========================================================================

TEST(MatrixConstructorFloatTests, DefaultConstructor)
{
  Matrix<float, 2, 2> A = {
      {0.0f, 0.0f},
      {0.0f, 0.0f}
  };
  const auto B = Matrix<float, 2, 2>();
  EXPECT_EQ(A, B);
}

TEST(MatrixConstructorFloatTests, ScalarInitializedConstructor)
{
  Matrix<float, 2, 2> A = {
      {3.1f, 3.1f},
      {3.1f, 3.1f}
  };
  const auto B = Matrix<float, 2, 2>(3.1f);
  EXPECT_EQ(A, B);
}

// ========================================================================
// == MATRIX NORM TESTS (INTEGER) ==
// ========================================================================

TEST(MatrixNormIntTests, ZeroMatrix)
{
  // The norm of a zero matrix is 0.
  Matrix<int, 2, 2> A = {
      {0, 0},
      {0, 0}
  };
  EXPECT_EQ(A.norm(), 0);  // sqrt(0) = 0
}

TEST(MatrixNormIntTests, IdentityMatrix)
{
  // Norm of 2x2 identity matrix
  Matrix<int, 2, 2> A = {
      {1, 0},
      {0, 1}
  };
  // True Norm = sqrt(1^2 + 0^2 + 0^2 + 1^2) = sqrt(1 + 1) = sqrt(2) approx 1.414
  // Since the function returns int, expect truncation.
  EXPECT_EQ(A.norm(), 1) << "Expecting integer truncation for sqrt(2)";
}

TEST(MatrixNormIntTests, PerfectSquareNorm)
{
  // A matrix whose squared elements sum to a perfect square.
  Matrix<int, 2, 2> A = {
      {3, 0},
      {0, 4}
  };
  // Norm = sqrt(3^2 + 0^2 + 0^2 + 4^2) = sqrt(9 + 16) = sqrt(25) = 5
  EXPECT_EQ(A.norm(), 5);
}

TEST(MatrixNormIntTests, PerfectSquareNormNegativeValues)
{
  // The square operation makes signs irrelevant for the norm calculation.
  Matrix<int, 2, 2> A = {
      {-3, 0 },
      {0,  -4}
  };
  // Norm = sqrt((-3)^2 + 0^2 + 0^2 + (-4)^2) = sqrt(9 + 16) = sqrt(25) = 5
  EXPECT_EQ(A.norm(), 5);
}

TEST(MatrixNormIntTests, VectorPerfectSquareNorm)
{
  // Test with a row vector (1xN matrix).
  Matrix<int, 1, 2> A = {
      {5, 12}
  };
  // Norm = sqrt(5^2 + 12^2) = sqrt(25 + 144) = sqrt(169) = 13
  EXPECT_EQ(A.norm(), 13);
}

TEST(MatrixNormIntTests, NonPerfectSquareNormTruncated)
{
  // A matrix whose norm is not an integer.
  Matrix<int, 2, 1> A = {{2}, {2}};  // Column vector
  // True Norm = sqrt(2^2 + 2^2) = sqrt(4 + 4) = sqrt(8) approx 2.828
  // Since the function returns int, expect truncation.
  EXPECT_EQ(A.norm(), 2) << "Expecting integer truncation for sqrt(8)";
}

TEST(MatrixNormIntTests, GeneralCaseTruncated)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  // True Norm = sqrt(1^2 + 2^2 + 3^2 + 4^2) = sqrt(1 + 4 + 9 + 16) = sqrt(30) approx 5.477
  // Since the function returns int, expect truncation.
  EXPECT_EQ(A.norm(), 5) << "Expecting integer truncation for sqrt(30)";
}

// ========================================================================
// == MATRIX NORM TESTS (FLOAT) ==
// ========================================================================

TEST(MatrixNormFloatTests, ZeroMatrix)
{
  // The norm of a zero matrix is 0.
  Matrix<float, 2, 3> A = {
      {0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f}
  };
  // Use EXPECT_FLOAT_EQ for potentially exact floating point zero.
  EXPECT_FLOAT_EQ(A.norm(), 0.0f);
}

TEST(MatrixNormFloatTests, IdentityMatrix)
{
  // Norm of 2x2 identity matrix.
  Matrix<float, 2, 2> A = {
      {1.0f, 0.0f},
      {0.0f, 1.0f}
  };
  // Norm = sqrt(1^2 + 0^2 + 0^2 + 1^2) = sqrt(1.0 + 1.0) = sqrt(2.0)
  float expected_norm = std::sqrt(2.0f);
  EXPECT_NEAR(A.norm(), expected_norm, 1e-6f);
}

TEST(MatrixNormFloatTests, SimpleNorm)
{
  Matrix<float, 2, 2> A = {
      {3.0f, 0.0f },
      {0.0f, -4.0f}
  };
  // Norm = sqrt(3^2 + 0^2 + 0^2 + (-4)^2) = sqrt(9 + 16) = sqrt(25.0) = 5.0f
  // Use EXPECT_FLOAT_EQ when the expected result should be exact.
  EXPECT_FLOAT_EQ(A.norm(), 5.0f);
}

TEST(MatrixNormFloatTests, VectorNorm)
{
  // Test with a column vector (Nx1 matrix).
  Matrix<float, 3, 1> A = {{1.5f}, {-2.0f}, {0.5f}};
  // Norm = sqrt(1.5^2 + (-2.0)^2 + 0.5^2) = sqrt(2.25 + 4.0 + 0.25) = sqrt(6.5)
  float expected_norm = std::sqrt(6.5f);
  EXPECT_NEAR(A.norm(), expected_norm, 1e-6f);
}

TEST(MatrixNormFloatTests, GeneralCase)
{
  Matrix<float, 2, 3> A = {
      {1.0f, -1.0f, 2.0f },
      {0.0f, 3.0f,  -0.5f}
  };
  // Norm = sqrt(1^2 + (-1)^2 + 2^2 + 0^2 + 3^2 + (-0.5)^2)
  //      = sqrt(1 + 1 + 4 + 0 + 9 + 0.25) = sqrt(15.25)
  float expected_norm = std::sqrt(15.25f);
  EXPECT_NEAR(A.norm(), expected_norm, 1e-6f);
}

// ========================================================================
// == MATRIX TRANSPOSE TESTS (INTEGER) ==
// ========================================================================

TEST(MatrixTransposeIntTests, SquareMatrix)
{
  // Input Matrix (2x2)
  const Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };

  // Expected Transposed Matrix (2x2)
  Matrix<int, 2, 2> expected = {
      {1, 3},
      {2, 4}
  };

  auto result = A.transpose();  // Result type is Matrix<int, 2, 2>

  // For integers, direct comparison is fine
  EXPECT_EQ(result, expected);
}

TEST(MatrixTransposeIntTests, TallMatrix)
{
  // Input Matrix (3x2)
  const Matrix<int, 3, 2> A = {
      {1, 2},
      {3, 4},
      {5, 6}
  };

  // Expected Transposed Matrix (2x3)
  Matrix<int, 2, 3> expected = {
      {1, 3, 5},
      {2, 4, 6}
  };

  auto result = A.transpose();  // Result type is Matrix<int, 2, 3>
  EXPECT_EQ(result, expected);
}

TEST(MatrixTransposeIntTests, WideMatrix)
{
  // Input Matrix (2x3)
  const Matrix<int, 2, 3> A = {
      {1, 2, 3},
      {4, 5, 6}
  };

  // Expected Transposed Matrix (3x2)
  Matrix<int, 3, 2> expected = {
      {1, 4},
      {2, 5},
      {3, 6}
  };

  auto result = A.transpose();  // Result type is Matrix<int, 3, 2>
  EXPECT_EQ(result, expected);
}

TEST(MatrixTransposeIntTests, RowVector)
{
  // Input Matrix (1x4) - A Row Vector
  const Matrix<int, 1, 4> A = {
      {10, -20, 0, 30}
  };

  // Expected Transposed Matrix (4x1) - A Column Vector
  Matrix<int, 4, 1> expected = {{10}, {-20}, {0}, {30}};

  auto result = A.transpose();  // Result type is Matrix<int, 4, 1>
  EXPECT_EQ(result, expected);
}

TEST(MatrixTransposeIntTests, ColumnVector)
{
  // Input Matrix (3x1) - A Column Vector
  const Matrix<int, 3, 1> A = {{5}, {-6}, {7}};

  // Expected Transposed Matrix (1x3) - A Row Vector
  Matrix<int, 1, 3> expected = {
      {5, -6, 7}
  };

  auto result = A.transpose();  // Result type is Matrix<int, 1, 3>
  EXPECT_EQ(result, expected);
}

TEST(MatrixTransposeIntTests, ScalarMatrix)
{
  // Input Matrix (1x1)
  const Matrix<int, 1, 1> A = {{-99}};

  // Expected Transposed Matrix (1x1) - transpose of 1x1 is itself
  Matrix<int, 1, 1> expected = {{-99}};

  auto result = A.transpose();  // Result type is Matrix<int, 1, 1>
  EXPECT_EQ(result, expected);
}

// ========================================================================
// == MATRIX TRANSPOSE TESTS (FLOAT) ==
// ========================================================================

TEST(MatrixTransposeFloatTests, SquareMatrix)
{
  // Input Matrix (2x2)
  // Declared non-const only because _assertRoughlyEqual expects non-const
  const Matrix<float, 2, 2> A = {
      {1.1f, -2.2f},
      {3.3f, 0.0f }
  };

  // Expected Transposed Matrix (2x2)
  Matrix<float, 2, 2> expected = {
      {1.1f,  3.3f},
      {-2.2f, 0.0f}
  };

  auto result = A.transpose();  // Result type is Matrix<float, 2, 2>

  // Use the provided helper function
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixTransposeFloatTests, TallMatrix)
{
  // Input Matrix (3x2)
  const Matrix<float, 3, 2> A = {
      {1.5f,  2.5f },
      {-3.5f, 4.5f },
      {0.0f,  -5.5f}
  };

  // Expected Transposed Matrix (2x3)
  Matrix<float, 2, 3> expected = {
      {1.5f, -3.5f, 0.0f },
      {2.5f, 4.5f,  -5.5f}
  };

  auto result = A.transpose();  // Result type is Matrix<float, 2, 3>
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixTransposeFloatTests, WideMatrix)
{
  // Input Matrix (2x3)
  const Matrix<float, 2, 3> A = {
      {1.5f, -3.5f, 0.0f },
      {2.5f, 4.5f,  -5.5f}
  };

  // Expected Transposed Matrix (3x2)
  Matrix<float, 3, 2> expected = {
      {1.5f,  2.5f },
      {-3.5f, 4.5f },
      {0.0f,  -5.5f}
  };

  auto result = A.transpose();  // Result type is Matrix<float, 3, 2>
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixTransposeFloatTests, RowVector)
{
  // Input Matrix (1x3) - Row Vector
  const Matrix<float, 1, 3> A = {
      {10.1f, -20.2f, 30.3f}
  };

  // Expected Transposed Matrix (3x1) - Column Vector
  Matrix<float, 3, 1> expected = {{10.1f}, {-20.2f}, {30.3f}};

  auto result = A.transpose();  // Result type is Matrix<float, 3, 1>
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixTransposeFloatTests, ColumnVector)
{
  // Input Matrix (3x1) - Column Vector
  const Matrix<float, 3, 1> A = {{5.5f}, {-6.6f}, {7.7f}};

  // Expected Transposed Matrix (1x3) - Row Vector
  Matrix<float, 1, 3> expected = {
      {5.5f, -6.6f, 7.7f}
  };

  auto result = A.transpose();  // Result type is Matrix<float, 1, 3>
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixTransposeFloatTests, ScalarMatrix)
{
  // Input Matrix (1x1)
  const Matrix<float, 1, 1> A = {{-9.9f}};

  // Expected Transposed Matrix (1x1)
  Matrix<float, 1, 1> expected = {{-9.9f}};

  auto result = A.transpose();  // Result type is Matrix<float, 1, 1>
  _assertRoughlyEqual(result, expected);
}

// ========================================================================
// == MATRIX INVERT TESTS (FLOAT) ==
// ========================================================================

TEST(MatrixInvertFloatTests, IdentityMatrix)
{
  const auto I = Matrix<float, 3, 3>::identity();
  auto inverted_I = I.invert();
  // Use helper for float comparison
  _assertRoughlyEqual(inverted_I, I);
}

TEST(MatrixInvertFloatTests, Simple2x2)
{
  // Input matrix
  const Matrix<float, 2, 2> A = {
      {1.0f, 2.0f}, // Non-const due to helper :(
      {3.0f, 4.0f}
  };
  // Determinant = 1*4 - 2*3 = -2.0f
  // Expected Inverse = (1/-2.0f) * {{4.0f, -2.0f}, {-3.0f, 1.0f}}
  //                  = {{-2.0f, 1.0f}, {1.5f, -0.5f}}
  Matrix<float, 2, 2> expected_inv = {
      {-2.0f, 1.0f },
      {1.5f,  -0.5f}
  };

  auto A_inv = A.invert();
  _assertRoughlyEqual(A_inv, expected_inv);

  // Verify the core property: A * A_inv == I
  Matrix<float, 2, 2> I = Matrix<float, 2, 2>::identity();
  auto product = A * A_inv;  // Assuming operator* works
  _assertRoughlyEqual(product, I);
}

TEST(MatrixInvertFloatTests, General3x3)
{
  // Choose a known invertible 3x3 matrix
  const Matrix<float, 3, 3> A = {
      {1.0f, 2.0f, 3.0f}, // Non-const due to helper
      {0.0f, 1.0f, 4.0f},
      {5.0f, 6.0f, 0.0f}
  };
  // Determinant = 1(0 - 24) - 2(0 - 20) + 3(0 - 5) = -24 + 40 - 15 = 1.0f

  // Calculating expected inverse manually is tedious.
  // We rely primarily on the property A * A_inv = I
  auto A_inv = A.invert();
  Matrix<float, 3, 3> I = Matrix<float, 3, 3>::identity();
  auto product = A * A_inv;  // Assuming operator* works

  _assertRoughlyEqual(product, I);

  // We can also check A_inv * A = I
  auto product_inv_A = A_inv * A;
  _assertRoughlyEqual(product_inv_A, I);
}

TEST(MatrixInvertFloatTests, SingularMatrix)
{
  // Matrix with determinant close to 0
  const Matrix<float, 2, 2> A = {
      {1.0f, 2.0f},
      {2.0f, 4.0f}
  };  // Det = 1*4 - 2*2 = 0
  // Expect invert() to throw an exception (adjust exception type if needed)
  EXPECT_THROW(A.invert(), std::runtime_error) << "Inverting a singular float matrix should throw";
}

// ========================================================================
// == MATRIX DOT PRODUCT TESTS (INTEGER) ==
// == (Frobenius Inner Product) ==
// ========================================================================

TEST(MatrixDotIntTests, DotWithZeroMatrix)
{
  const Matrix<int, 2, 3> A = {
      {1, 2, 3},
      {4, 5, 6}
  };
  const Matrix<int, 2, 3> Zeros = {
      {0, 0, 0},
      {0, 0, 0}
  };
  // Dot product with zero matrix is always 0
  EXPECT_EQ(A.dot(Zeros), 0);
  EXPECT_EQ(Zeros.dot(A), 0);  // Dot product is commutative
}

TEST(MatrixDotIntTests, SelfDotProductIsSumOfSquares)
{
  // The dot product of a matrix with itself is the sum of squares of its elements
  const Matrix<int, 2, 2> A = {
      {1, -2},
      {3, 0 }
  };
  // Dot = 1*1 + (-2)*(-2) + 3*3 + 0*0 = 1 + 4 + 9 + 0 = 14
  int expected = 14;
  EXPECT_EQ(A.dot(A), expected);
}

TEST(MatrixDotIntTests, GeneralCaseSquare)
{
  const Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  const Matrix<int, 2, 2> B = {
      {-1, 0 },
      {5,  -6}
  };
  // Dot = (1 * -1) + (2 * 0) + (3 * 5) + (4 * -6)
  //     = -1 + 0 + 15 - 24 = -10
  int expected = -10;
  EXPECT_EQ(A.dot(B), expected);
  EXPECT_EQ(B.dot(A), expected);  // Commutative check
}

TEST(MatrixDotIntTests, GeneralCaseRectangular)
{
  const Matrix<int, 3, 2> A = {
      {1,  0 },
      {-1, 2 },
      {3,  -3}
  };
  const Matrix<int, 3, 2> B = {
      {5, 1},
      {0, 1},
      {2, 2}
  };
  // Dot = (1*5 + 0*1) + (-1*0 + 2*1) + (3*2 + -3*2)
  //     = (5 + 0) + (0 + 2) + (6 - 6)
  //     = 5 + 2 + 0 = 7
  int expected = 7;
  EXPECT_EQ(A.dot(B), expected);
  EXPECT_EQ(B.dot(A), expected);
}

TEST(MatrixDotIntTests, VectorCaseOrthogonal)
{
  // Interpreting 1xN matrices as vectors
  const Matrix<int, 1, 3> A = {
      {2, 1, -2}
  };
  const Matrix<int, 1, 3> B = {
      {3, -4, 1}
  };
  // Dot = (2*3) + (1*-4) + (-2*1) = 6 - 4 - 2 = 0
  // These vectors are orthogonal
  int expected = 0;
  EXPECT_EQ(A.dot(B), expected);
  EXPECT_EQ(B.dot(A), expected);
}

TEST(MatrixDotIntTests, VectorCaseGeneral)
{
  // Interpreting Nx1 matrices as vectors
  const Matrix<int, 4, 1> A = {{1}, {0}, {-1}, {2}};
  const Matrix<int, 4, 1> B = {{3}, {10}, {5}, {-2}};
  // Dot = (1*3) + (0*10) + (-1*5) + (2*-2)
  //     = 3 + 0 - 5 - 4 = -6
  int expected = -6;
  EXPECT_EQ(A.dot(B), expected);
  EXPECT_EQ(B.dot(A), expected);
}

// ========================================================================
// == MATRIX DOT PRODUCT TESTS (FLOAT) ==
// == (Frobenius Inner Product) ==
// ========================================================================

TEST(MatrixDotFloatTests, DotWithZeroMatrix)
{
  const Matrix<float, 2, 2> A = {
      {1.5f,  -0.5f},
      {10.0f, 0.1f }
  };
  const Matrix<float, 2, 2> Zeros = {
      {0.0f, 0.0f},
      {0.0f, 0.0f}
  };
  EXPECT_FLOAT_EQ(A.dot(Zeros), 0.0f);
  EXPECT_FLOAT_EQ(Zeros.dot(A), 0.0f);
}

TEST(MatrixDotFloatTests, SelfDotProductIsSumOfSquares)
{
  const Matrix<float, 1, 3> A = {
      {1.5f, -2.0f, 0.5f}
  };
  // Dot = 1.5*1.5 + (-2.0)*(-2.0) + 0.5*0.5
  //     = 2.25 + 4.0 + 0.25 = 6.5f
  float expected = 6.5f;
  // Calculation is exact in this case
  EXPECT_FLOAT_EQ(A.dot(A), expected);
}

TEST(MatrixDotFloatTests, GeneralCaseSquare)
{
  const Matrix<float, 2, 2> A = {
      {0.5f, 1.5f },
      {2.0f, -1.0f}
  };
  const Matrix<float, 2, 2> B = {
      {4.0f, -2.0f},
      {0.0f, 1.0f }
  };
  // Dot = (0.5*4.0) + (1.5*-2.0) + (2.0*0.0) + (-1.0*1.0)
  //     = 2.0 - 3.0 + 0.0 - 1.0 = -2.0f
  float expected = -2.0f;
  EXPECT_FLOAT_EQ(A.dot(B), expected);
  EXPECT_FLOAT_EQ(B.dot(A), expected);
}

TEST(MatrixDotFloatTests, GeneralCaseRectangular)
{
  const Matrix<float, 2, 3> A = {
      {1.0f, 2.0f, 3.0f },
      {0.5f, 0.0f, -0.5f}
  };
  const Matrix<float, 2, 3> B = {
      {-1.0f, 0.0f, 1.0f },
      {2.0f,  4.0f, -2.0f}
  };
  // Dot = (1*-1 + 2*0 + 3*1) + (0.5*2 + 0*4 + -0.5*-2)
  //     = (-1 + 0 + 3) + (1.0 + 0 + 1.0)
  //     = 2 + 2.0 = 4.0f
  float expected = 4.0f;
  EXPECT_FLOAT_EQ(A.dot(B), expected);
  EXPECT_FLOAT_EQ(B.dot(A), expected);
}

TEST(MatrixDotFloatTests, VectorCaseOrthogonal)
{
  // Vectors [1, 0.5] and [-0.5, 1] are orthogonal
  const Matrix<float, 1, 2> A = {
      {1.0f, 0.5f}
  };
  const Matrix<float, 1, 2> B = {
      {-0.5f, 1.0f}
  };
  // Dot = (1.0 * -0.5) + (0.5 * 1.0) = -0.5 + 0.5 = 0.0f
  float expected = 0.0f;
  EXPECT_FLOAT_EQ(A.dot(B), expected);
  EXPECT_FLOAT_EQ(B.dot(A), expected);
}

TEST(MatrixDotFloatTests, VectorCaseGeneral)
{
  // Treat Nx1 matrices as vectors
  const Matrix<float, 3, 1> A = {{1.5f}, {0.0f}, {-0.5f}};
  const Matrix<float, 3, 1> B = {{2.0f}, {10.0f}, {4.0f}};
  // Dot = (1.5*2.0) + (0.0*10.0) + (-0.5*4.0)
  //     = 3.0 + 0.0 - 2.0 = 1.0f
  float expected = 1.0f;
  EXPECT_FLOAT_EQ(A.dot(B), expected);
  EXPECT_FLOAT_EQ(B.dot(A), expected);
}

// ========================================================================
// == MATRIX SUBMATRIX TESTS (INTEGER) - CORRECTED EXPECT_THROW ==
// ========================================================================

TEST(MatrixSubmatrixIntTests, ExtractMiddle)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 2, 2> expected = {
      {6,  7 },
      {10, 11}
  };
  auto result = A.submatrix<2, 2>(1, 1);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractTopLeft)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 2, 3> expected = {
      {1, 2, 3},
      {5, 6, 7}
  };
  auto result = A.submatrix<2, 3>(0, 0);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractBottomRight)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 2, 2> expected = {
      {7,  8 },
      {11, 12}
  };
  auto result = A.submatrix<2, 2>(1, 2);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractRow)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 1, 4> expected = {
      {5, 6, 7, 8}
  };
  auto result = A.submatrix<1, 4>(1, 0);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractColumn)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 3, 1> expected = {{3}, {7}, {11}};
  auto result = A.submatrix<3, 1>(0, 2);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractElement)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 1, 1> expected = {{10}};
  auto result = A.submatrix<1, 1>(2, 1);
  EXPECT_EQ(result, expected);
}

TEST(MatrixSubmatrixIntTests, ExtractFullMatrix)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  Matrix<int, 3, 4> expected = A;
  auto result = A.submatrix<3, 4>(0, 0);
  EXPECT_EQ(result, expected);
}

// --- Bounds Checking Tests (Integer) ---

TEST(MatrixSubmatrixIntTests, BoundsCheckRow)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  // CORRECTED: Added extra parentheses around A.submatrix<...>(...)
  EXPECT_THROW((A.submatrix<2, 2>(2, 0)), std::out_of_range)
      << "Submatrix starting row + subrows exceeds original rows";
}

TEST(MatrixSubmatrixIntTests, BoundsCheckCol)
{
  const Matrix<int, 3, 4> A = {
      {1, 2,  3,  4 },
      {5, 6,  7,  8 },
      {9, 10, 11, 12}
  };
  // CORRECTED: Added extra parentheses around A.submatrix<...>(...)
  EXPECT_THROW((A.submatrix<2, 3>(0, 2)), std::out_of_range)
      << "Submatrix starting col + subcols exceeds original cols";
}

// ========================================================================
// == MATRIX SUBMATRIX TESTS (FLOAT) - CORRECTED EXPECT_THROW ==
// ========================================================================

TEST(MatrixSubmatrixFloatTests, ExtractMiddle)
{
  const Matrix<float, 3, 3> A = {
      {1.1f, 1.2f, 1.3f}, // Non-const due to helper
      {2.1f, 2.2f, 2.3f},
      {3.1f, 3.2f, 3.3f}
  };
  Matrix<float, 2, 2> expected = {
      {2.2f, 2.3f},
      {3.2f, 3.3f}
  };
  auto result = A.submatrix<2, 2>(1, 1);
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixSubmatrixFloatTests, ExtractColumn)
{
  const Matrix<float, 3, 3> A = {
      {1.1f, 1.2f, 1.3f}, // Non-const due to helper
      {2.1f, 2.2f, 2.3f},
      {3.1f, 3.2f, 3.3f}
  };
  Matrix<float, 3, 1> expected = {{1.3f}, {2.3f}, {3.3f}};  // Corrected: Extract Col 2
  auto result = A.submatrix<3, 1>(0, 2);                    // Corrected: Start at col 2
  _assertRoughlyEqual(result, expected);
}

TEST(MatrixSubmatrixFloatTests, ExtractFullMatrix)
{
  const Matrix<float, 2, 2> A = {
      {1.0f,  -1.0f}, // Non-const due to helper
      {-2.0f, 2.0f }
  };
  Matrix<float, 2, 2> expected = A;
  auto result = A.submatrix<2, 2>(0, 0);
  _assertRoughlyEqual(result, expected);
}

// --- Bounds Checking Tests (Float) ---

TEST(MatrixSubmatrixFloatTests, BoundsCheckRowAndCol)
{
  const Matrix<float, 3, 3> A = {
      {1.1f, 1.2f, 1.3f},
      {2.1f, 2.2f, 2.3f},
      {3.1f, 3.2f, 3.3f}
  };
  // CORRECTED: Added extra parentheses around A.submatrix<...>(...)
  EXPECT_THROW((A.submatrix<2, 2>(2, 2)), std::out_of_range) << "Submatrix dimensions exceed bounds starting at (2, 2)";
}

TEST(MatrixSubmatrixFloatTests, BoundsCheckStartOutOfBounds)
{
  const Matrix<float, 3, 3> A = {
      {1.1f, 1.2f, 1.3f},
      {2.1f, 2.2f, 2.3f},
      {3.1f, 3.2f, 3.3f}
  };
  // CORRECTED: Added extra parentheses around A.submatrix<...>(...)
  EXPECT_THROW((A.submatrix<1, 1>(3, 0)), std::out_of_range) << "Submatrix starting row is out of bounds";
  // CORRECTED: Added extra parentheses around A.submatrix<...>(...)
  EXPECT_THROW((A.submatrix<1, 1>(0, 3)), std::out_of_range) << "Submatrix starting col is out of bounds";
}

// ========================================================================
// == MATRIX SETSUBMATRIX TESTS (INTEGER) ==
// ========================================================================

TEST(MatrixSetSubmatrixIntTests, SetMiddle)
{
  // Target matrix (MUST be non-const as it will be modified)
  Matrix<int, 3, 4> A = {
      {0, 0, 0, 0},
      {0, 0, 0, 0},
      {0, 0, 0, 0}
  };

  // Submatrix to insert (can be const)
  const Matrix<int, 2, 2> sub = {
      {9, 8},
      {7, 6}
  };

  // Expected state of A after setting
  // Use a separate variable for clarity
  Matrix<int, 3, 4> expected = {
      {0, 0, 0, 0},
      {0, 9, 8, 0},
      {0, 7, 6, 0}
  };

  // Perform the operation
  A.setSubmatrix<2, 2>(1, 1, sub);  // Set 2x2 starting at (1, 1)

  // Assert that A now matches the expected state
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetTopLeftOverwriting)
{
  // Start with non-zero data to test overwriting
  Matrix<int, 3, 3> A = {
      {9, 9, 9},
      {9, 9, 9},
      {9, 9, 9}
  };
  const Matrix<int, 2, 2> sub = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 3, 3> expected = {
      {1, 2, 9},
      {3, 4, 9},
      {9, 9, 9}
  };
  A.setSubmatrix<2, 2>(0, 0, sub);  // Set 2x2 starting at (0, 0)
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetBottomRight)
{
  Matrix<int, 3, 3> A = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}
  };
  const Matrix<int, 2, 2> sub = {
      {1, 2},
      {3, 4}
  };
  Matrix<int, 3, 3> expected = {
      {0, 0, 0},
      {0, 1, 2},
      {0, 3, 4}
  };
  A.setSubmatrix<2, 2>(1, 1, sub);  // Set 2x2 starting at (1, 1)
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetRow)
{
  Matrix<int, 3, 4> A = {
      {1, 1, 1, 1},
      {1, 1, 1, 1},
      {1, 1, 1, 1}
  };
  const Matrix<int, 1, 4> subRow = {
      {5, 6, 7, 8}
  };  // A row vector
  Matrix<int, 3, 4> expected = {
      {1, 1, 1, 1},
      {5, 6, 7, 8}, // Row 1 should be replaced
      {1, 1, 1, 1}
  };
  A.setSubmatrix<1, 4>(1, 0, subRow);  // Set row 1 starting at (1,0)
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetColumn)
{
  Matrix<int, 3, 4> A = {
      {1, 1, 1, 1},
      {1, 1, 1, 1},
      {1, 1, 1, 1}
  };
  const Matrix<int, 3, 1> subCol = {{10}, {20}, {30}};  // A column vector
  Matrix<int, 3, 4> expected = {
      {1, 10, 1, 1},
      {1, 20, 1, 1}, // Column 1 should be replaced
      {1, 30, 1, 1}
  };
  A.setSubmatrix<3, 1>(0, 1, subCol);  // Set column 1 starting at (0,1)
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetElement)
{
  Matrix<int, 2, 2> A = {
      {1, 2},
      {3, 4}
  };
  const Matrix<int, 1, 1> subElement = {{-9}};
  Matrix<int, 2, 2> expected = {
      {1,  2},
      {-9, 4}
  };
  A.setSubmatrix<1, 1>(1, 0, subElement);  // Set element at (1,0)
  EXPECT_EQ(A, expected);
}

TEST(MatrixSetSubmatrixIntTests, SetFullMatrix)
{
  Matrix<int, 2, 3> A = {
      {0, 0, 0},
      {0, 0, 0}
  };
  const Matrix<int, 2, 3> subFull = {
      {1, 2, 3},
      {4, 5, 6}
  };
  Matrix<int, 2, 3> expected = subFull;  // Expect A to become identical to sub
  A.setSubmatrix<2, 3>(0, 0, subFull);   // Set 2x3 starting at (0,0)
  EXPECT_EQ(A, expected);
}

// --- Bounds Checking Tests (Integer) ---

TEST(MatrixSetSubmatrixIntTests, BoundsCheckRow)
{
  Matrix<int, 3, 3> A;  // Target matrix (non-const)
  const Matrix<int, 2, 2> sub = {
      {1, 1},
      {1, 1}
  };  // Submatrix to insert
  // Attempt to set 2x2 starting at row 2 (needs rows 2, 3; A only has 0, 1, 2)
  // Remember extra parentheses for EXPECT_THROW argument!
  EXPECT_THROW((A.setSubmatrix<2, 2>(2, 0, sub)), std::out_of_range)
      << "setSubmatrix starting row + subrows exceeds original rows";
}

TEST(MatrixSetSubmatrixIntTests, BoundsCheckCol)
{
  Matrix<int, 3, 3> A;
  const Matrix<int, 2, 2> sub = {
      {1, 1},
      {1, 1}
  };
  // Attempt to set 2x2 starting at col 2 (needs cols 2, 3; A only has 0, 1, 2)
  EXPECT_THROW((A.setSubmatrix<2, 2>(0, 2, sub)), std::out_of_range)
      << "setSubmatrix starting col + subcols exceeds original cols";
}

TEST(MatrixSetSubmatrixIntTests, BoundsCheckStartOutOfBounds)
{
  Matrix<int, 3, 3> A;
  const Matrix<int, 1, 1> sub = {{1}};
  // Attempt to start placement outside the matrix bounds
  EXPECT_THROW((A.setSubmatrix<1, 1>(3, 0, sub)), std::out_of_range) << "setSubmatrix starting row is out of bounds";
  EXPECT_THROW((A.setSubmatrix<1, 1>(0, 3, sub)), std::out_of_range) << "setSubmatrix starting col is out of bounds";
}

// ========================================================================
// == MATRIX SETSUBMATRIX TESTS (FLOAT) ==
// ========================================================================

TEST(MatrixSetSubmatrixFloatTests, SetMiddle)
{
  // Target matrix (non-const)
  Matrix<float, 3, 3> A = {
      {9.9f, 9.9f, 9.9f},
      {9.9f, 9.9f, 9.9f},
      {9.9f, 9.9f, 9.9f}
  };
  const Matrix<float, 2, 2> sub = {
      {1.1f, 1.2f},
      {2.1f, 2.2f}
  };
  // Expected result (must be non-const for helper _assertRoughlyEqual)
  Matrix<float, 3, 3> expected = {
      {9.9f, 1.1f, 1.2f},
      {9.9f, 2.1f, 2.2f},
      {9.9f, 9.9f, 9.9f}
  };
  A.setSubmatrix<2, 2>(0, 1, sub);  // Set 2x2 starting at (0, 1)
  _assertRoughlyEqual(A, expected);
}

TEST(MatrixSetSubmatrixFloatTests, SetFullMatrix)
{
  Matrix<float, 2, 2> A = {
      {0.0f, 0.0f},
      {0.0f, 0.0f}
  };
  const Matrix<float, 2, 2> sub = {
      {1.5f,  -1.5f},
      {-2.5f, 2.5f }
  };
  Matrix<float, 2, 2> expected = sub;  // Make a copy for the helper
  A.setSubmatrix<2, 2>(0, 0, sub);
  _assertRoughlyEqual(A, expected);
}

TEST(MatrixSetSubmatrixFloatTests, SetRow)
{
  Matrix<float, 2, 3> A = {
      {1.0f, 2.0f, 3.0f},
      {4.0f, 5.0f, 6.0f}
  };
  const Matrix<float, 1, 3> sub = {
      {9.1f, 9.2f, 9.3f}
  };
  Matrix<float, 2, 3> expected = {
      {9.1f, 9.2f, 9.3f}, // Row 0 replaced
      {4.0f, 5.0f, 6.0f}
  };
  A.setSubmatrix<1, 3>(0, 0, sub);  // Set row 0
  _assertRoughlyEqual(A, expected);
}

// --- Bounds Checking Tests (Float) ---

TEST(MatrixSetSubmatrixFloatTests, BoundsCheckRowAndCol)
{
  Matrix<float, 3, 3> A;  // Target
  const Matrix<float, 2, 2> sub = {
      {1.f, 1.f},
      {1.f, 1.f}
  };  // Submatrix
  // Attempt to set 2x2 starting at (2, 2) - needs rows 2,3 and cols 2,3
  EXPECT_THROW((A.setSubmatrix<2, 2>(2, 2, sub)), std::out_of_range)
      << "setSubmatrix dimensions exceed bounds starting at (2, 2)";
}

TEST(MatrixSetSubmatrixFloatTests, BoundsCheckStartOutOfBounds)
{
  Matrix<float, 3, 3> A;
  const Matrix<float, 1, 1> sub = {{1.f}};
  // Attempt to start placement outside the matrix bounds
  EXPECT_THROW((A.setSubmatrix<1, 1>(3, 0, sub)), std::out_of_range) << "setSubmatrix starting row is out of bounds";
  EXPECT_THROW((A.setSubmatrix<1, 1>(0, 3, sub)), std::out_of_range) << "setSubmatrix starting col is out of bounds";
}