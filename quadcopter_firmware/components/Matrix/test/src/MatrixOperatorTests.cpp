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
