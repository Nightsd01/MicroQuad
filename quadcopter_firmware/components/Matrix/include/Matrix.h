#pragma once

#include <cmath>    // for std::fabs, std::sqrt
#include <cstring>  // for std::memset

/**
 * @brief A simple templated matrix class
 *
 * Template parameters:
 *   T: any numeric type
 *   ROWS: number of rows
 *   COLS: number of columns
 *
 * Storage is row-major: data[row*COLS + col]
 */
template <typename T, size_t ROWS, size_t COLS>
class Matrix
{
 public:
  T data[ROWS * COLS];

  // Default constructor (zero-initialized)
  Matrix(void);

  // Constructor from nested initializer_list
  Matrix(std::initializer_list<std::initializer_list<T>> init);

  // Access element (row, col)
  T& operator()(size_t row, size_t col);

  // Const version
  const T& operator()(size_t row, size_t col) const;

  // Static method to create an Identity matrix (only if square)
  static Matrix<T, ROWS, COLS> identity(void);
};

// -----------------------------------------------------------------------------
// Operator Overloads
// -----------------------------------------------------------------------------

// Matrix + Matrix
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator+(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B);

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator-(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B);

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator*(T scalar, const Matrix<T, R, C>& A);

template <typename T, size_t R, size_t C, size_t K>
Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Matrix<T, C, K>& B);

// matrix += matrix
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

// matrix -= matrix
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

// matrix += scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const T& rhs);

// matrix -= scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const T& rhs);

// Element-wise multiplication (matrix)
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator%(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

// Dot product (scalar result)
template <typename T, size_t R, size_t C>
T dot(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

template <typename T, size_t R, size_t C>
Matrix<T, C, R> transpose(const Matrix<T, R, C>& A);

template <typename T, size_t N>
Matrix<T, N, N> invert(const Matrix<T, N, N>& A);

#include <Matrix_Imp.h>