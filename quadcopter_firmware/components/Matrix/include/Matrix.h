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
  inline T& operator()(size_t row, size_t col) __attribute__((always_inline));

  // Const version
  inline const T& operator()(size_t row, size_t col) const __attribute__((always_inline));

  inline void zeros(void) __attribute__((always_inline)) { std::memset(data, 0, sizeof(data)); }

  inline T norm(void) const __attribute__((always_inline));

  inline Matrix<T, COLS, ROWS> transpose(void) __attribute__((always_inline));

  inline T dot(const Matrix<T, ROWS, COLS>& rhs) __attribute__((always_inline));

  // Method to invert the matrix (only valid for square matrices)
  inline Matrix<T, ROWS, COLS> invert(void) const __attribute__((always_inline));

  // Static method to create an Identity matrix (only if square)
  static inline Matrix<T, ROWS, COLS> identity(void) __attribute__((always_inline));

  // Access sub-matrix copy (row, col)
  template <size_t SUBROWS, size_t SUBCOLS>
  inline Matrix<T, SUBROWS, SUBCOLS> submatrix(size_t row, size_t col) __attribute__((always_inline));
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

#include <Matrix_Imp.h>