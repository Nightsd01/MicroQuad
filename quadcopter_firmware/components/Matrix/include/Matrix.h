#pragma once

#include <cmath>    // for std::fabs, std::sqrt
#include <cstring>  // for std::memset
#include <initializer_list>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>

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

  // Constructor (initialize all elements to a specific value)
  Matrix(const T& value);

  // Initialize from C array (in row-major order)
  Matrix(T values[ROWS * COLS]);

  std::string description(void) const;

  // Access element (row, col)
  inline T& operator()(size_t row, size_t col) __attribute__((always_inline));

  // Const version
  inline const T& operator()(size_t row, size_t col) const __attribute__((always_inline));

  inline void zeros(void) __attribute__((always_inline)) { std::memset(data, 0, sizeof(data)); }

  inline T norm(void) const __attribute__((always_inline));

  inline Matrix<T, COLS, ROWS> transpose(void) const __attribute__((always_inline));

  inline T dot(const Matrix<T, ROWS, COLS>& rhs) const __attribute__((always_inline));

  // Method to invert the matrix (only valid for square matrices)
  inline Matrix<T, ROWS, COLS> invert(void) const __attribute__((always_inline));

  // Static method to create an Identity matrix (only if square)
  static Matrix<T, ROWS, COLS> identity(void);

  // Access sub-matrix copy (row, col)
  template <size_t SUBROWS, size_t SUBCOLS>
  inline Matrix<T, SUBROWS, SUBCOLS> slice(size_t row, size_t col) const __attribute__((always_inline));

  // Access sub-matrix copy (row, col)
  template <size_t SUBROWS, size_t SUBCOLS>
  inline void setSlice(size_t row, size_t col, Matrix<T, SUBROWS, SUBCOLS> sub) __attribute__((always_inline));

  inline T determinant(void) const __attribute__((always_inline));

 private:
  /**
   * @brief Creates a submatrix by removing a specified row and column.
   * @details Used internally for calculating the determinant (finding minors).
   * Assumes ROWS > 0 and COLS > 0. Template parameters ensure return type
   * matches the submatrix dimensions. Only sensible if ROWS > 1 and COLS > 1.
   * @param skip_row The index of the row to exclude.
   * @param skip_col The index of the column to exclude.
   * @return A Matrix<T, ROWS-1, COLS-1> which is the minor matrix.
   */
  inline Matrix<T, ROWS - 1, COLS - 1> _createSubmatrix(size_t skip_row, size_t skip_col) const
      __attribute__((always_inline));
};

// -----------------------------------------------------------------------------
// Operator Overloads
// -----------------------------------------------------------------------------

template <typename T, size_t R, size_t C>
inline std::ostream& operator<<(std::ostream& os, const Matrix<T, R, C>& matrix) __attribute__((always_inline));

// Matrix + Matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator+(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B) __attribute__((always_inline));

// Matrix + Scalar
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator+(const Matrix<T, R, C>& A, const T& B) __attribute__((always_inline));

// Scalar + Matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator+(const T& A, const Matrix<T, R, C>& B) __attribute__((always_inline));

// Matrix - Matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator-(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B) __attribute__((always_inline));

// Matrix negation
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator-(const Matrix<T, R, C>& A) __attribute__((always_inline));

// Matrix - Scalar
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator-(const Matrix<T, R, C>& A, const T& B) __attribute__((always_inline));

// Scalar - Matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator-(const T& A, const Matrix<T, R, C>& B) __attribute__((always_inline));

// Multiplication
// IMPORTANT NOTE: The following two functions are elementwise multiplication
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator*(T scalar, const Matrix<T, R, C>& A) __attribute__((always_inline));

template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator*(const Matrix<T, R, C>& A, T scalar) __attribute__((always_inline));

// IMPORTANT NOTE: The following function is matrix multiplication (not elementwise)
template <typename T, size_t R, size_t C, size_t K>
inline Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Matrix<T, C, K>& B) __attribute__((always_inline));

// Element-wise multiplication (matrix)
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C> operator%(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) __attribute__((always_inline));

// matrix += matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) __attribute__((always_inline));

// matrix -= matrix
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) __attribute__((always_inline));

// matrix += scalar
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const T& rhs) __attribute__((always_inline));

// matrix -= scalar
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const T& rhs) __attribute__((always_inline));

// matrix *= scalar - elementwise
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator*=(Matrix<T, R, C>& lhs, const T& rhs) __attribute__((always_inline));

// matrix /= scalar - elementwise
template <typename T, size_t R, size_t C>
inline Matrix<T, R, C>& operator/=(Matrix<T, R, C>& lhs, const T& rhs) __attribute__((always_inline));

// matrix equality
template <typename T, size_t R, size_t C>
bool operator==(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

template <typename T, size_t R, size_t C>
bool operator!=(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs);

#include <Matrix_Imp.h>