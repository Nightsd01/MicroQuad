#pragma once

#include <utility>

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>::Matrix(void)
{
  std::memset(data, 0, sizeof(data));
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>::Matrix(std::initializer_list<std::initializer_list<T>> init)
{
  // Check that the number of rows in 'init' matches ROWS
  if (init.size() != ROWS) throw std::runtime_error("Incorrect number of row initializers");

  // Copy the data
  size_t i = 0;
  for (auto& rowList : init) {
    if (rowList.size() != COLS) throw std::runtime_error("Incorrect number of column initializers");

    size_t j = 0;
    for (auto& val : rowList) {
      data[i * COLS + j] = val;
      j++;
    }
    i++;
  }
}

template <typename T, size_t ROWS, size_t COLS>
T& Matrix<T, ROWS, COLS>::operator()(size_t row, size_t col)
{
  return data[row * COLS + col];
}

template <typename T, size_t ROWS, size_t COLS>
const T& Matrix<T, ROWS, COLS>::operator()(size_t row, size_t col) const
{
  return data[row * COLS + col];
}

template <typename T, size_t ROWS, size_t COLS>
T Matrix<T, ROWS, COLS>::norm() const
{                       // Mark function as const as it doesn't modify the matrix
  double sum_sq = 0.0;  // Use double for accumulation

  // Iterate through all elements in the flat data array
  for (size_t i = 0; i < ROWS * COLS; ++i) {
    // Cast the element to double for the sum-of-squares calculation
    double element_d = static_cast<double>(data[i]);
    sum_sq += element_d * element_d;
  }

  // Calculate the square root (which returns a double)
  double norm_d = std::sqrt(sum_sq);

  // Cast the final result back to the requested type T
  return static_cast<T>(norm_d);
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, COLS, ROWS> Matrix<T, ROWS, COLS>::transpose(void)
{
  Matrix<T, COLS, ROWS> result;
  for (size_t i = 0; i < ROWS; i++) {
    for (size_t j = 0; j < COLS; j++) {
      result(j, i) = data[i * COLS + j];
    }
  }
  return result;
}

template <typename T, size_t ROWS, size_t COLS>
T Matrix<T, ROWS, COLS>::dot(const Matrix<T, ROWS, COLS>& rhs)
{
  T sum = T{};
  for (size_t i = 0; i < ROWS * COLS; i++) {
    sum += this.data[i] * rhs.data[i];
  }
  return sum;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::invert(void) const
{
  static_assert(ROWS == COLS, "Matrix inversion is only defined for square matrices.");
  Matrix<T, ROWS, COLS> M = this;
  Matrix<T, ROWS, COLS> Inv = Matrix<T, ROWS, COLS>::identity();

  for (size_t col = 0; col < ROWS; col++) {
    // Find pivot row
    T pivotVal = std::abs(M(col, col));  // <--- changed
    size_t pivotRow = col;
    for (size_t r = col + 1; r < ROWS; r++) {
      T candidate = std::abs(M(r, col));  // <--- changed
      if (candidate > pivotVal) {
        pivotVal = candidate;
        pivotRow = r;
      }
    }

    // Check near singular
    // If T is float, use 1e-6; if double, 1e-12; etc.
    static constexpr T EPSILON = std::numeric_limits<T>::epsilon() * 1000.0;
    if (pivotVal < EPSILON) {
      return Matrix<T, ROWS, ROWS>();  // zero matrix signals singular
    }

    // Swap pivot row
    if (pivotRow != col) {
      for (size_t c = 0; c < ROWS; c++) {
        std::swap(M(col, c), M(pivotRow, c));
        std::swap(Inv(col, c), Inv(pivotRow, c));
      }
    }

    // Normalize pivot row
    T diag = M(col, col);
    for (size_t c = 0; c < ROWS; c++) {
      M(col, c) /= diag;
      Inv(col, c) /= diag;
    }

    // Eliminate
    for (size_t r = 0; r < ROWS; r++) {
      if (r != col) {
        T factor = M(r, col);
        for (size_t c = 0; c < ROWS; c++) {
          M(r, c) -= factor * M(col, c);
          Inv(r, c) -= factor * Inv(col, c);
        }
      }
    }
  }
  return Inv;
}

template <typename T, size_t ROWS, size_t COLS>
template <size_t SUBROWS, size_t SUBCOLS>
Matrix<T, SUBROWS, SUBCOLS> Matrix<T, ROWS, COLS>::submatrix(size_t row, size_t col)
{
  static_assert(SUBROWS <= ROWS, "Submatrix rows exceed original matrix rows");
  static_assert(SUBCOLS <= COLS, "Submatrix columns exceed original matrix columns");
  static_assert(SUBROWS > 0, "Submatrix rows must be greater than zero");
  static_assert(SUBCOLS > 0, "Submatrix columns must be greater than zero");
  Matrix<T, SUBROWS, SUBCOLS> subMatrix;
  if (row + SUBROWS > ROWS || col + SUBCOLS > COLS) {
    throw std::out_of_range("Submatrix exceeds original matrix bounds");
  }
  for (size_t i = 0; i < SUBROWS; i++) {
    for (size_t j = 0; j < SUBCOLS; j++) {
      subMatrix(i, j) = this(row + i, col + j);
    }
  }
  return subMatrix;
}

template <typename T, size_t ROWS, size_t COLS>
static Matrix<T, ROWS, COLS> identity(void)
{
  static_assert(ROWS == COLS, "Identity only valid for square matrix");
  Matrix<T, ROWS, COLS> I;
  for (size_t i = 0; i < ROWS; i++) {
    for (size_t j = 0; j < COLS; j++) {
      I(i, j) = (i == j) ? 1.0f : 0.0f;
    }
  }
  return I;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator+(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = A.data[i] + B.data[i];
  }
  return result;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator-(const Matrix<T, R, C>& A, const Matrix<T, R, C>& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = A.data[i] - B.data[i];
  }
  return result;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator*(T scalar, const Matrix<T, R, C>& A)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = scalar * A.data[i];
  }
  return result;
}

template <typename T, size_t R, size_t C, size_t K>
Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Matrix<T, C, K>& B)
{
  Matrix<T, R, K> result;
  for (size_t i = 0; i < R; i++) {
    for (size_t j = 0; j < K; j++) {
      T sum = 0.0f;
      for (size_t m = 0; m < C; m++) {
        sum += A(i, m) * B(m, j);
      }
      result(i, j) = sum;
    }
  }
  return result;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] += rhs.data[i];
  return lhs;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] -= rhs.data[i];
  return lhs;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator+=(Matrix<T, R, C>& lhs, const T& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] += rhs;
  return lhs;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator-=(Matrix<T, R, C>& lhs, const T& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] -= rhs;
  return lhs;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator%(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = lhs.data[i] * rhs.data[i];
  }
  return result;
}
