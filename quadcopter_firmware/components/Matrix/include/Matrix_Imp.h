#pragma once

#include <stdexcept>  // <--- ADD THIS LINE
#include <type_traits>
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
Matrix<T, ROWS, COLS>::Matrix(const T& value)
{
  for (size_t i = 0; i < ROWS * COLS; ++i) {
    data[i] = value;
  }
}

template <typename T, size_t ROWS, size_t COLS>
std::string Matrix<T, ROWS, COLS>::description(void) const
{
  std::ostringstream oss;

  oss << "[\n";  // Opening bracket for the matrix

  for (size_t r = 0; r < ROWS; ++r) {
    // indentation
    for (size_t i = 0; i < r; ++i) {
      oss << "  ";  // Two spaces for indentation
    }
    oss << "[";  // Opening bracket for the row
    for (size_t c = 0; c < COLS; ++c) {
      // Apply formatting if T is a floating-point type
      // Conditionally compile this formatting part
      if constexpr (std::is_floating_point_v<T>) {
        oss << std::fixed << std::setprecision(4);
      }

      // Access element using the const version of operator()
      oss << (*this)(r, c);

      // Add separator if not the last element in the row
      if (c < COLS - 1) {
        oss << ", ";
      }
    }
    oss << "]";  // Closing bracket for the row

    // Add newline if not the last row
    if (r < ROWS - 1) {
      oss << ",\n";
    } else {
      oss << "\n";  // Newline for the last row
    }
  }
  oss << "]";  // Closing bracket for the matrix
  return oss.str();
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
Matrix<T, COLS, ROWS> Matrix<T, ROWS, COLS>::transpose(void) const
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
T Matrix<T, ROWS, COLS>::dot(const Matrix<T, ROWS, COLS>& rhs) const
{
  T sum = T{};
  for (size_t i = 0; i < ROWS * COLS; i++) {
    sum += this->data[i] * rhs.data[i];
  }
  return sum;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::invert(void) const
{
  static_assert(ROWS == COLS, "Matrix inversion requires a square matrix.");
  static_assert(
      std::is_floating_point_v<T>,
      "Matrix inversion is numerically stable primarily for floating-point types (float, double) due to required "
      "divisions. Integer inversion may produce truncated/incorrect results.");

  const size_t N = ROWS;

  Matrix<T, N, N> tempA = *this;
  Matrix<T, N, N> inv = Matrix<T, N, N>::identity();

  // Perform Gauss-Jordan elimination with partial pivoting
  T tolerance = static_cast<T>(1e-8);
  if constexpr (std::is_same_v<T, float>) {
    tolerance = 1e-6f;  // Use a slightly looser tolerance for single precision
  }

  for (size_t j = 0; j < N; ++j) {
    // Iterate through columns (and pivot rows)
    size_t pivot_row = j;
    T max_val = std::abs(tempA(j, j));

    for (size_t k = j + 1; k < N; ++k) {
      T current_abs = std::abs(tempA(k, j));
      if (current_abs > max_val) {
        max_val = current_abs;
        pivot_row = k;
      }
    }

    // Check for singularity: If the largest element in the column is close to zero,
    // the matrix is singular (or numerically close to singular).
    if (max_val < tolerance) {
      throw std::runtime_error("Matrix is singular or nearly singular; cannot invert.");
    }

    if (pivot_row != j) {
      for (size_t k = 0; k < N; ++k) {
        std::swap(tempA(j, k), tempA(pivot_row, k));
        std::swap(inv(j, k), inv(pivot_row, k));
      }
    }

    // Normalization
    T pivot_val = tempA(j, j);
    for (size_t k = 0; k < N; ++k) {
      if (k >= j) {
        tempA(j, k) /= pivot_val;
      }
      inv(j, k) /= pivot_val;
    }

    // Elimination
    for (size_t i = 0; i < N; ++i) {
      if (i != j) {
        T factor = tempA(i, j);

        for (size_t k = 0; k < N; ++k) {
          if (k >= j) {
            tempA(i, k) -= factor * tempA(j, k);
          }
          inv(i, k) -= factor * inv(j, k);
        }
      }
    }
  }

  return inv;
}

template <typename T, size_t ROWS, size_t COLS>
template <size_t SUBROWS, size_t SUBCOLS>
Matrix<T, SUBROWS, SUBCOLS> Matrix<T, ROWS, COLS>::submatrix(size_t row, size_t col) const
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
      subMatrix(i, j) = data[(row + i) * COLS + (col + j)];
    }
  }
  return subMatrix;
}

template <typename T, size_t ROWS, size_t COLS>
template <size_t SUBROWS, size_t SUBCOLS>
void Matrix<T, ROWS, COLS>::setSubmatrix(size_t row, size_t col, Matrix<T, SUBROWS, SUBCOLS> sub)
{
  static_assert(SUBROWS <= ROWS, "Submatrix rows exceed original matrix rows");
  static_assert(SUBCOLS <= COLS, "Submatrix columns exceed original matrix columns");
  static_assert(SUBROWS > 0, "Submatrix rows must be greater than zero");
  static_assert(SUBCOLS > 0, "Submatrix columns must be greater than zero");
  if (row + SUBROWS > ROWS || col + SUBCOLS > COLS) {
    throw std::out_of_range("Submatrix exceeds original matrix bounds");
  }
  for (size_t i = 0; i < SUBROWS; i++) {
    for (size_t j = 0; j < SUBCOLS; j++) {
      data[(row + i) * COLS + (col + j)] = sub(i, j);
    }
  }
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::identity(void)
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

template <typename T, size_t ROWS, size_t COLS>
std::ostream& operator<<(std::ostream& os, const Matrix<T, ROWS, COLS>& matrix)
{
  // Delegate to your existing description method
  os << matrix.description();
  return os;
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

// Matrix + Scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator+(const Matrix<T, R, C>& A, const T& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = A.data[i] + B;
  }
  return result;
}

// Scalar + Matrix
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator+(const T& A, const Matrix<T, R, C>& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = B.data[i] + A;
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

// Matrix - Scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator-(const Matrix<T, R, C>& A, const T& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = A.data[i] - B;
  }
  return result;
}

// Scalar + Matrix
template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator-(const T& A, const Matrix<T, R, C>& B)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = A - B.data[i];
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

template <typename T, size_t R, size_t C>
Matrix<T, R, C> operator*(const Matrix<T, R, C>& A, T scalar)
{
  return scalar * A;
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
Matrix<T, R, C> operator%(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  Matrix<T, R, C> result;
  for (size_t i = 0; i < R * C; i++) {
    result.data[i] = lhs.data[i] * rhs.data[i];
  }
  return result;
}

template <typename T, size_t R, size_t C, size_t K>
Matrix<T, R, C>& operator*=(Matrix<T, R, C>& lhs, const Matrix<T, C, K>& rhs)
{
  lhs = lhs * rhs;
  return lhs;
}

template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator/=(Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  lhs = lhs / rhs;
  return lhs;
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

// matrix += scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator*=(Matrix<T, R, C>& lhs, const T& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] *= rhs;
  return lhs;
}

// matrix -= scalar
template <typename T, size_t R, size_t C>
Matrix<T, R, C>& operator/=(Matrix<T, R, C>& lhs, const T& rhs)
{
  for (size_t i = 0; i < R * C; i++) lhs.data[i] /= rhs;
  return lhs;
}

template <typename T, size_t R, size_t C>
bool operator==(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  for (size_t i = 0; i < R * C; i++) {
    if (lhs.data[i] != rhs.data[i]) return false;
  }
  return true;
}

template <typename T, size_t R, size_t C>
bool operator!=(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs)
{
  return !(lhs == rhs);
}