#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>
#include <cmath>
#include <algorithm>

#include "Matrix_AccelerationImp.h"

// Helper to generate index sequence for N-D operations
template<size_t... Is>
struct index_sequence {};

template<size_t N, size_t... Is>
struct make_index_sequence : make_index_sequence<N-1, N-1, Is...> {};

template<size_t... Is>
struct make_index_sequence<0, Is...> {
    using type = index_sequence<Is...>;
};

// Description method
template <Numeric T, size_t... Dims>
std::string Matrix<T, Dims...>::description() const
{
    std::ostringstream oss;
    
    if constexpr (ndims == 1) {
        // Vector case
        oss << "[";
        for (size_t i = 0; i < total_size; ++i) {
            if constexpr (std::is_floating_point_v<T>) {
                oss << std::fixed << std::setprecision(4);
            }
            oss << data[i];
            if (i < total_size - 1) oss << ", ";
        }
        oss << "]";
    } else if constexpr (ndims == 2) {
        // 2D matrix case - maintain backward compatibility
        constexpr size_t rows = dimensions[0];
        constexpr size_t cols = dimensions[1];
        
        oss << "[\n";
        for (size_t r = 0; r < rows; ++r) {
            // indentation
            for (size_t i = 0; i < r; ++i) {
                oss << "  ";
            }
            oss << "[";
            for (size_t c = 0; c < cols; ++c) {
                if constexpr (std::is_floating_point_v<T>) {
                    oss << std::fixed << std::setprecision(4);
                }
                oss << (*this)(r, c);
                if (c < cols - 1) {
                    oss << ", ";
                }
            }
            oss << "]";
            if (r < rows - 1) {
                oss << ",\n";
            } else {
                oss << "\n";
            }
        }
        oss << "]";
    } else {
        // N-D tensor case - avoid using typeid for RTTI-disabled builds
        oss << "Tensor<";
        if constexpr (std::is_same_v<T, float>) {
            oss << "float";
        } else if constexpr (std::is_same_v<T, double>) {
            oss << "double";
        } else if constexpr (std::is_same_v<T, int>) {
            oss << "int";
        } else if constexpr (std::is_same_v<T, int16_t>) {
            oss << "int16_t";
        } else if constexpr (std::is_same_v<T, int32_t>) {
            oss << "int32_t";
        } else if constexpr (std::is_same_v<T, uint8_t>) {
            oss << "uint8_t";
        } else if constexpr (std::is_same_v<T, uint16_t>) {
            oss << "uint16_t";
        } else if constexpr (std::is_same_v<T, uint32_t>) {
            oss << "uint32_t";
        } else {
            oss << "numeric";
        }
        oss << ", ";
        for (size_t i = 0; i < ndims; ++i) {
            oss << dimensions[i];
            if (i < ndims - 1) oss << ", ";
        }
        oss << "> with " << total_size << " elements";
    }
    
    return oss.str();
}

// Norm method
template <Numeric T, size_t... Dims>
T Matrix<T, Dims...>::norm() const
{
    double sum_sq = 0.0;
    
    for (size_t i = 0; i < total_size; ++i) {
        double element_d = static_cast<double>(data[i]);
        sum_sq += element_d * element_d;
    }
    
    double norm_d = std::sqrt(sum_sq);
    return static_cast<T>(norm_d);
}

// Transpose for 2D matrices
template <Numeric T, size_t... Dims>
auto Matrix<T, Dims...>::transpose() const requires (ndims == 2)
{
    constexpr size_t rows = dimensions[0];
    constexpr size_t cols = dimensions[1];
    
    Matrix<T, cols, rows> result;
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

// Dot product
template <Numeric T, size_t... Dims>
T Matrix<T, Dims...>::dot(const Matrix<T, Dims...>& rhs) const
{
    T sum = T{};
    for (size_t i = 0; i < total_size; ++i) {
        sum += this->data[i] * rhs.data[i];
    }
    return sum;
}

// Matrix inversion for square 2D matrices
template <Numeric T, size_t... Dims>
Matrix<T, Dims...> Matrix<T, Dims...>::invert() const requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value)
{
    static_assert(FloatingPoint<T>, 
        "Matrix inversion is numerically stable primarily for floating-point types");
    
    constexpr size_t N = dimensions[0];
    
    Matrix<T, N, N> tempA = *this;
    Matrix<T, N, N> inv = Matrix<T, N, N>::identity();
    
    T tolerance = static_cast<T>(1e-8);
    if constexpr (std::is_same_v<T, float>) {
        tolerance = 1e-6f;
    }
    
    for (size_t j = 0; j < N; ++j) {
        size_t pivot_row = j;
        T max_val = std::abs(tempA(j, j));
        
        for (size_t k = j + 1; k < N; ++k) {
            T current_abs = std::abs(tempA(k, j));
            if (current_abs > max_val) {
                max_val = current_abs;
                pivot_row = k;
            }
        }
        
        if (max_val < tolerance) {
            throw std::runtime_error("Matrix is singular or nearly singular; cannot invert.");
        }
        
        if (pivot_row != j) {
            for (size_t k = 0; k < N; ++k) {
                std::swap(tempA(j, k), tempA(pivot_row, k));
                std::swap(inv(j, k), inv(pivot_row, k));
            }
        }
        
        T pivot_val = tempA(j, j);
        for (size_t k = 0; k < N; ++k) {
            if (k >= j) {
                tempA(j, k) /= pivot_val;
            }
            inv(j, k) /= pivot_val;
        }
        
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

// Identity matrix for square 2D matrices
template <Numeric T, size_t... Dims>
Matrix<T, Dims...> Matrix<T, Dims...>::identity() requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value)
{
    constexpr size_t N = dimensions[0];
    Matrix<T, N, N> I;
    
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            I(i, j) = (i == j) ? T{1} : T{0};
        }
    }
    return I;
}

// Slicing for 2D matrices (backward compatibility)
template <Numeric T, size_t... Dims>
template <size_t SubRows, size_t SubCols>
auto Matrix<T, Dims...>::slice(size_t startRow, size_t startCol) const -> Matrix<T, SubRows, SubCols> requires (ndims == 2)
{
    if (startRow + SubRows > dimensions[0] || startCol + SubCols > dimensions[1]) {
        throw std::out_of_range("Slice exceeds original matrix bounds");
    }
    
    Matrix<T, SubRows, SubCols> result;
    
    for (size_t i = 0; i < SubRows; ++i) {
        for (size_t j = 0; j < SubCols; ++j) {
            result(i, j) = (*this)(startRow + i, startCol + j);
        }
    }
    
    return result;
}

// Set slice for 2D matrices (backward compatibility)
template <Numeric T, size_t... Dims>
template <size_t SubRows, size_t SubCols>
void Matrix<T, Dims...>::setSlice(size_t row, size_t col, const Matrix<T, SubRows, SubCols>& sub) requires (ndims == 2)
{
    if (row + SubRows > dimensions[0] || col + SubCols > dimensions[1]) {
        throw std::out_of_range("Slice exceeds original matrix bounds");
    }
    
    for (size_t i = 0; i < SubRows; ++i) {
        for (size_t j = 0; j < SubCols; ++j) {
            (*this)(row + i, col + j) = sub(i, j);
        }
    }
}

// Determinant for square 2D matrices
template <Numeric T, size_t... Dims>
T Matrix<T, Dims...>::determinant() const requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value)
{
    constexpr size_t N = dimensions[0];
    
    if constexpr (N == 1) {
        return (*this)(0, 0);
    } else if constexpr (N == 2) {
        return ((*this)(0, 0) * (*this)(1, 1)) - ((*this)(0, 1) * (*this)(1, 0));
    } else {
        T det = T{0};
        T sign = T{1};
        
        for (size_t j = 0; j < N; ++j) {
            Matrix<T, N-1, N-1> minor_matrix = _createSubmatrix(0, j);
            T minor_determinant = minor_matrix.determinant();
            det += sign * (*this)(0, j) * minor_determinant;
            sign = -sign;
        }
        return det;
    }
}

// Helper for determinant
template <Numeric T, size_t... Dims>
auto Matrix<T, Dims...>::_createSubmatrix(size_t skip_row, size_t skip_col) const requires (ndims == 2)
{
    constexpr size_t N = dimensions[0];
    Matrix<T, N-1, N-1> sub;
    size_t sub_r = 0;
    
    for (size_t r = 0; r < N; ++r) {
        if (r == skip_row) continue;
        
        size_t sub_c = 0;
        for (size_t c = 0; c < N; ++c) {
            if (c == skip_col) continue;
            sub(sub_r, sub_c) = (*this)(r, c);
            sub_c++;
        }
        sub_r++;
    }
    return sub;
}

// Operator<< implementation
template <Numeric T, size_t... Dims>
std::ostream& operator<<(std::ostream& os, const Matrix<T, Dims...>& matrix)
{
    os << matrix.description();
    return os;
}

// Element-wise operations - now return expression templates
template <Numeric T, size_t... Dims>
auto operator+(const Matrix<T, Dims...>& A, const Matrix<T, Dims...>& B) -> AddExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>
{
    return AddExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), MatrixExpr<T, Dims...>(B));
}

template <Numeric T, size_t... Dims>
auto operator+(const Matrix<T, Dims...>& A, const T& B) -> ScalarAddExpr<MatrixExpr<T, Dims...>>
{
    return ScalarAddExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), B);
}

template <Numeric T, size_t... Dims>
auto operator+(const T& A, const Matrix<T, Dims...>& B) -> ScalarAddExpr<MatrixExpr<T, Dims...>>
{
    return ScalarAddExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(B), A);
}

template <Numeric T, size_t... Dims>
auto operator-(const Matrix<T, Dims...>& A, const Matrix<T, Dims...>& B) -> SubExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>
{
    return SubExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), MatrixExpr<T, Dims...>(B));
}

template <Numeric T, size_t... Dims>
auto operator-(const Matrix<T, Dims...>& A) -> NegExpr<MatrixExpr<T, Dims...>>
{
    return NegExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A));
}

template <Numeric T, size_t... Dims>
auto operator-(const Matrix<T, Dims...>& A, const T& B) -> ScalarSubExpr<MatrixExpr<T, Dims...>>
{
    return ScalarSubExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), B);
}

template <Numeric T, size_t... Dims>
auto operator-(const T& A, const Matrix<T, Dims...>& B) -> ReverseScalarSubExpr<MatrixExpr<T, Dims...>>
{
    return ReverseScalarSubExpr<MatrixExpr<T, Dims...>>(A, MatrixExpr<T, Dims...>(B));
}

// Scalar multiplication
template <Numeric T, size_t... Dims>
auto operator*(T scalar, const Matrix<T, Dims...>& A) -> ScalarMulExpr<MatrixExpr<T, Dims...>>
{
    return ScalarMulExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), scalar);
}

template <Numeric T, size_t... Dims>
auto operator*(const Matrix<T, Dims...>& A, T scalar) -> ScalarMulExpr<MatrixExpr<T, Dims...>>
{
    return ScalarMulExpr<MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(A), scalar);
}

// Matrix multiplication for 2D matrices
template <Numeric T, size_t R, size_t C, size_t K>
Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Matrix<T, C, K>& B)
{
    Matrix<T, R, K> result;
    
#ifdef USE_ACCELERATION
    if constexpr (std::is_same_v<T, float> || std::is_same_v<T, int16_t>) {
        performMultiplication<T, R, C, K>(A.data, B.data, result.data);
        return result;
    }
#endif
    
    for (size_t i = 0; i < R; ++i) {
        for (size_t j = 0; j < K; ++j) {
            T sum = T{0};
            for (size_t m = 0; m < C; ++m) {
                sum += A(i, m) * B(m, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

// Element-wise multiplication
template <Numeric T, size_t... Dims>
auto operator%(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs) -> MulExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>
{
    return MulExpr<MatrixExpr<T, Dims...>, MatrixExpr<T, Dims...>>(MatrixExpr<T, Dims...>(lhs), MatrixExpr<T, Dims...>(rhs));
}

// Compound assignment operators
template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator+=(Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] += rhs.data[i];
    }
    return lhs;
}

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator-=(Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] -= rhs.data[i];
    }
    return lhs;
}

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator+=(Matrix<T, Dims...>& lhs, const T& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] += rhs;
    }
    return lhs;
}

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator-=(Matrix<T, Dims...>& lhs, const T& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] -= rhs;
    }
    return lhs;
}

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator*=(Matrix<T, Dims...>& lhs, const T& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] *= rhs;
    }
    return lhs;
}

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator/=(Matrix<T, Dims...>& lhs, const T& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        lhs.data[i] /= rhs;
    }
    return lhs;
}

// Comparison operators
template <Numeric T, size_t... Dims>
bool operator==(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs)
{
    for (size_t i = 0; i < Matrix<T, Dims...>::total_size; ++i) {
        if (lhs.data[i] != rhs.data[i]) return false;
    }
    return true;
}

template <Numeric T, size_t... Dims>
bool operator!=(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs)
{
    return !(lhs == rhs);
}

// Expression template operators for chaining
template<typename LHS, typename RHS>
auto operator+(const LHS& lhs, const RHS& rhs) -> AddExpr<LHS, RHS>
requires requires {
    typename LHS::value_type;
    typename RHS::value_type;
    { LHS::total_size } -> std::convertible_to<size_t>;
    { RHS::total_size } -> std::convertible_to<size_t>;
    { lhs.eval(size_t{}) } -> std::convertible_to<typename LHS::value_type>;
    { rhs.eval(size_t{}) } -> std::convertible_to<typename RHS::value_type>;
}
{
    return AddExpr<LHS, RHS>(lhs, rhs);
}

template<typename LHS, typename RHS>
auto operator-(const LHS& lhs, const RHS& rhs) -> SubExpr<LHS, RHS>
requires requires {
    typename LHS::value_type;
    typename RHS::value_type;
    { LHS::total_size } -> std::convertible_to<size_t>;
    { RHS::total_size } -> std::convertible_to<size_t>;
    { lhs.eval(size_t{}) } -> std::convertible_to<typename LHS::value_type>;
    { rhs.eval(size_t{}) } -> std::convertible_to<typename RHS::value_type>;
}
{
    return SubExpr<LHS, RHS>(lhs, rhs);
}

template<typename LHS, typename RHS>
auto operator%(const LHS& lhs, const RHS& rhs) -> MulExpr<LHS, RHS>
requires requires {
    typename LHS::value_type;
    typename RHS::value_type;
    { LHS::total_size } -> std::convertible_to<size_t>;
    { RHS::total_size } -> std::convertible_to<size_t>;
    { lhs.eval(size_t{}) } -> std::convertible_to<typename LHS::value_type>;
    { rhs.eval(size_t{}) } -> std::convertible_to<typename RHS::value_type>;
}
{
    return MulExpr<LHS, RHS>(lhs, rhs);
}

template<typename Operand>
auto operator-(const Operand& operand) -> NegExpr<Operand>
requires requires {
    typename Operand::value_type;
    { Operand::total_size } -> std::convertible_to<size_t>;
    { operand.eval(size_t{}) } -> std::convertible_to<typename Operand::value_type>;
}
{
    return NegExpr<Operand>(operand);
}

// Mixed Matrix + Expression operators
template <Numeric T, size_t... Dims, typename Expr>
auto operator+(const Matrix<T, Dims...>& A, const Expr& B) -> AddExpr<MatrixExpr<T, Dims...>, Expr>
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == Matrix<T, Dims...>::total_size) && std::is_same_v<typename Expr::value_type, T>
{
    return AddExpr<MatrixExpr<T, Dims...>, Expr>(MatrixExpr<T, Dims...>(A), B);
}

template <Numeric T, size_t... Dims, typename Expr>
auto operator+(const Expr& A, const Matrix<T, Dims...>& B) -> AddExpr<Expr, MatrixExpr<T, Dims...>>
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { A.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == Matrix<T, Dims...>::total_size) && std::is_same_v<typename Expr::value_type, T>
{
    return AddExpr<Expr, MatrixExpr<T, Dims...>>(A, MatrixExpr<T, Dims...>(B));
}

// Mixed Matrix - Expression operators
template <Numeric T, size_t... Dims, typename Expr>
auto operator-(const Matrix<T, Dims...>& A, const Expr& B) -> SubExpr<MatrixExpr<T, Dims...>, Expr>
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == Matrix<T, Dims...>::total_size) && std::is_same_v<typename Expr::value_type, T>
{
    return SubExpr<MatrixExpr<T, Dims...>, Expr>(MatrixExpr<T, Dims...>(A), B);
}

template <Numeric T, size_t... Dims, typename Expr>
auto operator-(const Expr& A, const Matrix<T, Dims...>& B) -> SubExpr<Expr, MatrixExpr<T, Dims...>>
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { A.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == Matrix<T, Dims...>::total_size) && std::is_same_v<typename Expr::value_type, T>
{
    return SubExpr<Expr, MatrixExpr<T, Dims...>>(A, MatrixExpr<T, Dims...>(B));
}

// Matrix multiplication with expressions - convert expression to Matrix first
template <Numeric T, size_t R, size_t C, size_t K, typename Expr>
Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Expr& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == C * K) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, C, K> B_matrix;
    for (size_t i = 0; i < C * K; ++i) {
        B_matrix.data[i] = B.eval(i);
    }
    return A * B_matrix;
}

template <Numeric T, size_t R, size_t C, size_t K, typename Expr>
Matrix<T, R, K> operator*(const Expr& A, const Matrix<T, C, K>& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { A.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == R * C) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, R, C> A_matrix;
    for (size_t i = 0; i < R * C; ++i) {
        A_matrix.data[i] = A.eval(i);
    }
    return A_matrix * B;
}

// Specific overloads for common matrix-vector multiplications
template <Numeric T, typename Expr>
Matrix<T, 3, 1> operator*(const Matrix<T, 3, 3>& A, const Expr& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == 3) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, 3, 1> B_matrix;
    for (size_t i = 0; i < 3; ++i) {
        B_matrix.data[i] = B.eval(i);
    }
    return A * B_matrix;
}

template <Numeric T, typename Expr>
Matrix<T, 4, 1> operator*(const Matrix<T, 4, 4>& A, const Expr& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == 4) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, 4, 1> B_matrix;
    for (size_t i = 0; i < 4; ++i) {
        B_matrix.data[i] = B.eval(i);
    }
    return A * B_matrix;
}

template <Numeric T, typename Expr>
Matrix<T, 10, 1> operator*(const Matrix<T, 10, 10>& A, const Expr& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { B.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == 10) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, 10, 1> B_matrix;
    for (size_t i = 0; i < 10; ++i) {
        B_matrix.data[i] = B.eval(i);
    }
    return A * B_matrix;
}

// Expression * Matrix multiplication for common cases
template <Numeric T, typename Expr>
Matrix<T, 10, 10> operator*(const Expr& A, const Matrix<T, 1, 10>& B)
requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { A.eval(size_t{}) } -> std::convertible_to<T>;
} && (Expr::total_size == 10) && std::is_same_v<typename Expr::value_type, T>
{
    // Convert expression to Matrix first, then multiply
    Matrix<T, 10, 1> A_matrix;
    for (size_t i = 0; i < 10; ++i) {
        A_matrix.data[i] = A.eval(i);
    }
    return A_matrix * B;
}