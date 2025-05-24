#pragma once

#include <algorithm>
#include <array>
#include <cmath>    // for std::fabs, std::sqrt
#include <cstring>  // for std::memset
#include <cstdint>  // for int16_t, int32_t, etc.
#include <concepts>
#include <initializer_list>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

// C++20 concepts for type constraints
template<typename T>
concept Numeric = std::is_arithmetic_v<T>;

template<typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

// Helper to calculate total size from dimensions
template<size_t... Dims>
constexpr size_t product() {
    return (Dims * ...);
}

// Helper to get the Nth element from a parameter pack
template<size_t N, size_t First, size_t... Rest>
struct nth_element {
    static constexpr size_t value = nth_element<N-1, Rest...>::value;
};

template<size_t First, size_t... Rest>
struct nth_element<0, First, Rest...> {
    static constexpr size_t value = First;
};

// Helper to count dimensions
template<size_t... Dims>
constexpr size_t dimension_count() {
    return sizeof...(Dims);
}

/**
 * @brief A templated N-dimensional matrix/tensor class
 *
 * Template parameters:
 *   T: any numeric type
 *   Dims...: dimensions of the matrix (e.g., Matrix<float, 3, 4, 5> is a 3x4x5 tensor)
 *
 * Storage is row-major for 2D, generalized to lexicographic order for N-D
 */
template <Numeric T, size_t... Dims>
class Matrix
{
    static_assert(sizeof...(Dims) > 0, "Matrix must have at least one dimension");
    static_assert(((Dims > 0) && ...), "All dimensions must be greater than zero");

public:
    static constexpr size_t ndims = sizeof...(Dims);
    static constexpr size_t total_size = product<Dims...>();
    static constexpr std::array<size_t, ndims> dimensions = {Dims...};
    
    T data[total_size];

    // Default constructor (zero-initialized)
    Matrix() {
        std::memset(data, 0, sizeof(data));
    }

    // Constructor (initialize all elements to a specific value)
    explicit Matrix(const T& value) {
        std::fill(std::begin(data), std::end(data), value);
    }

    // Initialize from C array (in row-major/lexicographic order)
    explicit Matrix(const T (&values)[total_size]) {
        std::copy(std::begin(values), std::end(values), std::begin(data));
    }

    // Initialize from pointer (for backward compatibility)
    explicit Matrix(const T* values) {
        std::copy(values, values + total_size, std::begin(data));
    }

    // For 2D matrices, support nested initializer_list (backward compatibility)
    Matrix(std::initializer_list<std::initializer_list<T>> init) requires (ndims == 2) {
        constexpr size_t rows = nth_element<0, Dims...>::value;
        constexpr size_t cols = nth_element<1, Dims...>::value;
        
        if (init.size() != rows) {
            throw std::runtime_error("Incorrect number of row initializers");
        }

        size_t i = 0;
        for (auto& rowList : init) {
            if (rowList.size() != cols) {
                throw std::runtime_error("Incorrect number of column initializers");
            }
            size_t j = 0;
            for (auto& val : rowList) {
                data[i * cols + j] = val;
                j++;
            }
            i++;
        }
    }

    // General element access using variadic indices
    template<typename... Indices>
    T& operator()(Indices... indices) requires (sizeof...(indices) == ndims) {
        return data[linear_index(indices...)];
    }

    template<typename... Indices>
    const T& operator()(Indices... indices) const requires (sizeof...(indices) == ndims) {
        return data[linear_index(indices...)];
    }

    // For backward compatibility with 2D matrices
    T& operator()(size_t row, size_t col) requires (ndims == 2) {
        constexpr size_t cols = nth_element<1, Dims...>::value;
        return data[row * cols + col];
    }

    const T& operator()(size_t row, size_t col) const requires (ndims == 2) {
        constexpr size_t cols = nth_element<1, Dims...>::value;
        return data[row * cols + col];
    }

    // Linear indexing (direct array access)
    T& operator[](size_t idx) {
        return data[idx];
    }

    const T& operator[](size_t idx) const {
        return data[idx];
    }

    void zeros() {
        std::memset(data, 0, sizeof(data));
    }

    T norm() const;
    
    std::string description() const;

    // Transpose for 2D matrices (backward compatibility)
    auto transpose() const requires (ndims == 2);

    // General axis swapping for N-D tensors
    template<size_t Axis1, size_t Axis2>
    auto swap_axes() const;

    // Dot product (element-wise multiplication and sum)
    T dot(const Matrix<T, Dims...>& rhs) const;

    // Matrix inversion for square 2D matrices
    Matrix<T, Dims...> invert() const requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value);

    // Identity matrix for square 2D matrices
    static Matrix<T, Dims...> identity() requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value);

    // Slicing operations - for 2D matrices only
    template<size_t SubRows, size_t SubCols>
    auto slice(size_t startRow, size_t startCol) const -> Matrix<T, SubRows, SubCols> requires (ndims == 2);

    template<size_t SubRows, size_t SubCols>
    void setSlice(size_t row, size_t col, const Matrix<T, SubRows, SubCols>& sub) requires (ndims == 2);

    // Determinant for square 2D matrices
    T determinant() const requires (ndims == 2 && nth_element<0, Dims...>::value == nth_element<1, Dims...>::value);

private:
    // Convert multi-dimensional indices to linear index
    template<typename... Indices>
    size_t linear_index(Indices... indices) const {
        size_t idx = 0;
        size_t stride = 1;
        size_t index_array[] = {static_cast<size_t>(indices)...};
        
        for (int i = ndims - 1; i >= 0; --i) {
            idx += index_array[i] * stride;
            stride *= dimensions[i];
        }
        return idx;
    }

    // Helper for determinant calculation
    auto _createSubmatrix(size_t skip_row, size_t skip_col) const requires (ndims == 2);
};

// Type aliases for common cases
template<Numeric T>
using Vector = Matrix<T, 1>;

template<Numeric T, size_t N>
using SquareMatrix = Matrix<T, N, N>;

// Operator overloads
template <Numeric T, size_t... Dims>
std::ostream& operator<<(std::ostream& os, const Matrix<T, Dims...>& matrix);

// Element-wise operations
template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator+(const Matrix<T, Dims...>& A, const Matrix<T, Dims...>& B);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator+(const Matrix<T, Dims...>& A, const T& B);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator+(const T& A, const Matrix<T, Dims...>& B);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator-(const Matrix<T, Dims...>& A, const Matrix<T, Dims...>& B);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator-(const Matrix<T, Dims...>& A);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator-(const Matrix<T, Dims...>& A, const T& B);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator-(const T& A, const Matrix<T, Dims...>& B);

// Scalar multiplication
template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator*(T scalar, const Matrix<T, Dims...>& A);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator*(const Matrix<T, Dims...>& A, T scalar);

// Matrix multiplication for 2D matrices
template <Numeric T, size_t R, size_t C, size_t K>
Matrix<T, R, K> operator*(const Matrix<T, R, C>& A, const Matrix<T, C, K>& B);

// Element-wise multiplication
template <Numeric T, size_t... Dims>
Matrix<T, Dims...> operator%(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs);

// Compound assignment operators
template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator+=(Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator-=(Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator+=(Matrix<T, Dims...>& lhs, const T& rhs);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator-=(Matrix<T, Dims...>& lhs, const T& rhs);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator*=(Matrix<T, Dims...>& lhs, const T& rhs);

template <Numeric T, size_t... Dims>
Matrix<T, Dims...>& operator/=(Matrix<T, Dims...>& lhs, const T& rhs);

// Comparison operators
template <Numeric T, size_t... Dims>
bool operator==(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs);

template <Numeric T, size_t... Dims>
bool operator!=(const Matrix<T, Dims...>& lhs, const Matrix<T, Dims...>& rhs);

#include <Matrix_Imp.h>