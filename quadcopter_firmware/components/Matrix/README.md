# N-Dimensional Matrix Implementation

This is a new implementation of the Matrix class that supports N-dimensional tensors using C++20 features.

## Key Features

### 1. **N-Dimensional Support**
- Supports any number of dimensions (1D vectors, 2D matrices, 3D/4D/... tensors)
- Uses variadic templates to specify dimensions at compile time
- Example: `Matrix<float, 3, 4, 5>` creates a 3×4×5 tensor

### 2. **C++20 Features**
- **Concepts**: Uses `Numeric` and `FloatingPoint` concepts for type constraints
- **Requires clauses**: Ensures compile-time validation of operations
- **Improved template syntax**: Cleaner and more readable code

### 3. **Backward Compatibility**
- 2D matrices maintain all existing functionality
- Nested initializer lists still work for 2D matrices
- Hardware acceleration on ESP32-S3 is preserved for 2D operations

### 4. **Element Access**
- Variadic indexing: `tensor(i, j, k, ...)` for N-dimensional access
- Linear indexing: `tensor[idx]` for direct array access
- Special optimized path for 2D matrices

## Usage Examples

### 1D Vector
```cpp
Matrix<float, 5> vec;  // 5-element vector
vec(0) = 1.0f;
vec(4) = 5.0f;
```

### 2D Matrix (Traditional)
```cpp
Matrix<int, 3, 3> mat = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
};
auto transposed = mat.transpose();
```

### 3D Tensor
```cpp
Matrix<float, 2, 3, 4> tensor3D;
tensor3D(0, 1, 2) = 3.14f;

// Element-wise operations
auto result = tensor3D + 1.0f;  // Add 1 to all elements
```

### 4D and Higher
```cpp
Matrix<double, 2, 3, 4, 5> tensor4D;  // 2×3×4×5 tensor
std::cout << "Total elements: " << tensor4D.total_size << std::endl;  // 120
```

## API Reference

### Properties
- `ndims`: Number of dimensions
- `total_size`: Total number of elements
- `dimensions`: Array containing the size of each dimension

### Operations
- **Element-wise**: `+`, `-`, `*` (scalar), `%` (element-wise multiplication)
- **Matrix multiplication**: `*` (for 2D matrices only)
- **Norms and dot products**: `norm()`, `dot()`
- **Transpose**: `transpose()` (2D only)
- **Slicing**: `slice<dims...>(starts...)`, `setSlice()`

### Type Aliases
```cpp
template<Numeric T>
using Vector = Matrix<T, 1>;

template<Numeric T, size_t N>
using SquareMatrix = Matrix<T, N, N>;
```

## Implementation Details

### Memory Layout
- Row-major storage for 2D (same as before)
- Lexicographic order for N-D tensors
- Contiguous memory allocation

### Hardware Acceleration
- ESP32-S3 DSP acceleration preserved for 2D float/int16_t operations
- Automatically used when `USE_ACCELERATION` is defined

### Compile-Time Validation
- Dimensions must be greater than 0
- Operations validated using concepts and requires clauses
- Template errors are more readable with C++20

## Migration Guide

### From Old to New API

Most 2D code works without changes:
```cpp
// Old (still works)
Matrix<float, 3, 3> A;
A(0, 0) = 1.0f;

// New N-D capability
Matrix<float, 3, 3, 3> B;  // 3D tensor
B(0, 0, 0) = 1.0f;
```

### Slice API Changes
```cpp
// Old: A.slice<2, 2>(row, col)
// New: A.slice<2, 2>(row, col)  // Same for 2D!

// For N-D slicing (future enhancement)
// tensor.slice<2, 3, 4>(i, j, k)
```

## Performance Considerations

1. **Compile-time dimensions**: All dimensions known at compile time for optimal performance
2. **Inline operations**: Key methods marked inline for performance
3. **Hardware acceleration**: Maintained for supported operations
4. **Zero overhead**: N-D support adds no runtime cost for 2D operations

## Future Enhancements

1. **N-D slicing**: Full support for slicing in all dimensions
2. **Axis operations**: `sum()`, `mean()`, `max()` along specific axes
3. **Broadcasting**: NumPy-style broadcasting for operations
4. **Tensor contractions**: Generalized matrix multiplication for N-D
5. **Views and reshaping**: Zero-copy tensor views

## Testing

All existing tests pass, plus new tests for:
- 3D and 4D tensors
- N-dimensional element access
- Element-wise operations on N-D tensors
- Dot products and norms for N-D

Run tests:
```bash
cd components/Matrix/test/build
cmake --build .
ctest
``` 