# Matrix Library Changes

## N-Dimensional Matrix Support (C++20)

### Summary
The Matrix library has been rewritten to support N-dimensional tensors while maintaining backward compatibility with 2D matrices. The implementation uses modern C++20 features for better type safety and performance.

### Key Changes

1. **N-Dimensional Support**
   - Template now accepts variadic dimensions: `Matrix<T, Dims...>`
   - Support for 1D vectors, 2D matrices, 3D tensors, and beyond
   - Example: `Matrix<float, 3, 4, 5>` creates a 3×4×5 tensor

2. **C++20 Features**
   - Concepts for type constraints (`Numeric`, `FloatingPoint`)
   - Requires clauses for compile-time validation
   - Improved template metaprogramming

3. **Backward Compatibility**
   - All existing 2D matrix operations preserved
   - Nested initializer lists still work for 2D matrices
   - Hardware acceleration on ESP32-S3 maintained

4. **New Features**
   - Compile-time dimension information via `ndims` and `dimensions[]`
   - Element access using multiple indices: `tensor(i, j, k)`
   - Efficient memory layout (row-major/lexicographic order)

5. **API Changes**
   - `slice()` and `setSlice()` methods now use explicit template parameters
   - Example: `mat.slice<2, 3>(row, col)` instead of variadic parameters

### Testing
- All existing tests pass
- New tests added for N-dimensional operations
- Build system updated to use C++20 standard

### Files Modified
- `Matrix.h` - Main header with N-dimensional template
- `Matrix_Imp.h` - Implementation details
- `Matrix_AccelerationImp.h` - Hardware acceleration (unchanged)
- `CMakeLists.txt` - Updated to C++20
- `MatrixTests.cpp` - Updated tests and added N-dimensional tests

### Examples
```cpp
// 1D Vector
Matrix<float, 5> vec;

// 2D Matrix (backward compatible)
Matrix<int, 3, 3> mat = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
};

// 3D Tensor
Matrix<double, 2, 3, 4> tensor3D;
tensor3D(0, 1, 2) = 3.14;

// 4D Tensor
Matrix<float, 2, 3, 4, 5> tensor4D;
``` 