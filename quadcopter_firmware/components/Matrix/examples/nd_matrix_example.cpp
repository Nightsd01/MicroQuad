/**
 * Example demonstrating N-dimensional Matrix capabilities
 * 
 * This example shows how to use the new N-dimensional Matrix class
 * with various dimensions, from 1D vectors to 4D tensors.
 */

#include <iostream>
#include "Matrix.h"

int main() {
    std::cout << "=== N-Dimensional Matrix Examples ===" << std::endl << std::endl;
    
    // 1D Vector (5 elements)
    std::cout << "1D Vector Example:" << std::endl;
    Matrix<float, 5> vec;
    for (size_t i = 0; i < 5; ++i) {
        vec(i) = static_cast<float>(i * 2.0f);
    }
    std::cout << "Vector: " << vec << std::endl;
    std::cout << "Norm: " << vec.norm() << std::endl << std::endl;
    
    // 2D Matrix (traditional)
    std::cout << "2D Matrix Example:" << std::endl;
    Matrix<int, 3, 3> mat2D = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    std::cout << "Matrix:\n" << mat2D << std::endl;
    
    // Transpose (only for 2D)
    auto mat2D_T = mat2D.transpose();
    std::cout << "Transposed:\n" << mat2D_T << std::endl << std::endl;
    
    // 3D Tensor
    std::cout << "3D Tensor Example:" << std::endl;
    Matrix<float, 2, 3, 4> tensor3D;
    
    // Initialize with sequential values
    float value = 0.0f;
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            for (size_t k = 0; k < 4; ++k) {
                tensor3D(i, j, k) = value++;
            }
        }
    }
    
    std::cout << "3D Tensor shape: " << tensor3D.ndims << "D with dimensions [";
    for (size_t i = 0; i < tensor3D.ndims; ++i) {
        std::cout << tensor3D.dimensions[i];
        if (i < tensor3D.ndims - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "Total elements: " << tensor3D.total_size << std::endl;
    std::cout << "Sample values: tensor3D(0,0,0) = " << tensor3D(0,0,0) 
              << ", tensor3D(1,2,3) = " << tensor3D(1,2,3) << std::endl << std::endl;
    
    // Element-wise operations on 3D tensors
    std::cout << "Element-wise Operations on 3D Tensors:" << std::endl;
    Matrix<float, 2, 2, 2> A(1.0f);  // All elements = 1.0
    Matrix<float, 2, 2, 2> B(2.0f);  // All elements = 2.0
    
    auto C = A + B;  // Element-wise addition
    auto D = B * 3.0f;  // Scalar multiplication
    
    std::cout << "A + B: All elements should be 3.0" << std::endl;
    std::cout << "First element: " << C(0,0,0) << ", Last element: " << C(1,1,1) << std::endl;
    
    std::cout << "B * 3: All elements should be 6.0" << std::endl;
    std::cout << "First element: " << D(0,0,0) << ", Last element: " << D(1,1,1) << std::endl << std::endl;
    
    // Dot product of 3D tensors
    std::cout << "Dot Product of 3D Tensors:" << std::endl;
    float dot_product = A.dot(B);
    std::cout << "A.dot(B) = " << dot_product << " (should be 8*1*2 = 16)" << std::endl << std::endl;
    
    // 4D Tensor
    std::cout << "4D Tensor Example:" << std::endl;
    Matrix<double, 2, 2, 2, 2> tensor4D(1.5);  // All elements = 1.5
    std::cout << "4D Tensor with all elements = 1.5" << std::endl;
    std::cout << "Shape: [" << tensor4D.dimensions[0] << ", " 
              << tensor4D.dimensions[1] << ", " 
              << tensor4D.dimensions[2] << ", " 
              << tensor4D.dimensions[3] << "]" << std::endl;
    std::cout << "Norm: " << tensor4D.norm() << std::endl << std::endl;
    
    // Type aliases for convenience
    std::cout << "Using Type Aliases:" << std::endl;
    Vector<float> vec1D;  // Equivalent to Matrix<float, 1>
    SquareMatrix<float, 3> sqMat;  // Equivalent to Matrix<float, 3, 3>
    std::cout << "Vector<float> is a " << vec1D.ndims << "D tensor" << std::endl;
    std::cout << "SquareMatrix<float, 3> is a " << sqMat.ndims << "D tensor" << std::endl;
    
    return 0;
} 