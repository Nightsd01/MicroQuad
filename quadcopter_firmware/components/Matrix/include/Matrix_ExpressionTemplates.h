#pragma once

#include <type_traits>
#include <utility>

// Forward declaration
template <Numeric T, size_t... Dims>
class Matrix;

// Expression template base concept
template <typename E>
concept MatrixExpression = requires(E e, size_t i) {
  { e.eval(i) } -> std::convertible_to<typename E::value_type>;
  typename E::value_type;
  { E::total_size } -> std::convertible_to<size_t>;
};

// Binary operation expression template
template <typename LHS, typename RHS, typename Op>
class BinaryExpr
{
 public:
  using value_type = typename LHS::value_type;
  static constexpr size_t total_size = LHS::total_size;

  static_assert(LHS::total_size == RHS::total_size, "Matrix dimensions must match for binary operations");
  static_assert(std::is_same_v<typename LHS::value_type, typename RHS::value_type>, "Matrix element types must match");

  BinaryExpr(LHS lhs, RHS rhs) : lhs_(std::move(lhs)), rhs_(std::move(rhs)) {}

  value_type eval(size_t i) const { return Op::apply(lhs_.eval(i), rhs_.eval(i)); }

  // Subscript operator for compatibility with existing tests
  value_type operator[](size_t i) const { return eval(i); }

 private:
  LHS lhs_;  // Store by value instead of reference
  RHS rhs_;  // Store by value instead of reference
};

// Unary operation expression template
template <typename Operand, typename Op>
class UnaryExpr
{
 public:
  using value_type = typename Operand::value_type;
  static constexpr size_t total_size = Operand::total_size;

  UnaryExpr(Operand operand) : operand_(std::move(operand)) {}

  value_type eval(size_t i) const { return Op::apply(operand_.eval(i)); }

  // Subscript operator for compatibility with existing tests
  value_type operator[](size_t i) const { return eval(i); }

 private:
  Operand operand_;  // Store by value instead of reference
};

// Scalar operation expression template
template <typename Operand, typename Op>
class ScalarExpr
{
 public:
  using value_type = typename Operand::value_type;
  static constexpr size_t total_size = Operand::total_size;

  ScalarExpr(Operand operand, const value_type& scalar) : operand_(std::move(operand)), scalar_(scalar) {}

  value_type eval(size_t i) const { return Op::apply(operand_.eval(i), scalar_); }

  // Subscript operator for compatibility with existing tests
  value_type operator[](size_t i) const { return eval(i); }

 private:
  Operand operand_;  // Store by value instead of reference
  const value_type scalar_;
};

// Reverse scalar operation expression template (for scalar - matrix, etc.)
template <typename Operand, typename Op>
class ReverseScalarExpr
{
 public:
  using value_type = typename Operand::value_type;
  static constexpr size_t total_size = Operand::total_size;

  ReverseScalarExpr(const value_type& scalar, Operand operand) : scalar_(scalar), operand_(std::move(operand)) {}

  value_type eval(size_t i) const { return Op::apply(scalar_, operand_.eval(i)); }

  // Subscript operator for compatibility with existing tests
  value_type operator[](size_t i) const { return eval(i); }

 private:
  const value_type scalar_;
  Operand operand_;  // Store by value instead of reference
};

// Matrix wrapper for expression templates
template <Numeric T, size_t... Dims>
class MatrixExpr
{
 public:
  using value_type = T;
  static constexpr size_t total_size = (Dims * ...);

  MatrixExpr(const Matrix<T, Dims...>& matrix) : matrix_(matrix) {}

  T eval(size_t i) const { return matrix_.data[i]; }

  // Subscript operator for compatibility with existing tests
  T operator[](size_t i) const { return matrix_.data[i]; }

 private:
  const Matrix<T, Dims...>& matrix_;  // Keep reference for MatrixExpr since Matrix objects are not temporaries
};

// Operation functors
struct AddOp
{
  template <typename T>
  static T apply(const T& a, const T& b)
  {
    return a + b;
  }
};

struct SubOp
{
  template <typename T>
  static T apply(const T& a, const T& b)
  {
    return a - b;
  }
};

struct MulOp
{
  template <typename T>
  static T apply(const T& a, const T& b)
  {
    return a * b;
  }
};

struct DivOp
{
  template <typename T>
  static T apply(const T& a, const T& b)
  {
    return a / b;
  }
};

struct NegOp
{
  template <typename T>
  static T apply(const T& a)
  {
    return -a;
  }
};

// Type aliases for common expressions
template <typename LHS, typename RHS>
using AddExpr = BinaryExpr<LHS, RHS, AddOp>;

template <typename LHS, typename RHS>
using SubExpr = BinaryExpr<LHS, RHS, SubOp>;

template <typename LHS, typename RHS>
using MulExpr = BinaryExpr<LHS, RHS, MulOp>;

template <typename LHS, typename RHS>
using DivExpr = BinaryExpr<LHS, RHS, DivOp>;

template <typename Operand>
using NegExpr = UnaryExpr<Operand, NegOp>;

template <typename Operand>
using ScalarAddExpr = ScalarExpr<Operand, AddOp>;

template <typename Operand>
using ScalarSubExpr = ScalarExpr<Operand, SubOp>;

template <typename Operand>
using ScalarMulExpr = ScalarExpr<Operand, MulOp>;

template <typename Operand>
using ScalarDivExpr = ScalarExpr<Operand, DivOp>;

template <typename Operand>
using ReverseScalarSubExpr = ReverseScalarExpr<Operand, SubOp>;

template <typename Operand>
using ReverseScalarDivExpr = ReverseScalarExpr<Operand, DivOp>;

// Helper to check if a type is an expression
template <typename T>
struct is_matrix_expression : std::false_type
{
};

template <typename LHS, typename RHS, typename Op>
struct is_matrix_expression<BinaryExpr<LHS, RHS, Op>> : std::true_type
{
};

template <typename Operand, typename Op>
struct is_matrix_expression<UnaryExpr<Operand, Op>> : std::true_type
{
};

template <typename Operand, typename Op>
struct is_matrix_expression<ScalarExpr<Operand, Op>> : std::true_type
{
};

template <typename Operand, typename Op>
struct is_matrix_expression<ReverseScalarExpr<Operand, Op>> : std::true_type
{
};

template <Numeric T, size_t... Dims>
struct is_matrix_expression<MatrixExpr<T, Dims...>> : std::true_type
{
};

template <typename T>
constexpr bool is_matrix_expression_v = is_matrix_expression<T>::value;

// Scalar multiplication operators for expressions
template <typename Expr>
auto operator*(const Expr& expr, typename Expr::value_type scalar) -> ScalarMulExpr<Expr>
  requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { expr.eval(size_t{}) } -> std::convertible_to<typename Expr::value_type>;
  }
{
  return ScalarMulExpr<Expr>(expr, scalar);
}

template <typename Expr>
auto operator*(typename Expr::value_type scalar, const Expr& expr) -> ScalarMulExpr<Expr>
  requires requires {
    typename Expr::value_type;
    { Expr::total_size } -> std::convertible_to<size_t>;
    { expr.eval(size_t{}) } -> std::convertible_to<typename Expr::value_type>;
  }
{
  return ScalarMulExpr<Expr>(expr, scalar);
}