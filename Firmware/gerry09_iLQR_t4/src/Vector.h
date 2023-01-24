/* @brief A collection of Vector (aka std::array<float, N>) utilities
 * @author Gerry
 */
#pragma once

#include <array>
#include <algorithm>
#include <numeric>

// Define a Vector class
template <size_t N>
using Vector = std::array<float, N>;

// "Constructors" / adapters
Vector<2> makeVector(const std::pair<float, float> &x) {
  return {x.first, x.second};
}
std::pair<float, float> makePair(const Vector<2> &x) {
  return {std::get<0>(x), std::get<1>(x)};
}

// Convenience functions to wrap std::transform
namespace detail {
template <size_t N, typename UNARY_OP>
Vector<N> transform(const Vector<N> &a, UNARY_OP f) {
  Vector<N> ret;
  std::transform(std::begin(a), std::end(a), std::begin(ret), f);
  return ret;
}
template <size_t N, typename UNARY_OP>
Vector<N> transform(const Vector<N> &a, const Vector<N> &b, UNARY_OP f) {
  Vector<N> ret;
  std::transform(std::begin(a), std::end(a), std::begin(b), std::begin(ret), f);
  return ret;
}
}  // namespace detail

// Elementwise Addition
template <size_t N>
Vector<N> operator+(const Vector<N> &a, const Vector<N> &b) {
  return detail::transform(a, b, std::plus<float>());
}

// Elementwise Subtraction
template <size_t N>
Vector<N> operator-(const Vector<N> &a, const Vector<N> &b) {
  return detail::transform(a, b, std::minus<float>());
}

// Elementwise Multiplication
template <size_t N>
Vector<N> operator*(const Vector<N> &a, const Vector<N> &b) {
  return detail::transform(a, b, std::multiplies<float>());
}

// Scalar Multiplication
template <size_t N>
Vector<N> operator*(const Vector<N> &a, float b) {
  return detail::transform(a, [&b](float x) { return x * b; });
}
template <size_t N>
Vector<N> operator*(float b, const Vector<N> &a) {
  return a * b;
}

// Scalar Division
template <size_t N>
Vector<N> operator/(const Vector<N> &a, float b) {
  return a * (1 / b);
}
// Scalar Division
template <size_t N>
Vector<N> operator/(float b, const Vector<N> &a) {
  return detail::transform(a, [&b](float x) { return b / x; });
}

// Dot Product
template <size_t N>
float dot(const Vector<N> &a, const Vector<N> &b) {
  return std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0f);
}

// Cross Product
Vector<3> cross(const Vector<3> &a, const Vector<3> &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
float cross(const Vector<2> &a, const Vector<2> &b) {
  return a[0] * b[1] - a[1] * b[0];
}

// Norm
template <size_t N>
float norm2(const Vector<N> &a) {
  return dot(a, a);
}
template <size_t N>
float norm(const Vector<N> &a) {
  return sqrt(norm2(a));
}
template <size_t N>
Vector<N> normalized(const Vector<N> &a) {
  return a / norm(a);
}
