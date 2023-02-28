/* @brief A collection of Vector (aka std::array<float, N>) utilities
 * @author Gerry
 */
#pragma once

#include <array>
#include <algorithm>
#include <numeric>

#include "utils.h"

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

// head, tail, mid, padding
template <size_t NEW_DIM, size_t OLD_DIM>
Vector<NEW_DIM> head(const Vector<OLD_DIM> &x) {
  static_assert(NEW_DIM <= OLD_DIM, "New dimension must be <= old dimension");
  Vector<NEW_DIM> ret;
  std::copy(std::begin(x), std::begin(x) + NEW_DIM, std::begin(ret));
  return ret;
}
template <size_t NEW_DIM, size_t OLD_DIM>
Vector<NEW_DIM> tail(const Vector<OLD_DIM> &x) {
  static_assert(NEW_DIM <= OLD_DIM, "New dimension must be <= old dimension");
  Vector<NEW_DIM> ret;
  std::copy(std::end(x) - NEW_DIM, std::end(x), std::begin(ret));
  return ret;
}
template <size_t START, size_t LEN, size_t OLD_DIM>
Vector<LEN> mid(const Vector<OLD_DIM> &x) {
  static_assert(LEN <= OLD_DIM, "New dimension must be <= old dimension");
  static_assert(START + LEN <= OLD_DIM,
                "New dimension must be <= old dimension");
  Vector<LEN> ret;
  std::copy(std::begin(x) + START, std::begin(x) + START + LEN,
            std::begin(ret));
  return ret;
}
template <size_t NEW_DIM, size_t OLD_DIM>
Vector<NEW_DIM> pad_zeros(const Vector<OLD_DIM> &x) {
  static_assert(NEW_DIM >= OLD_DIM, "New dimension must be >= old dimension");
  Vector<NEW_DIM> ret;
  std::fill(std::begin(ret) + OLD_DIM, std::end(ret), 0);
  std::copy(std::begin(x), std::end(x), std::begin(ret));
  return ret;
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

// Clamp (inplace!!!)
template <size_t N>
void clamp_(Vector<N> &a, const Vector<N> &min, const Vector<N> &max,
            float tol = 0.0f) {
  for (int i = 0; i < static_cast<int>(N); ++i) {
    clamp(&a.at(i), min.at(i) - tol, max.at(i) + tol);
  }
}
// Clamp (functional)
template <size_t N>
Vector<N> clamp(Vector<N> a, const Vector<N> &min, const Vector<N> &max,
                float tol = 0.0f) {
  clamp_(a, min, max, tol);
  return a;
}
// Checks if within limits (i.e. would clamp do anything?).True if within limits
template <size_t N>
bool inLimits(Vector<N> a, const Vector<N> &min, const Vector<N> &max,
              float tol = 0.0f) {
  for (int i = 0; i < static_cast<int>(N); ++i) {
    if (a.at(i) < (min.at(i) - tol) || a.at(i) > (max.at(i) + tol))
      return false;
  }
  return true;
}

/******************************************************************************/
template <size_t N>
float towards(const float maxD, const Vector<N> &start, const Vector<N> &end,
              Vector<N> *next) {
  Vector<N> dx = end - start;
  float d2 = norm2(dx);
  if (d2 > (maxD * maxD)) {
    float scale = maxD / sqrt(d2);
    if (next) *next = start + dx * scale;
  } else {
    if (next) *next = end;
    return d2;
  }
}
