/* @brief Piecewise polynomial math utilities
 * @author Gerry
 *
 * Polynomials are represented as as std::array of coefficients, where the first
 * element is the highest order term and the last element is the constant term.
 * For example, the polynomial 3x^2 + 2x + 1 would be represented as {3, 2, 1}.
 *
 * Piecewise Polynomials (PPoly) are represented with synchronized breakpoints
 * and polynomials.  The breakpoints are expressed as the independent variable
 * (likely time), and the polynomials should be evaluated at the time minus the
 * start time of that polynomial (reparameterize so that each polynomial starts
 * at 0).
 */

#pragma once

#include <array>

/******************************* Typedefs *************************************/
template <int MaxSegments>
using PPolyBreakpoints = std::array<float, MaxSegments + 1>;

template <int Degree = 3>
using PolyCoeffs1d = std::array<float, Degree + 1>;
template <int Dims = 2, int Degree = 3>
using PolyCoeffs = std::array<PolyCoeffs1d<Degree>, Dims>;
template <int MaxSegments, int Dims = 2, int Degree = 3>
using PPolyCoeffs = std::array<PolyCoeffs<Dims, Degree>, MaxSegments>;

using CubicPolyCoeffs1d = PolyCoeffs1d<3>;
using CubicPolyCoeffs = PolyCoeffs<3>;
using CubicPPolyCoeffs = PPolyCoeffs<3>;

/******************************* Constants ************************************/

template <int DIMS>
constexpr std::array<float, DIMS> kZero() {
  std::array<float, DIMS> ret{};
  ret.fill(0);
  return ret;
}
template <int DIMS>
constexpr std::array<float, DIMS> kDefaultPos() {
  return kZero<DIMS>();
}
template <>
constexpr std::array<float, 2> kDefaultPos<2>() {
  return {1.5, 1.5};
}

/*************************** Polynomial Utils *********************************/
template <int Degree>
float polyval(float t, const PolyCoeffs1d<Degree>& coeffs) {
  float y = coeffs[0];
  for (int i = 1; i <= Degree; ++i) {
    y *= t;
    y += coeffs[i];
  }
  return y;
}
template <int Dims, int Degree>
std::array<float, Dims> polyval(float t,
                                const PolyCoeffs<Dims, Degree>& coeffs) {
  std::array<float, Dims> ret;
  std::transform(std::begin(coeffs), std::end(coeffs), std::begin(ret),
                 [&t](auto coeff) { return polyval<Degree>(t, coeff); });
  return ret;
}

template <int Degree>
PolyCoeffs1d<std::max<int>(0, Degree - 1)> polyder(
    const PolyCoeffs1d<Degree>& coeffs) {
  PolyCoeffs1d<std::max<int>(0, Degree - 1)> ret;
  for (int i = 0; i < Degree; ++i) {
    ret[i] = (Degree - i) * coeffs[i];
  }
  return ret;
}
template <>
PolyCoeffs1d<0> polyder<0>(const PolyCoeffs1d<0>& coeffs
                           __attribute__((unused))) {
  return {0};
}
template <int Dim, int Degree>
PolyCoeffs<Dim, std::max<int>(0, Degree - 1)> polyder(
    const PolyCoeffs<Dim, Degree>& coeffs) {
  PolyCoeffs<Dim, std::max<int>(0, Degree - 1)> ret;
  std::transform(std::begin(coeffs), std::end(coeffs), std::begin(ret),
                 &polyder<Degree>);
  return ret;
}

/************************* Piecewise Polynomial *******************************/
template <int MaxSegments, int Dims = 2, int Degree = 3>
class PPoly {
 public:
  using PPolyBreakpoints_ = PPolyBreakpoints<MaxSegments>;
  using PolyCoeffs1d_ = PolyCoeffs1d<Degree>;
  using PolyCoeffs_ = PolyCoeffs<Dims, Degree>;
  enum { XDeg = Degree };
  enum { VDeg = std::max<int>(0, Degree - 1) };
  enum { ADeg = std::max<int>(0, Degree - 2) };
  using PPolyCoeffs_ = PPolyCoeffs<MaxSegments, Dims, Degree>;
  using PPolyCoeffsD = PPolyCoeffs<MaxSegments, Dims, VDeg>;
  using PPolyCoeffsDD = PPolyCoeffs<MaxSegments, Dims, ADeg>;

  using X = std::array<float, Dims>;

 protected:
  PPolyBreakpoints_ breakpoints_;  // First breakpoint should always be at 0!!!
  PPolyCoeffs_ coeffs_;            // (segment, dimension, degree)
  PPolyCoeffsD coeffsd_;
  PPolyCoeffsDD coeffsdd_;
  int n_segments_, cur_segment_;

 public:
  PPoly() : n_segments_(0), cur_segment_(0) { breakpoints_[0] = 0; }

  // Returns the segment index number and the time relative to that segment
  std::pair<int, float> reparameterize_dumb(float t) const;

  // Same as `reparameterize_dumb` except caches result
  std::pair<int, float> reparameterize(float t);

  // Evaluate the `Deg`th derivative
  template <int Deg>
  X eval(float t, PPolyCoeffs<MaxSegments, Dims, Deg> coeffs,
         bool is_pos = true) const;

  // Evaluate the pos, vel, acc
  X eval(float t) const { return eval<XDeg>(t, coeffs_, true); }
  X evald(float t) const { return eval<VDeg>(t, coeffsd_, false); }
  X evaldd(float t) const { return eval<ADeg>(t, coeffsdd_, false); }
  float duration() const {
    return (n_segments_ == 0) ? 0.0 : breakpoints_[n_segments_];
  }

  void reset() {
    n_segments_ = 0;
    cur_segment_ = 0;
    breakpoints_[0] = 0;
  }
  bool add_segment(float tmax, PolyCoeffs_ coeffs) {
    if (n_segments_ >= MaxSegments) return false;
    coeffs_[n_segments_] = coeffs;
    coeffsd_[n_segments_] = polyder<Dims, XDeg>(coeffs);
    coeffsdd_[n_segments_] = polyder<Dims, VDeg>(coeffsd_[n_segments_]);
    breakpoints_[++n_segments_] = tmax;
    return true;
  }

  // Getters
  float get_breakpoint(int seg) const { return breakpoints_[seg]; }
  int get_n_segments() const { return n_segments_; }
  PolyCoeffs_ get_coeffs(int seg) const { return coeffs_[seg]; }
};

/************************** IMPLEMENTATION ************************************/

// Returns the segment index number and the time relative to that segment
template <int A, int B, int C>
std::pair<int, float> PPoly<A, B, C>::reparameterize_dumb(float t) const {
  if (n_segments_ == 0) {
    return {-1, 0};
  }
  if (t < 0) {
    return {0, 0};
  }
  for (int cur_segment = 0; cur_segment < n_segments_; ++cur_segment) {
    if (t < breakpoints_[cur_segment + 1]) {
      return {cur_segment, t - breakpoints_[cur_segment]};
    }
  }
  return {n_segments_, 0};
}

// Same as `reparameterize_dumb` except caches result
template <int A, int B, int C>
std::pair<int, float> PPoly<A, B, C>::reparameterize(float t) {
  if (n_segments_ == 0) {
    return {-1, 0};
  }
  if (t < breakpoints_[cur_segment_]) {  // we went back in time.  Re-search
    auto ret = reparameterize_dumb(t);
    cur_segment_ = ret.first;
    return ret;
  } else {
    while (t >= breakpoints_[cur_segment_ + 1]) {  // next segment
      ++cur_segment_;
      if (cur_segment_ >= n_segments_) {
        --cur_segment_;
        return {n_segments_, 0};
      }
    }
    // Now (cur_segment_) <= t <= (cur_segment_ + 1)
    return {cur_segment_, t - breakpoints_[cur_segment_]};
  }
}

template <int A, int B, int C>
template <int Deg>
typename PPoly<A, B, C>::X PPoly<A, B, C>::eval(float t,
                                                PPolyCoeffs<A, B, Deg> coeffs,
                                                bool is_pos) const {
  auto reparam = reparameterize_dumb(t);
  int& index = reparam.first;
  float& t_ = reparam.second;
  if (index == -1) {                                // if before beginning
    return is_pos ? kDefaultPos<B>() : kZero<B>();  // default position/vel/acc
  } else if (index == n_segments_) {                // if after end
    if (!is_pos) return kZero<B>();                 // 0 vel/acc
    index = n_segments_ - 1;
    t_ = breakpoints_[n_segments_] - breakpoints_[index];  // stay at last pos
  }

  return polyval<B, Deg>(t_, coeffs[index]);
}
