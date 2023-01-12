/* @brief Piecewise polynomial math utilities
 * @author Gerry
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
  static_assert(
      Dims == 2,
      "Dimensions other than 2 not implemented yet (see eval functions)");

 protected:
  PPolyBreakpoints_ breakpoints_;  // First breakpoint should always be at 0!!!
  PPolyCoeffs_ coeffs_;           // (segment, dimension, degree)
  PPolyCoeffsD coeffsd_;
  PPolyCoeffsDD coeffsdd_;
  int n_segments_, cur_segment_;

 public:
  PPoly() : n_segments_(0), cur_segment_(0) { breakpoints_[0] = 0; }

  // Returns the segment index number and the time relative to that segment
  std::pair<int, float> reparameterize_dumb(float t) const;

  // Same as `reparameterize_dumb` except caches result
  std::pair<int, float> reparameterize(float t);

  template <int Deg>
  X eval(float t, PPolyCoeffs<MaxSegments, Dims, Deg> coeffs,
         bool is_pos = true) const;

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
  using X = PPoly<A, B, C>::X;

  auto reparam = reparameterize_dumb(t);
  int& index = reparam.first;
  float& t_ = reparam.second;
  if (index == -1) {                        // if before beginning
    return is_pos ? X{1.5, 1.5} : X{0, 0};  // rest at center
  } else if (index == n_segments_) {        // if after end
    if (!is_pos) return {0, 0};             // 0 vel/acc
    index = n_segments_ - 1;
    t_ = breakpoints_[n_segments_] - breakpoints_[index];  // stay at last pos
  }

  return {polyval<Deg>(t_, coeffs[index][0]),
          polyval<Deg>(t_, coeffs[index][1])};
}
