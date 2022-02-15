#pragma once

#include <algorithm>
#include <numeric>

/// Clamps the value (in-place) between bounds
inline void clamp(float *x, float xmin, float xmax) {
  *x = min(xmax, max(xmin, *x));
}

/// Applies a simple low-pass filter (in-place) to the value.
inline void lpf(float *oldV, const float &newV, float alph = 0.1) {
  *oldV += (newV - *oldV) * alph;
}

/// Inverts a 2x2 matrix
void inv2x2(const float (&A)[2][2], float (&Ainv)[2][2]);

/// Matrix multiplication
template <int M, int N, int P>
void matmul(const float (&A)[M][N], const float (&B)[N][P], float (&C)[M][P]);

/// Matrix addition
template <int M, int N>
void matadd(const float (&A)[M][N], const float (&B)[M][N], float (&C)[M][N]);

/** Returns a new point towards the target coordinates, traveling a maximum
 * distance of `maxD`.
 * @param maxD the maximum distance we're willing to travel from the starting
 * point
 * @param x the starting x
 * @param y the starting y
 * @param tx the target x
 * @param ty the target y
 * @param[out] nx return x
 * @param[out] ny return y
 */
float towards(const float maxD, const float x, const float y, const float tx,
              const float ty, float *nx, float *ny);

// Find the squared euclidian length of a vector
template <int M>
float norm2(const float (&vec)[M]);
// Find the euclidian length of a vector
template <int M>
float norm(const float (&vec)[M]);
// Normalize a vector
template <int M>
void normalize(float (&vec)[M]);
// Find the dot product of two vectors
template <int M>
float dot(const float (&v1)[M], const float (&v2)[M]);

/************************ Implementations ************************/

/******************************************************************************/
void inv2x2(const float (&A)[2][2], float (&Ainv)[2][2]) {
  float det =
      A[0][0] * A[1][1] - A[1][0] * A[0][1];  // what to do when singular/det=0?
  Ainv[0][0] = A[1][1] / det;
  Ainv[0][1] = -A[0][1] / det;
  Ainv[1][0] = -A[1][0] / det;
  Ainv[1][1] = A[0][0] / det;
}

/******************************************************************************/
template <int M, int N, int P>
void matmul(const float (&A)[M][N], const float (&B)[N][P], float (&C)[M][P]) {
  memset(C, 0, M * P * sizeof(C[0][0]));
  for (int row = 0; row < M; ++row) {
    for (int col = 0; col < P; ++col) {
      for (int inner = 0; inner < N; ++inner) {
        C[row][col] += A[row][inner] * B[inner][col];
      }
    }
  }
}

/******************************************************************************/
template <int M, int N>
void matmul(const float (&A)[M][N], const float (&B)[N], float (&C)[M]) {
  memset(C, 0, M * sizeof(C[0]));
  for (int row = 0; row < M; ++row) {
    for (int col = 0; col < N; ++col) {
      C[row] += A[row][col] * B[col];
    }
  }
}

/******************************************************************************/
template <int M, int N>
void matadd(const float (&A)[M][N], const float (&B)[M][N], float (&C)[M][N]) {
  for (int row = 0; row < M; ++row) {
    for (int col = 0; col < N; ++col) {
      C[row][col] = A[row][col] + B[row][col];
    }
  }
}

/******************************************************************************/
template <int M>
void matadd(const float (&A)[M], const float (&B)[M], float (&C)[M]) {
  for (int row = 0; row < M; ++row) {
    C[row] = A[row] + B[row];
  }
}

/******************************************************************************/
float towards(const float maxD, const float x, const float y, const float tx,
              const float ty, float *nx, float *ny) {
  float dx = tx - x;
  float dy = ty - y;
  float d2 = dx * dx + dy * dy;
  if (d2 > (maxD * maxD)) {
    float scale = maxD / sqrt(d2);
    *nx = x + dx * scale;
    *ny = y + dy * scale;
    // return false;
  } else {
    *nx = tx;
    *ny = ty;
    // return true;
  }
  return d2;
}

/******************************************************************************/
template <int M>
float norm2(const float (&vec)[M]) {
  return std::accumulate(vec, vec + M, 0.0,
                         [](const float &running_sum, const float &value) {
                           return running_sum + value * value;
                         });
}

/******************************************************************************/
template <int M>
float norm(const float (&vec)[M]) {
  return sqrt(norm2<M>(vec));
}

/******************************************************************************/
template <int M>
void normalize(float (&vec)[M]) {
  float n = norm<M>(vec);
  if (n == 0) {
    return;
  }
  std::transform(vec, vec + M, vec, [n](float f) { return f / n; });
}

/******************************************************************************/
template <int M>
float dot(const float (&v1)[M], const float (&v2)[M]) {
  float res = 0;
  for (int i = 0; i < M; ++i) {
    res += v1[i] * v2[i];
  }
  return res;
}

/******************************************************************************/
/**
 * This is a convenience datatype that holds both a value as well as the last
 * time it got updated.  This way we can check if a value is old/expired.
 */
template <typename T>
struct Timestamped {
  T value_;
  uint64_t last_update_us_;

  explicit Timestamped(T value) : value_(value), last_update_us_(0) {}
  Timestamped<T> &operator=(T other) noexcept {
    last_update_us_ = micros();
    value_ = other;
    return *this;
  }
  operator T &() { return value_; }
  operator T() const { return value_; }
  bool isValid(uint64_t max_age_us = 5e3) const {
    return (last_update_us_) && ((micros() - last_update_us_) < max_age_us);
  }
};
