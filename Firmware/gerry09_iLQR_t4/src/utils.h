#pragma once

#include <algorithm>
#include <numeric>
#include <array>

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
float normalize(float (&vec)[M]);
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
float normalize(float (&vec)[M]) {
  float n = norm<M>(vec);
  if (n == 0) {
    return 0;
  }
  std::transform(vec, vec + M, vec, [n](float f) { return f / n; });
  return n;
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

/******************************************************************************/
// CircularBuffer is a queue with fixed-size memory footprint.
// CircularBufferView is a helper-class that doesn't hold any memory.
// CircularBuffer is then the "real" class that holds the memory.
/******************************************************************************/
template <class T, size_t N>
class CircularBufferView {
 protected:
  std::array<T, N> &buffer_;
  size_t head_ = 0;  // size_t is an unsigned long
  size_t tail_ = 0;
  size_t size_ = 0;
  T empty_ = T();

 public:
  CircularBufferView<T, N>(std::array<T, N> &buffer) : buffer_(buffer) {}

  // Add an item to this circular buffer.
  bool push(const T &item) {
    if (full()) return false;
    buffer_[tail_] = item;
    tail_ = (tail_ + 1) % N;
    size_ += 1;
    return true;
  }

  // Remove an item from this circular buffer and return it inplace.
  bool pop(T &output) {
    if (empty()) return false;
    output = buffer_[head_];
    return drop();
  }

  // Remove item from buffer without returning it.
  bool drop() {
    if (empty()) return false;
    // buffer_[head_] = empty_;
    head_ = (head_ + 1) % N;
    size_ -= 1;
    return true;
  }

  // Return the item at the front of this circular buffer (without removing)
  T &front() { return buffer_[head_]; }
  T &back() { return buffer_[(tail_ == 0) ? (N - 1) : (tail_ - 1)]; }

  void clear() {
    while (!empty()) drop();
  }

  bool empty() const { return size() == 0; }
  bool full() const { return size() >= N; }
  size_t size() const { return size_; }

  // TODO(gerry): implement a const_iterator, which would eliminate the need for
  // view
};

/// @brief CircularBuffer is a queue with fixed-size memory footprint.
/// @tparam T the data type held in the buffer
/// @tparam N the number of elements to allocate in the buffer
template <class T, size_t N>
class CircularBuffer : public CircularBufferView<T, N> {
 private:
  using View = CircularBufferView<T, N>;
  std::array<T, N> buffer_;

 public:
  CircularBuffer<T, N>() : View(buffer_) {
    std::fill(std::begin(buffer_), std::end(buffer_), View::empty_);
  };

  const View view() const { return *this; }
};
