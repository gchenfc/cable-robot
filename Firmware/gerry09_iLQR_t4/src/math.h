#pragma once

#include <array>

#define MATRIX_FOR_LOOP(line)     \
  for (int r = 0; r < N; ++r) {   \
    for (int c = 0; c < M; ++c) { \
      line;                       \
    }                             \
  }
#define ENABLE_VECTOR int M1 = M, std::enable_if_t<M1 == 1, bool> = true

template <int N, int M>
class Matrix {
 private:
  std::array<std::array<float, M>, N> data_;
  using This = Matrix<N, M>;

 public:
  // Constructors
  Matrix() { MATRIX_FOR_LOOP(data_[r][c] = 0.0f;) };
  Matrix(const float (&data)[N][M]) {
    MATRIX_FOR_LOOP(data_[r][c] = data[r][c])
  }
  template <ENABLE_VECTOR>
  Matrix(const float (&data)[N]) {
    for (int r = 0; r < N; ++r) {
      data_[r][0] = data[r];
    }
  }
  template <ENABLE_VECTOR>
  Matrix(const std::array<float, N>& data) {
    for (int r = 0; r < N; ++r) {
      data_[r][0] = data[r];
    }
  }

  // Converters
  void toFloatArray(float (&data)[N][M]) const {
    MATRIX_FOR_LOOP(data[r][c] = data_[r][c])
  }
  template <ENABLE_VECTOR>
  void toFloatVector(float (&data)[N]) const {
    for (int r = 0; r < N; ++r) {
      data[r] = data_[r][0];
    }
  }

  // Accessors
  float operator()(const int r, const int c) const { return data_[r][c]; }
  template <ENABLE_VECTOR>
  float operator()(const int r) const {
    return data_[r][0];
  }
  float& operator()(const int r, const int c) { return data_[r][c]; }
  template <ENABLE_VECTOR>
  float& operator()(const int r) {
    return data_[r][0];
  }
  Matrix<N, 1> col(int c = 0) const {
    if (c >= M) return col();
    Matrix<N, 1> col;
    for (int r = 0; r < N; ++r) {
      col(r) = data_[r][c];
    }
    return col;
  }
  Matrix<M, 1> row(int r = 0) const { return data_.at(r); }

  // Operators
  This operator-() const {
    This result;
    MATRIX_FOR_LOOP(result(r, c) = -data_[r][c])
    return result;
  }
  This& operator+=(const This& rhs) {
    MATRIX_FOR_LOOP(data_[r][c] += rhs.data_[r][c])
    return *this;
  }
  This& operator-=(const This& rhs) { return *this += -rhs; }
  This& operator*=(const float& rhs) {
    MATRIX_FOR_LOOP(data_[r][c] *= rhs)
    return *this;
  }
  This& operator/=(const float& rhs) {
    MATRIX_FOR_LOOP(data_[r][c] /= rhs)
    return *this;
  }

  // Fancy math
  template <int N1 = N, int M1 = M,
            std::enable_if_t<(M1 == 2) && (N1 == 2), bool> = true>
  Matrix<2, 2> inv() const {
    const float det = data_[0][0] * data_[1][1] - data_[0][1] * data_[1][0];
    return Matrix<2, 2>({{data_[1][1] / det, -data_[0][1] / det},
                         {-data_[1][0] / det, data_[0][0] / det}});
  }
  Matrix<M, N> T() const {
    Matrix<M, N> result;
    for (int r = 0; r < N; ++r) {
      for (int c = 0; c < M; ++c) {
        result(c, r) = data_[r][c];
      }
    }
    return result;
  }
  Matrix<M, N> transpose() const { return T(); }
};

// Vector alias
template <int N>
using Vector = Matrix<N, 1>;

// More operators
template <int N, int M>
Matrix<N, M> operator+(Matrix<N, M> lhs, const Matrix<N, M>& rhs) {
  lhs += rhs;
  return lhs;
}
template <int N, int M>
Matrix<N, M> operator-(Matrix<N, M> lhs, const Matrix<N, M>& rhs) {
  lhs -= rhs;
  return lhs;
}
template <int N, int M, int P>
Matrix<N, P> operator*(const Matrix<N, M>& lhs, const Matrix<M, P>& rhs) {
  Matrix<N, P> result;
  for (int n = 0; n < N; ++n) {
    for (int p = 0; p < P; ++p) {
      for (int c = 0; c < M; ++c) {
        result(n, p) += lhs(n, c) * rhs(c, p);
      }
    }
  }
  return result;
}
template <int N, int M>
Matrix<N, M> operator*(Matrix<N, M> lhs, float rhs) {
  lhs *= rhs;
  return lhs;
}
template <int N, int M>
Matrix<N, M> operator*(float rhs, Matrix<N, M> lhs) {
  rhs *= lhs;
  return lhs;
}
template <int N, int M>
Matrix<N, M> operator/(Matrix<N, M> lhs, float rhs) {
  lhs /= rhs;
  return lhs;
}
template <int N, int M>
Matrix<N, M> operator/(float rhs, Matrix<N, M> lhs) {
  rhs /= lhs;
  return lhs;
}

#undef MATRIX_FOR_LOOP
#undef ENABLE_VECTOR
