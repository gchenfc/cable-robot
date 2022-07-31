#pragma once

#include <array>

#define MATRIX_FOR_LOOP(line)     \
  for (int r = 0; r < N; ++r) {   \
    for (int c = 0; c < M; ++c) { \
      line;                       \
    }                             \
  }
#define ENABLE_VECTOR int M1 = M, std::enable_if_t<M1 == 1, bool> = true

// forward declarations
template <int N, int M>
struct ElementwiseView;

// Matrix Class
template <int N, int M>
class Matrix {
 private:
  std::array<std::array<float, M>, N> data_;
  using This = Matrix<N, M>;
  typedef float CMatrix[N][M];
  typedef float CVector[N];

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
  void toCMatrix(float (&data)[N][M]) const {
    MATRIX_FOR_LOOP(data[r][c] = data_[r][c])
  }
  template <ENABLE_VECTOR>
  void toCVector(float (&data)[N]) const {
    for (int r = 0; r < N; ++r) {
      data[r] = data_[r][0];
    }
  }
  // TODO(gerry): is this UB?  I don't think so
  CMatrix& asCMatrix() {
    return *reinterpret_cast<CMatrix*>(data_.data()->data());
  }
  template <ENABLE_VECTOR>
  CVector& asCVector() {
    return *reinterpret_cast<CVector*>(data_.data()->data());
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

  template <int R, int C, int NROWS, int NCOLS,
            std::enable_if_t<(R + NROWS <= N) && (C + NCOLS <= M), bool> = true>
  Matrix<NROWS, NCOLS> block() {
    Matrix<NROWS, NCOLS> block;
    for (int r = 0; r < NROWS; ++r) {
      for (int c = 0; c < NCOLS; ++c) {
        block(r, c) = data_[r + R][c + C];
      }
    }
    return block;
  }

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
  ElementwiseView<N, M> elementwise() { return ElementwiseView<N, M>(*this); }
  template <int N1 = N, int M1 = M,
            std::enable_if_t<(M1 == 2) && (N1 == 2), bool> = true>
  Matrix<2, 2> inv() const {
    const float det = data_[0][0] * data_[1][1] - data_[0][1] * data_[1][0];
    return Matrix<2, 2>({{data_[1][1] / det, -data_[0][1] / det},
                         {-data_[1][0] / det, data_[0][0] / det}});
  }
};

// Vector alias
template <int N>
using Vector = Matrix<N, 1>;

// Matrix Views
template <int N, int M>
struct MatrixView {
  Matrix<N, M>& matrix;
  MatrixView(Matrix<N, M>& matrix) : matrix(matrix) {}
};
template <int N, int M>
struct ElementwiseView : public MatrixView<N, M> {
  using Mat = Matrix<N, M>;
  using MatrixView<N, M>::MatrixView;
  Mat& operator*=(const Mat& rhs) {
    MATRIX_FOR_LOOP(this->matrix(r, c) *= rhs(r, c))
    return this->matrix;
  }
  Mat& operator/=(const Mat& rhs) {
    MATRIX_FOR_LOOP(this->matrix(r, c) /= rhs(r, c))
    return this->matrix;
  }
};

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
Matrix<N, M> operator*(const ElementwiseView<N, M>& lhs, Matrix<N, M> rhs) {
  rhs.elementwise() *= lhs.matrix;
  return rhs;
}
template <int N, int M>
Matrix<N, M> operator/(const ElementwiseView<N, M>& lhs,
                       const Matrix<N, M>& rhs) {
  Matrix<N, M> copy = lhs.matrix;
  copy.elementwise() /= rhs;
  return copy;
}
template <int N, int M>
Matrix<N, M> operator*(Matrix<N, M> lhs, float rhs) {
  lhs *= rhs;
  return lhs;
}
template <int N, int M>
Matrix<N, M> operator*(float lhs, Matrix<N, M> rhs) {
  rhs *= lhs;
  return rhs;
}
template <int N, int M>
Matrix<N, M> operator/(Matrix<N, M> lhs, float rhs) {
  lhs /= rhs;
  return lhs;
}

// Functions
template <int N, int M>
Matrix<N, M> sign(Matrix<N, M> lhs) {
  MATRIX_FOR_LOOP(lhs(r, c) = (lhs(r, c) > 0) ? 1 : -1);
  return lhs;
}

// Least Squares: solve min||Ax - b||^2 for x (using pseudoinverse)
// Can solve multiple problems simultaneously if b has multiple columns
template <int M, int N, int P = 1>
Matrix<N, P> leastSquares(const Matrix<M, N>& A, const Matrix<M, P>& b) {
  // least squares: x = (W'.W)^(-1).W'.b
  return (A.T() * A).inv() * (A.T() * b);
}

#undef MATRIX_FOR_LOOP
#undef ENABLE_VECTOR
