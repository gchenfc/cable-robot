#pragma once

/// Clamps the value (in-place) between bounds
inline void clamp(float *x, float xmin, float xmax) {
  *x = min(xmax, max(xmin, *x));
}

/// Applies a simple low-pass filter (in-place) to the value.
inline void lpf(float *oldV, const float &newV, float alph = 0.1) {
  *oldV += (newV - *oldV) * alph;
}

/// Inverts a 2x2 matrix
void inv2x2(const float A[2][2], float Ainv[2][2]);

/// Matrix multiplication
template <int M, int N, int P>
void matmul(const float A[M][N], const float B[N][P], float C[M][P]);

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

/************************ Implementations ************************/

/******************************************************************************/
void inv2x2(const float A[2][2], float Ainv[2][2]) {
  float det =
      A[0][0] * A[1][1] - A[1][0] * A[0][1];  // what to do when singular/det=0?
  Ainv[0][0] = A[1][1] / det;
  Ainv[0][1] = -A[0][1] / det;
  Ainv[1][0] = -A[1][0] / det;
  Ainv[1][1] = A[0][0] / det;
}

/******************************************************************************/
template <int M, int N, int P>
void matmul(const float A[M][N], const float B[N][P], float C[M][P]) {
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