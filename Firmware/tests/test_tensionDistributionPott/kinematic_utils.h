static float mountPoints[4][2] = {{width, 0}, {width, height}, {0, height}, {0, 0}};

void inv2x2(float A[2][2]);
void jacobian(float W[4][2], float lengths[4]);
void IK(float lengths[4], float x, float y);
void FK(float &x, float &y, const float lengths[4]);
void FKv(float W[4][2], float &vx, float &vy);
void forceSolverPott(float tensions[4], float Fx, float Fy, float x, float y);

// impl
float inv2x2(float A[2][2], float Ainv[2][2]) {
  float det = A[0][0]*A[1][1] - A[1][0]*A[0][1];  // what to do when singular/det=0?
  Ainv[0][0] = A[1][1]/det;     Ainv[0][1] = -A[0][1]/det;
  Ainv[1][0] = -A[1][0]/det;    Ainv[1][1] = A[0][0]/det;
}
void jacobian(float W[4][2], float lengths[4]) {
  float x, y;
  FK(x, y, lengths);
  for (uint8_t i = 0; i < 4; ++i) {
    float dx = mountPoints[i][0] - x;
    float dy = mountPoints[i][1] - y;
    float norm = sqrt(dx*dx + dy*dy);
    W[i][0] = dx / norm;
    W[i][1] = dy / norm;
  }
}
void IK(float lengths[4], float x, float y) {
  for (int i = 0; i < 4; i++) {
    float dx = x - mountPoints[i][0];
    float dy = y - mountPoints[i][1];
    lengths[i] = sqrt(x*x + y*y);
  }
}
void FK(float &x, float &y, const float lengths[4]) {
  float a = width;
  float b = -lengths[3];
  float c = -lengths[0];
  float cosC = (a*a + b*b - c*c) / (2*a*b);
  if ((cosC <= 1) && (cosC >= -1)) {
    x = b * cosC;
    y = b * sqrt(1-cosC*cosC);
  } else {
    // kinematically infeasible
    x = 0;
    y = 0;
  }
}

void forceSolverPott(float tensions[4], float Fx, float Fy, float x, float y) {
  // t = tm - pinv(W)*(f + W' * tm) = tm - W.inv(W'.W).(f + W'.tm)
  float tm[4] = {0.6 / r, 0.6 / r, 0.6 / r, 0.6 / r};
  // f - W'.tm
  for (int i = 0; i < 4; ++i) {
    Fx -= W[i][0] * tm[i];
    Fy -= W[i][1] * tm[i];
  }

  // inv(W'.W)'
  float WTW[2][2] = {};
  for (uint8_t i = 0; i < 4; ++i) {
    WTW[0][0] += W[i][0] * W[i][0];
    WTW[1][0] += W[i][1] * W[i][0];
    WTW[1][1] += W[i][1] * W[i][1];
  }
  WTW[0][1] = WTW[1][0];
  float WTWinv[2][2];
  inv2x2(WTW, WTWinv);
  // inv(W'.W) * (f + W' * tm)
  float intermediate[2];
  intermediate[0] = WTWinv[0][0] * Fx + WTWinv[0][1] * Fy;
  intermediate[1] = WTWinv[1][0] * Fx + WTWinv[1][1] * Fy;
  // tm - W.inv(W'.W).(f + W'.tm)
  for (int i = 0; i < 4; ++i) {
    tensions[i] = tm[i] + W[i][0] * intermediate[0] + W[i][1] * intermediate[1];
  }
}
