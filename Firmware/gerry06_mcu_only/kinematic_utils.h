static float mountPoints[4][2] = {{width, 0}, {width, height}, {0, height}, {0, 0}};

// headers and convenience
float setZero(uint8_t node) {
  zeros[node] = pos[node];
  Serial1.print("Zeros:\t");
  for (auto z : zeros) {
    Serial1.print(z);
    Serial1.print(", ");
  }
  Serial1.println();
}
float getLen(uint8_t node) {
  return (pos[node] - zeros[node]) * r * tau;  // this has a sign error but too risky to change
}
float getLenDot(uint8_t node) {
  return vel[node] * r * tau;
}
float getPos(uint8_t node, float len) {
  return len / r / tau + zeros[node];
}
void clamp(float &x, float xmin, float xmax) {
  x = min(xmax, max(xmin, x));
}
void LPF(float &oldV, const float &newV, float alph = 0.1);
void inv2x2(float A[2][2]);
float towards(float maxD, float x, float y, float tx, float ty, float &nx, float &ny);
void jacobian(float W[4][2]);
void IK(float lengths[4], float x, float y);
void FK(float &x, float &y, const float lengths[4]);
void FK(float &x, float &y) {
  float lengths[4];
  for (int i = 0; i < 4; i++) {
    lengths[i] = getLen(i);
  }
  FK(x, y, lengths);
}
void FKv(float W[4][2], float &vx, float &vy);
void forceSolver(float tensions[4], float Fx, float Fy, float x, float y);
void forceSolver(float tensions[4], float Fx, float Fy) {
  float x, y;
  FK(x, y);
  forceSolver(tensions, Fx, Fy, x, y);
}

// impl
void LPF(float &oldV, const float &newV, float alph) {
  oldV += (newV-oldV) * alph;
}
float inv2x2(float A[2][2], float Ainv[2][2]) {
  float det = A[0][0]*A[1][1] - A[1][0]*A[0][1];  // what to do when singular/det=0?
  Ainv[0][0] = A[1][1]/det;     Ainv[0][1] = -A[0][1]/det;
  Ainv[1][0] = -A[1][0]/det;    Ainv[1][1] = A[0][0]/det;
}
float towards(float maxD, float x, float y, float tx, float ty, float &nx, float &ny) {
  float dx = tx - x;
  float dy = ty - y;
  float d2 = dx*dx + dy*dy;
  if (d2 > (maxD*maxD)) {
    float scale = maxD / sqrt(d2);
    nx = x + dx * scale;
    ny = y + dy * scale;
    // return false;
  } else {
    nx = tx;
    ny = ty;
    // return true;
  }
  return d2;
}
void jacobian(float W[4][2]) {
  float x, y;
  FK(x, y);
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
void FKv(float W[4][2], float &vx, float &vy) {
  // least squares: v = (W'.W)^(-1).W'.qdot
  float WTqdotx = 0, WTqdoty = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    float qdot = getLenDot(i);
    WTqdotx += W[i][0] * qdot;
    WTqdoty += W[i][1] * qdot;
  }
  float WTW[2][2] = {};
  for (uint8_t i = 0; i < 4; ++i) {
    WTW[0][0] += W[i][0] * W[i][0];
    WTW[1][0] += W[i][1] * W[i][0];
    WTW[1][1] += W[i][1] * W[i][1];
  }
  WTW[0][1] = WTW[1][0];
  float WTWinv[2][2];
  inv2x2(WTW, WTWinv);
  vx = WTWinv[0][0] * WTqdotx + WTWinv[0][1] * WTqdoty;
  vy = WTWinv[1][0] * WTqdotx + WTWinv[1][1] * WTqdoty;
  if (isnan(vx))
    vx = 0;
  if (isnan(vy))
    vy = 0;
}

void forceSolver(float tensions[4], float Fx, float Fy, float x, float y) {
  float mountPoints[4][2] = {{width, 0}, {width, height}, {0, height}, {0, 0}};
  uint8_t i; // cable index
  // norms will be useful later
  float norms[4];
  for (i = 0; i < 4; ++i) {
    float dx = mountPoints[i][0] - x;
    float dy = mountPoints[i][1] - y;
    norms[i] = sqrt(dx*dx + dy*dy);
  }
  // first assume every cable will pull with at least 0.2Nm and figure out how much remaining force we need
  for (i = 0; i < 4; ++i) {
    tensions[i] = 0.2 / r;
    Fx -= tensions[i]*(mountPoints[i][0] - x) / norms[i];
    Fy -= tensions[i]*(mountPoints[i][1] - y) / norms[i];
  }

  // identify which cables to use.
  // The desired force will lie between 2 cables (unless F is 0 in which case it doesn't matter)
  float dx = mountPoints[3][0] - x;
  float dy = mountPoints[3][1] - y;
  float cross1 = dx * Fy - dy * Fx;
  int cablei;
  for (cablei = 0; cablei < 4; cablei++) {
    dx = mountPoints[cablei][0] - x;
    dy = mountPoints[cablei][1] - y;
    float cross2 = dx * Fy - dy * Fx;
    if ((cross1 > 0) && (cross2 <= 0))  // cross product switches from + to -
      break;
    cross1 = cross2;
  }
  // indices to reduce if-trees
  uint8_t cable2 = cablei;          // CW side of force
  uint8_t cable1 = (cablei+3) % 4;  // CCW side of force
  uint8_t notcable1 = (cablei+1) % 4;
  uint8_t notcable2 = (cablei+2) % 4;
  // solve for:
  // (m1-x)*f1/n1 + (m2-x)*f2/n2 = F
  //    where m1/m2 are mounting points, n1/n2 are normalizing norms, and f1/f2 are cable tensions
  // [(m1x-x)/n1  (m2x-x)/n2].[f1] = [Fx] 
  // [(m1y-y)/n1  (m2y-y)/n2] [f2]   [Fy]
  float DX[2][2] = {{(mountPoints[cable1][0] - x) / norms[cable1], (mountPoints[cable2][0] - x) / norms[cable2]},
                    {(mountPoints[cable1][1] - y) / norms[cable1], (mountPoints[cable2][1] - y) / norms[cable2]}};
  float DXinv[2][2];
  inv2x2(DX, DXinv);
  // solve
  tensions[cable1] += DXinv[0][0]*Fx + DXinv[0][1]*Fy;
  tensions[cable2] += DXinv[1][0]*Fx + DXinv[1][1]*Fy;
}
