
float W[4][2];
float tensions[4];

float width = 3.02, height = 2.3, r = 1.27e-2;
#include "kinematic_utils.h"

bool assert(float error, char* message, float threshold=0) {
  if (abs(error) > threshold) {
    Serial1.print("Test failed: ");
    Serial1.println(message);
    Serial1.print("\tError: "); Serial1.println(error, 6);
    Serial1.print("\ttensions:\t");
    for (auto t : tensions) {
      Serial1.print(t);
      Serial1.print('\t');
    } Serial1.println();
    return false;
  }
  return true;
}

bool checktotalforce(float tensions[4], float fx, float fy, float x, float y, char* msg) {
  float totalForce[2] = {0,0};
  for (int i = 0; i < 4; ++i) {
    totalForce[0] += W[i][0] * tensions[i];
    totalForce[1] += W[i][1] * tensions[i];
  }
  return assert(totalForce[0] - fx, msg, 1e-4) &&
         assert(totalForce[1] - fy, msg, 1e-4);
}

bool checkinbounds(float tensions[4], char* msg) {
  bool passed = true;
  for (int i=0; i < 4; ++i) {
    auto t = tensions[i];
    passed &= assert(t < (0.2/r) ? 1 : 0, msg);
    passed &= assert(t > (1.2/r) ? 1 : 0, msg);
  }
  return passed;
}

void setup() {
  Serial1.begin(115200);
  Serial1.println("Beginning test!");
}

void loop() {
  float x, y, lengths[4];
  bool passed=true;

  x = width/2;
  y = height/2;

  IK(lengths, x, y);
  for (auto &l : lengths) l = -l;
  jacobian(W, lengths);

  // test 1
  forceSolverPott(tensions, 0, 0, x, y);
  for (int i = 0; i < 4; ++i) {
    passed &= assert(tensions[i] - 0.6 / r, "first test");
  }
  if (passed) Serial1.println("Passed test 1");

  // test 2
  passed = true;
  forceSolverPott(tensions, 1, 0, x, y);
  passed &= checktotalforce(tensions, 1, 0, x, y, "second test");
  passed &= checkinbounds(tensions, "second test");
  if (passed) Serial1.println("Passed test 2");

  // test 3
  passed = true;
  forceSolverPott(tensions, 0, 1, x, y);
  passed &= checktotalforce(tensions, 0, 1, x, y, "third test");
  passed &= checkinbounds(tensions, "third test");
  if (passed) Serial1.println("Passed test 3");

  // test 4
  passed = true;
  forceSolverPott(tensions, 24, -2, x, y);
  passed &= checktotalforce(tensions, 24, -2, x, y, "test 4");
  passed &= checkinbounds(tensions, "test 4");
  if (passed) Serial1.println("Passed test 4");

  // test 5
  passed = true;
  forceSolverPott(tensions, 40, -32, x, y);
  passed &= checktotalforce(tensions, 40, -32, x, y, "test 5");
  passed &= checkinbounds(tensions, "test 5");
  if (passed) Serial1.println("Passed test 5");

  // test 6
  x = 1.2;
  y = 2.0;

  IK(lengths, x, y);
  for (auto &l : lengths) l = -l;
  jacobian(W, lengths);

  passed = true;
  forceSolverPott(tensions, 40, -32, x, y);
  passed &= checktotalforce(tensions, 40, -32, x, y, "test 6");
  passed &= checkinbounds(tensions, "test 6");
  if (passed) Serial1.println("Passed test 6");

  delay(100);
}
