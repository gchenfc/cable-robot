/**
 * @file  testMotorFactor.cpp
 * @brief test cable factor
 * @Author: Frank Dellaert, Gerry Chen and Zhangqi Luo
 */

#include "factors/MotorFactor.h"

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>
#include <array>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test motor factor
*/

TEST(MotorFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol tq = symbol('q', 0);
  Symbol ct = symbol('c', 0);
  MotorFactor factor(tq,  ct, cost_model);

  std::array<double, 3> tqValue = {0.2, -70, 70}; 
  std::array<double, 3> tcValue = {0.1, -2.5, 2.5};
  std::array<double, 3> answer = {-0.2, 0,0};

  double tqHolder;
  double tcHolder;
  gtsam::Matrix11 answerHolder;  
  for (int i =0; i < tqValue.size(); i++){
    tqHolder = tqValue[i];
    tcHolder = tcValue[i];
    answerHolder << answer[i];
    Vector actual_errors = factor.evaluateError(
      tqHolder, tcHolder);

    EXPECT(assert_equal(answerHolder, actual_errors, 1e-6));

  }

  Values values;
  values.insertDouble(tq, tqHolder);
  values.insertDouble(ct, tcHolder);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}