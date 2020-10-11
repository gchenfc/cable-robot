/**
 * @file  testWrenchFactor.cpp
 * @brief test cable factor
 * @Author: Frank Dellaert, Gerry Chen and Zhangqi Luo
 */

#include "factors/WrenchFactor.h"

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


TEST(WrenchFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol tq = symbol('q', 0);
  Symbol ten = symbol('t', 0);
  WrenchFactor factor(ten,  tq, cost_model);

  std::array<double, 2> tenValue = {100, 200}; 
  std::array<double, 2> tqValue = {5, 10};
  std::array<double, 2> answer = {25, 50};

  double tqHolder;
  double tenHolder;
 gtsam::Matrix11 answerHolder;  
  for (int i =0; i < tqValue.size(); i++){
    tqHolder = tqValue[i];
    tenHolder = tenValue[i];
    answerHolder << answer[i];
    
    Vector actual_errors = factor.evaluateError( tenHolder, tqHolder);

    EXPECT(assert_equal(answerHolder, actual_errors, 1e-4));
  }

  Values values;
  values.insertDouble(tq, tqHolder);
  values.insertDouble(ten, tenHolder);EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}