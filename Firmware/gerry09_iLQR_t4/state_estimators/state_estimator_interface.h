#pragma once

#include "../robot.h"

class StateEstimatorInterface {
 public:
  // Common API
  virtual void setup() {}
  virtual void update() {}

  //
  virtual std::pair<float, float> pos() const = 0;
  virtual std::pair<float, float> pos(uint64_t time_us) const = 0;
  virtual std::pair<float, float> vel() const = 0;
  virtual std::pair<float, float> vel(uint64_t time_us) const { return vel(); };
};
