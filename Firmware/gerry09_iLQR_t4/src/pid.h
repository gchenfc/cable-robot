#pragma once

#include "../unit_test_simulator/arduino_test_utils.h"

class Pid {
 public:
  Pid(float Kp, float Ki, float Kd, float integrator_limit = 100,
      float value = 0)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), integrator_limit_(integrator_limit) {
    reset(value);
  }

  void reset(float value = 0) {
    integrator = 0;
    prev_value = value;
    prev_time_us = micros();
  }

  float update(float value) {
    uint64_t tnow_us = micros();
    uint64_t dt_us = std::max<uint64_t>(1, tnow_us - prev_time_us);
    float dt = static_cast<float>(dt_us) / 1e6;

    integrator += Ki_ * value * dt;
    clamp(&integrator, -integrator_limit_, integrator_limit_);
    float ret = Kp_ * value + integrator + Kd_ * (value - prev_value) / dt;

    prev_value = value;
    prev_time_us = tnow_us;

    return -ret;
  }

  void setKp(float kp, bool to_reset = true) {
    Kp_ = kp;
    if (to_reset) reset(prev_value);
  }
  void setKi(float ki, bool to_reset = true) {
    Ki_ = ki;
    if (to_reset) reset(prev_value);
  }
  void setKd(float kd, bool to_reset = true) {
    Kd_ = kd;
    if (to_reset) reset(prev_value);
  }
  void setIntegratorLimit(float integrator_limit) {
    integrator_limit_ = integrator_limit;
  }

 private:
  float Kp_, Ki_, Kd_, integrator_limit_;

  float integrator = 0;
  float prev_value = 0;
  uint64_t prev_time_us = 0;
};
