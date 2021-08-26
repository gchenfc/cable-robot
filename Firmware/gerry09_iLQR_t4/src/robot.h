/**
 * The Robot class just keeps track of the state of the robot.
 */

#pragma once

#include <array>

#include "../unit_test_simulator/arduino_test_utils.h"
#include "winch.h"

struct Robot {
  std::array<Winch, 4> winches;

  // Constructors
  Robot()
      : winches{Winch(kZeros[0]), Winch(kZeros[1]), Winch(kZeros[2]),
                Winch(kZeros[3])} {}
  explicit Robot(const float zeros[4])
      : winches{Winch(zeros[0]), Winch(zeros[1]), Winch(zeros[2]),
                Winch(zeros[3])} {}
  Robot(const float zeros[4], const float radii[4])
      : winches{Winch(zeros[0], radii[0]), Winch(zeros[1], radii[1]),
                Winch(zeros[2], radii[2]), Winch(zeros[3], radii[3])} {}
  // Delete copy/assign constructors
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  // Common API
  void setup() {}
  void update() {}

  // Calibration
  void setZero(uint8_t node) { winches[node].setZero(); }
  void setZeroAll() {
    for (auto& winch : winches) winch.setZero();
  }
  float zero(uint8_t node) { return winches[node].zero(); }
  std::array<float, 4> zeros() {
    std::array<float, 4> ret;
    for (int i = 0; i < 4; ++i) {
      ret[i] = zero(i);
    }
    return ret;
  }

  // Getter functions
  float len(uint8_t node) const { return winches[node].len(); }
  float lenDot(uint8_t node) const { return winches[node].lenDot(); }
  float theta(uint8_t node) const { return winches[node].theta(); }
  float theta(uint8_t node, float len) const {
    return winches[node].theta(len);
  }
  // In-place vector getters
  void lens(float ret[4]) const {
    for (int i = 0; i < 4; i++) {
      ret[i] = len(i);
    }
  }
  void lenDots(float ret[4]) const {
    for (int i = 0; i < 4; i++) {
      ret[i] = lenDot(i);
    }
  }
  void thetas(float ret[4]) const {
    for (int i = 0; i < 4; i++) {
      ret[i] = theta(i);
    }
  }
};
