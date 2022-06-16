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
  Robot() : winches{Winch(0), Winch(1), Winch(2), Winch(3)} {}
  // Delete copy/assign constructors
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  // Common API
  void setup() {
    restoreZeros();
    if (kOverwriteLenCorrectionParamsToEeprom) saveLenCorrectionParams();
    if (kOverwriteMountPointsToEeprom) saveMountPoints();
    if (kUseLenCorrectionParamsFromEeprom) restoreLenCorrectionParams();
    if (kUseMountPointsFromEeprom) restoreMountPoints();
  }
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

  // Saving/Restoring parameters
  static_assert((1 << 5) > Winch::STORAGE_SIZE,
                "EEPROM storage size too large for indexing "
                "scheme");
  void saveZero(uint8_t node) const { winches[node].saveZero(node << 5); }
  void restoreZero(uint8_t node) { winches[node].restoreZero(node << 5); }
  void saveLenCorrectionParams(uint8_t node) const {
    winches[node].saveLenCorrectionParams(node << 5);
  }
  void restoreLenCorrectionParams(uint8_t node) {
    winches[node].restoreLenCorrectionParams(node << 5);
  }
  void saveMountPoint(uint8_t node) const {
    winches[node].saveMountPoint(node << 5);
  }
  void restoreMountPoint(uint8_t node) {
    winches[node].restoreMountPoint(node << 5);
  }
  void saveAll(uint8_t node) const { winches[node].saveAll(node << 5); }
  void restoreAll(uint8_t node) { winches[node].restoreAll(node << 5); }
  void saveZeros() const {
    for (int i = 0; i < 4; ++i) saveZero(i);
  }
  void restoreZeros() {
    for (int i = 0; i < 4; ++i) restoreZero(i);
  }
  void saveLenCorrectionParams() const {
    for (int i = 0; i < 4; ++i) saveLenCorrectionParams(i);
  }
  void restoreLenCorrectionParams() {
    for (int i = 0; i < 4; ++i) restoreLenCorrectionParams(i);
  }
  void saveMountPoints() const {
    for (int i = 0; i < 4; ++i) saveMountPoint(i);
  }
  void restoreMountPoints() {
    for (int i = 0; i < 4; ++i) restoreMountPoint(i);
  }
  void saveAlls() const {
    for (int i = 0; i < 4; ++i) saveAll(i);
  }
  void restoreAlls() {
    for (int i = 0; i < 4; ++i) restoreAll(i);
  }
};
