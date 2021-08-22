/**
 * @file debug.h
 * @author Gerry Chen
 * @brief This class is for printing/sending/saving debug information.
 */

#pragma once

#include <Stream.h>

class Debug {
 public:
  Debug(Stream& serial) {}

  // Common API
  void setup() {}
  void update() {}
};
