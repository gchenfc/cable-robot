#pragma once
// this file will be written by Michael and is for communicating with the
// computer.

#include <Stream.h>

class Slave {
 public:
  Slave(Stream& serial) {}

  // Common API
  void setup() {}
  void update() {}
};
