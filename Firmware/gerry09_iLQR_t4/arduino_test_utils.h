/**
 * This file provides some (dummy) implementations of functions which would be
 * defined by Arduino, so that we can unit test code "offline".
 */

#ifndef Arduino_h

#pragma once

#include <time.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
time_t t_start = time(0);

template <class T>
T min(const T& t1, const T& t2) {
  return std::min(t1, t2);
}
template <class T>
T max(const T& t1, const T& t2) {
  return std::max(t1, t2);
}

template <class T>
T sqrt(const T& t) {
  return std::sqrt(t);
}

uint64_t millis() { return difftime(time(0), t_start) * 1000; }

class Print {
 public:
  virtual size_t write(uint8_t arg) = 0;
  virtual size_t write(const uint8_t* arg, size_t len) = 0;

  template <typename T>
  void print(T arg) {
    std::stringstream ss;
    ss << arg;
    std::string s = ss.str();
    write(reinterpret_cast<const uint8_t*>(s.c_str()), s.size());
  }
  template <typename T>
  void println(T arg) {
    std::stringstream ss;
    ss << arg << std::endl;
    std::string s = ss.str();
    write(reinterpret_cast<const uint8_t*>(s.c_str()), s.size());
  }
  void println() {
    write('\n');
  }
};

class StringPrinter : public Print {
 public:
  std::stringstream& getStream() { return ss; }

  virtual size_t write(uint8_t arg) {
    ss << arg;
    return 1;
  }
  virtual size_t write(const uint8_t* arg, size_t len) {
    for (size_t i = 0; i < len; ++i) ss << arg[i];
    return len;
  }

 private:
  std::stringstream ss;
};

class HardwareSerial : public Print {};

#endif
