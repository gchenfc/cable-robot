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
#include <chrono>

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

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using Clock = std::chrono::high_resolution_clock;
auto t_start = Clock::now();
uint64_t manual_millis = 0, manual_micros = 0;
uint64_t millis() {
  return manual_millis
             ? manual_millis
             : duration_cast<milliseconds>(Clock::now() - t_start).count();
}
uint64_t micros() {
  return manual_micros
             ? manual_micros
             : (duration_cast<microseconds>(Clock::now() - t_start).count());
}

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
  void println() { write('\n'); }
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
