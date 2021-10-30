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
inline T min(const T& t1, const T& t2) {
  return std::min(t1, t2);
}
template <class T>
inline T max(const T& t1, const T& t2) {
  return std::max(t1, t2);
}

template <class T>
inline constexpr T sqrt(const T& t) {
  return std::sqrt(t);
}

uint64_t millis();
uint64_t micros();
void set_time_us(uint64_t t_us);
uint64_t& getMicros();

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

  int printf(const char* format, ...) {
    char buf[1024];
    va_list args;
    va_start(args, format);
    int ret = vsprintf(buf, format, args);
    va_end(args);
    print(buf);
    return ret;
  }
};

class StringPrinter : public virtual Print {
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

class Stream : public virtual Print {
 public:
  // There are supposed to be lots more functions but we don't use them
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
};

// diamond inheritance pattern... hopefully it's fine...
class StringStreamer : public Stream, public StringPrinter {
 public:
  template <typename T>
  StringStreamer& operator<<(T arg) {
    os << arg;
    return *this;
  }
  int available() override { return os.str().length(); };
  int read() override {
    auto toret = os.str().at(0);
    os = std::ostringstream{os.str().substr(1)};
    return toret;
  }
  int peek() override { return os.rdbuf()->sgetc(); }

 private:
  std::ostringstream os;
};

class HardwareSerial : public Print {};

class Eeprom {
 public:
  template <typename T>
  void put(int addr, T data) {
    std::cout << "Saved to EEPROM address " << addr << data << std::endl;
  }
  template <typename T>
  void get(int addr, T& data) {
    std::cout << "Got from EEPROM address " << addr << data << std::endl;
  }
};
extern Eeprom EEPROM;

#endif
