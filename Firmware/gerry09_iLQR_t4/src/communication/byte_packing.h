/**
 * This file provides utilities for reading/writing data types into/out of byte
 * arrays.
 * Pay attention to endian-ness (le = little endian, be = big endian).
 * This is tested on a Teensy 4.x but if, hypothetically, it were to run on a
 * different MCU, then you would need to make sure the endianness is correct.
 */

#pragma once

#include <limits>

#include "../arduino_test_utils.h"

// Temporary object for reversing a byte array
uint8_t backwards_buf[1000];

/// Reads

template <typename T>
bool read_le(const uint8_t buf[], const size_t buf_len, T& result) {
  if (buf_len < sizeof(T)) return false;
  std::memcpy(&result, buf, sizeof(T));
  return true;
}

template <typename T>
bool read_be(const uint8_t buf[], const size_t buf_len, T& result) {
  if (buf_len < sizeof(T)) return false;
  std::memcpy(backwards_buf, buf, sizeof(T));
  std::reverse(backwards_buf + 0, backwards_buf + sizeof(T));
  std::memcpy(&result, backwards_buf, sizeof(T));
  return true;
}

template <>
bool read_le<float>(const uint8_t buf[], const size_t buf_len, float& value) {
  static_assert(sizeof(float) == 4, "32 bit floating point expected");
  static_assert(std::numeric_limits<float>::is_iec559,
                "IEEE 754 floating point expected");
  return read_le(buf, buf_len, *reinterpret_cast<uint32_t*>(&value));
}

template <>
bool read_be<float>(const uint8_t buf[], const size_t buf_len, float& value) {
  static_assert(sizeof(float) == 4, "32 bit floating point expected");
  static_assert(std::numeric_limits<float>::is_iec559,
                "IEEE 754 floating point expected");
  return read_be(buf, buf_len, *reinterpret_cast<uint32_t*>(&value));
}

template <typename T>
T read_le_unsafe(const uint8_t buf[], const size_t buf_len) {
  T result;
  read_le(buf, buf_len, result);
  return result;
}

template <typename T>
T read_be_unsafe(const uint8_t buf[], const size_t buf_len) {
  T result;
  read_be(buf, buf_len, result);
  return result;
}

/// Writes

template <typename T>
bool write_le(T input, uint8_t buf[], const size_t buf_len) {
  if (buf_len < sizeof(T)) return false;
  std::memcpy(buf, &input, sizeof(T));
  return true;
}

template <typename T>
bool write_be(T input, uint8_t buf[], const size_t buf_len) {
  if (!write_le(input, backwards_buf, buf_len)) {
    return false;
  }
  std::reverse(backwards_buf + 0, backwards_buf + sizeof(T));
  std::memcpy(buf, backwards_buf, sizeof(T));
  return true;
}

template <>
bool write_le<float>(float input, uint8_t buf[], const size_t buf_len) {
  static_assert(sizeof(float) == 4, "32 bit floating point expected");
  static_assert(std::numeric_limits<float>::is_iec559,
                "IEEE 754 floating point expected");
  return write_le(*reinterpret_cast<uint32_t*>(&input), buf, buf_len);
}

template <>
bool write_be<float>(float input, uint8_t buf[], const size_t buf_len) {
  static_assert(sizeof(float) == 4, "32 bit floating point expected");
  static_assert(std::numeric_limits<float>::is_iec559,
                "IEEE 754 floating point expected");
  return write_be(*reinterpret_cast<uint32_t*>(&input), buf, buf_len);
}
