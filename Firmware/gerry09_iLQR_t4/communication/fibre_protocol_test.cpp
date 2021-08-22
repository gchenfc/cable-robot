#ifndef ARDUINO

#include "communication/fibre_protocol.h"

#include "../CppUnitLite/TestHarness.h"

#include "arduino_test_utils.h"

TEST(fibre_protocol, testjsoncrc) {
  StringPrinter serial;
  sendMsg<odrive::JSON>(4, static_cast<uint32_t>(-1), &serial);
  auto& actual = serial.getStream();
  uint8_t expected[] = {
      0xAA, 0x0C, 0xE1,        // stream header
      0x81, 0x00,              // packet seq
      0x00, 0x80,              // endpoint
      0x04, 0x00,              // response size
      0xFF, 0xFF, 0xFF, 0xFF,  // payload
      0x01, 0x00,              // json crc (special case for requesting json)
      0xA9, 0x61,              // packet crc
  };
  LONGS_EQUAL(sizeof(expected), actual.str().size());
  for (size_t i = 0; i < sizeof(expected); ++i) {
    EXPECT_LONGS_EQUAL(expected[i], actual.get());
  }
}

TEST(fibre_protocol, testjsoncontent) {
  StringPrinter serial;
  sendMsg<odrive::JSON>(30, static_cast<uint32_t>(1234), &serial);
  auto& actual = serial.getStream();
  uint8_t expected[] = {
      0xAA, 0x0C, 0xE1,        // stream header
      0x81, 0x00,              // packet seq
      0x00, 0x80,              // endpoint
      0x1E, 0x00,              // response size
      0xD2, 0x04, 0x00, 0x00,  // payload (reversed byte order)
      0x01, 0x00,              // json crc (special case for requesting json)
      0x24, 0x5B,              // packet crc
  };
  LONGS_EQUAL(sizeof(expected), actual.str().size());
  for (size_t i = 0; i < sizeof(expected); ++i) {
    EXPECT_LONGS_EQUAL(expected[i], actual.get());
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
