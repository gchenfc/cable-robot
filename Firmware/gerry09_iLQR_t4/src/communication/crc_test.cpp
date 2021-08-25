#ifndef ARDUINO

#include "crc.h"

#include "../../CppUnitLite/TestHarness.h"

TEST(crc16, test1234) {
  const uint8_t buf[4] = {0x01, 0x02, 0x03, 0x04};
  uint16_t expected = 0x672E;
  uint16_t actual = crc16(buf, 4);
  EXPECT_LONGS_EQUAL(expected, actual);
}

TEST(crc16, test54321) {
  const uint8_t buf[5] = {0x05, 0x04, 0x03, 0x02, 0x01};
  uint16_t expected = 0xE251;
  uint16_t actual = crc16(buf, 5);
  EXPECT_LONGS_EQUAL(expected, actual);
}

TEST(crc8, test1234) {
  const uint8_t buf[4] = {0x01, 0x02, 0x03, 0x04};
  uint8_t expected = 0x61;
  uint8_t actual = crc8(buf, 4);
  EXPECT_LONGS_EQUAL(expected, actual);
}

TEST(crc8, test54321) {
  const uint8_t buf[5] = {0x05, 0x04, 0x03, 0x02, 0x01};
  uint8_t expected = 0x64;
  uint8_t actual = crc8(buf, 5);
  EXPECT_LONGS_EQUAL(expected, actual);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
