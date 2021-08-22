#ifndef ARDUINO

/**
 * This test must be run on the real MCU hardware to make sure the endianness
 * is correct.
 */
#include "communication/byte_packing.h"

#include "../CppUnitLite/TestHarness.h"

#define testWriteLe(T, input, buf_expected, buf_len) \
  {                                                  \
    uint8_t buf[100];                                \
    EXPECT(write_le<T>(input, buf, buf_len));        \
    for (size_t i = 0; i < buf_len; ++i) {           \
      EXPECT_LONGS_EQUAL(buf_expected[i], buf[i]);   \
    }                                                \
  }
#define testWriteBe(T, input, buf_expected, buf_len) \
  {                                                  \
    uint8_t buf[100];                                \
    EXPECT(write_be<T>(input, buf, buf_len));        \
    for (size_t i = 0; i < buf_len; ++i) {           \
      EXPECT_LONGS_EQUAL(buf_expected[i], buf[i]);   \
    }                                                \
  }

TEST(little_endian, test12345678) {
  const uint8_t buf[4] = {0x12, 0x34, 0x56, 0x78};
  const uint8_t* buf2 = buf + 2;

  EXPECT_LONGS_EQUAL(2018915346L, read_le_unsafe<uint32_t>(buf, 4));
  EXPECT_LONGS_EQUAL(2018915346L, read_le_unsafe<int32_t>(buf, 4));
  EXPECT_LONGS_EQUAL(13330, read_le_unsafe<uint16_t>(buf, 4));
  EXPECT_LONGS_EQUAL(30806, read_le_unsafe<uint16_t>(buf2, 2));
  EXPECT_DOUBLES_EQUAL(1.73782444e+34f, read_le_unsafe<float>(buf, 4), 0);
  testWriteLe(uint32_t, 2018915346L, buf, 4);
  testWriteLe(int32_t, 2018915346L, buf, 4);
  testWriteLe(uint16_t, 13330, buf, 2);
  testWriteLe(uint16_t, 30806, buf2, 2);
  testWriteLe(float, 1.73782444e+34f, buf, 4);
}

TEST(big_endian, test12345678) {
  const uint8_t buf[4] = {0x12, 0x34, 0x56, 0x78};
  const uint8_t* buf2 = buf + 2;

  EXPECT_LONGS_EQUAL(305419896L, read_be_unsafe<uint32_t>(buf, 4));
  EXPECT_LONGS_EQUAL(305419896L, read_be_unsafe<int32_t>(buf, 4));
  EXPECT_LONGS_EQUAL(4660, read_be_unsafe<uint16_t>(buf, 4));
  EXPECT_LONGS_EQUAL(22136, read_be_unsafe<uint16_t>(buf2, 2));
  EXPECT_DOUBLES_EQUAL(5.69045661e-28f, read_be_unsafe<float>(buf, 4), 0);
  testWriteBe(uint32_t, 305419896L, buf, 4);
  testWriteBe(int32_t, 305419896L, buf, 4);
  testWriteBe(uint16_t, 4660, buf, 2);
  testWriteBe(uint16_t, 22136, buf2, 2);
  testWriteBe(float, 5.69045661e-28f, buf, 4);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
