#ifndef ARDUINO

#include <iostream>
#include <algorithm>
#include "ascii_parser.h"

#include "../../CppUnitLite/TestHarness.h"

char begin[] = "ca1.2,9,n\n";
const AsciiParser parser_(begin, 10);
char c, a, n;
float f;
int i, i2;

TEST(ascii_parser, test_manual) {
  AsciiParser parser = parser_;

  CHECK(parser.checkChar('c'));
  CHECK(parser.getChar(&a));
  CHECK_EQUAL('a', a);
  CHECK(parser.parseFloat(&f));
  CHECK_EQUAL(1.2, f);
  CHECK(parser.parseInt(&i));
  CHECK_EQUAL(9, i);
  CHECK(parser.checkChar('n'));
  CHECK(parser.checkDone());
  EXPECT_LONGS_EQUAL('a', a);
  EXPECT_DOUBLES_EQUAL(1.2, f, 1e-4);
  EXPECT_LONGS_EQUAL(9, i);
  EXPECT_LONGS_EQUAL('n', n);
}

TEST(AsciiParser, test_template_success) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK(parser.parse(&a, &f, &i, &n));
  CHECK_EQUAL('a', a)
  CHECK_EQUAL(1.2, f)
  CHECK_EQUAL(9, i)
  CHECK_EQUAL('n', n)
}
TEST(AsciiParser, test_template_int_as_char_fail) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK(!parser.parse(&i2, &f, &i, &n));
}
TEST(AsciiParser, test_template_premature_fail) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK(!parser.parse(&a, &f, &i));
}
TEST(AsciiParser, test_template_extra_char) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK(!parser.parse(&a, &f, &i, &n, &i2));
}

TEST(AsciiParser, test_peek) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK(!parser.peek(&a, &f, &i, &n, &i2));
  CHECK(!parser.peek(&a, &f, &i));
  CHECK(parser.peek(&a, &f, &i, &n));
  EXPECT_LONGS_EQUAL('a', a);
  EXPECT_DOUBLES_EQUAL(1.2, f, 1e-4);
  EXPECT_LONGS_EQUAL(9, i);
  EXPECT_LONGS_EQUAL('n', n);
  CHECK(parser.parse(&a, &f, &i, &n));
  EXPECT_LONGS_EQUAL('a', a);
  EXPECT_DOUBLES_EQUAL(1.2, f, 1e-4);
  EXPECT_LONGS_EQUAL(9, i);
  EXPECT_LONGS_EQUAL('n', n);
}

TEST(AsciiParser, test_peek2) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK((!parser.peek<char, float, int, char, int>()));
  CHECK((!parser.peek<char, float, int>()));
  CHECK((parser.peek<char, float, int, char>()));
  CHECK((parser.parse(&a, &f, &i, &n)));
  EXPECT_LONGS_EQUAL('a', a);
  EXPECT_DOUBLES_EQUAL(1.2, f, 1e-4);
  EXPECT_LONGS_EQUAL(9, i);
  EXPECT_LONGS_EQUAL('n', n);
}

TEST(AsciiParser, aoeu) {
  char begin[] = "gs1\n";
  AsciiParser parser(begin, 10);

  if (parser.checkChar('g') && parser.checkChar('s')) {
    uint8_t num;
    EXPECT(parser.parseInt('\n', &num));
    EXPECT_LONGS_EQUAL(1, num);
    return;
  }
  EXPECT(false);
}

TEST(AsciiParser, sprayTest) {
  char s0[4] = "s0\n";
  char s1[4] = "s1\n";
  char sother[5] = "s01\n";
  auto sprayCheck = [](AsciiParser parser) {
    if (!parser.checkChar('s')) return -1;
    AsciiParser parser0 = parser;
    if (parser0.checkChar('0') && parser0.checkChar('\n')) return 0;
    AsciiParser parser1 = parser;
    if (parser1.checkChar('1') && parser1.checkChar('\n')) return 1;
    return 2;
  };
  EXPECT_LONGS_EQUAL(0, sprayCheck(AsciiParser(s0, 4)));
  EXPECT_LONGS_EQUAL(1, sprayCheck(AsciiParser(s1, 4)));
  EXPECT_LONGS_EQUAL(2, sprayCheck(AsciiParser(sother, 5)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
