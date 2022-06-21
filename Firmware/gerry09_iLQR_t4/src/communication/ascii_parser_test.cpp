#ifndef ARDUINO

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
  CHECK(parser.parse(&a, &f, &i, &n));
}

TEST(AsciiParser, test_peek2) {
  AsciiParser parser = parser_;
  CHECK(parser.checkChar('c'))
  CHECK((!parser.peek<char, float, int, char, int>()));
  CHECK((!parser.peek<char, float, int>()));
  CHECK((parser.peek<char, float, int, char>()));
  CHECK((parser.parse(&a, &f, &i, &n)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
