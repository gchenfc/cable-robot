#pragma once

namespace human_serial {

bool until(char** buffer_start, char* buffer_end, char delim = '\255') {
  if (*buffer_start == buffer_end) return false;
  if (**buffer_start == delim) return false;
  if (delim == '\n' && (std::find(*buffer_start, buffer_end, '\n') >
                        std::find(*buffer_start, buffer_end, ','))) {
    return false;
  }
  if (delim == '\255') {
    if ((**buffer_start == '\n') || (**buffer_start == ',')) return false;
    *buffer_start = std::min(std::find(*buffer_start, buffer_end, '\n'),
                             std::find(*buffer_start, buffer_end, ','));
  } else {
    *buffer_start = std::find(*buffer_start, buffer_end, delim);
  }
  if (*buffer_start == buffer_end) return false;
  *((*buffer_start)++) = 0;  // null terminate the number and advance to next
  return true;
}
template <typename T>
bool parseInt(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atoi(original_start);
  *(*buffer_start - 1) = delim;
  return true;
}
template <typename T>
bool parseFloat(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atof(original_start);
  *(*buffer_start - 1) = delim;
  return true;
}

}  // namespace human_serial

#define UNWRAP_PARSE_CHECK(declaration, expression) \
  declaration;                                      \
  if (!expression) return false;
struct AsciiParser {
  char *buffer_cur_, *buffer_end_;

 public:
  AsciiParser(char* buffer, int size)
      : buffer_cur_(buffer), buffer_end_(buffer + size) {}

  /// Checks that there is nothing left to parse in this string
  bool checkDone() const { return (buffer_cur_ == buffer_end_); }

  /// Asserts that next character matches argument
  bool checkChar(char expected) {
    return (buffer_cur_ != buffer_end_) && ((buffer_cur_++)[0] == expected);
  }
  /// Reads the next character
  bool getChar(char* ret) {
    if (buffer_cur_ == buffer_end_) return false;
    *ret = (buffer_cur_++)[0];
    return true;
  }
  /// If the character matches, then advance to next.  Else, do nothing
  bool advanceOnMatchChar(char expected) {
    if (buffer_cur_ == buffer_end_) return false;
    if (*buffer_cur_ == expected) buffer_cur_++;
    return true;
  }

  /// Parses an int, ending in delimiter
  template <typename T = int>
  bool parseInt(char delimiter, T* ret) {
    return human_serial::parseInt(&buffer_cur_, buffer_end_, delimiter, ret);
  }
  /// Parses an int, ending in either ',' or '\n'
  template <typename T = int>
  bool parseInt(T* ret) {
    return human_serial::parseInt(&buffer_cur_, buffer_end_, '\255', ret);
  }

  /// Parses a float, ending in delimiter
  template <typename T = float>
  bool parseFloat(char delimiter, T* ret) {
    return human_serial::parseFloat(&buffer_cur_, buffer_end_, delimiter, ret);
  }
  /// Parses a float, ending in ',' or '\n'
  template <typename T = float>
  bool parseFloat(T* ret) {
    return human_serial::parseFloat(&buffer_cur_, buffer_end_, '\255', ret);
  }

  bool parse() { return checkDone(); }
#define COMMA ,
#define ENABLE_IF(expr) std::enable_if_t<expr, bool> = true
  template <typename T, ENABLE_IF(std::is_same<char COMMA T>::value)>
  bool parse(T* ret, char delimiter = '\n') {
    return getChar(ret) && (delimiter == ',' ? advanceOnMatchChar(delimiter)
                                             : checkChar(delimiter));
  }
  template <typename T, ENABLE_IF(std::is_integral<T>::value),
            ENABLE_IF(!std::is_same<char COMMA T>::value)>
  bool parse(T* ret, char delimiter = '\n') {
    return parseInt(delimiter, ret);
  }
  template <typename T, ENABLE_IF(std::is_floating_point<T>::value)>
  bool parse(T* ret, char delimiter = '\n') {
    return parseFloat(delimiter, ret);
  }
  template <typename T1, typename... T>
  bool parse(T1* ret, T*... otherRets) {
    return parse(ret, ',') && parse(otherRets...);
  }

  template <typename... T>
  bool peek(T*... ret) {
    AsciiParser parser(*this);
    return parser.parse(ret...);
  }
};
