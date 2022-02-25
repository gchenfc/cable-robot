#pragma once

namespace human_serial {

bool until(char** buffer_start, char* buffer_end, char delim) {
  *buffer_start = std::find(*buffer_start, buffer_end, delim);
  if (*buffer_start == buffer_end) return false;
  *((*buffer_start)++) =
      0;  // null terminate the number and advance to next one
  return true;
}
template <typename T>
bool parseInt(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atoi(original_start);
  return true;
}
template <typename T>
bool parseFloat(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atof(original_start);
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

  bool checkChar(char expected) {
    return (buffer_cur_ != buffer_end_) && ((buffer_cur_++)[0] == expected);
  }

  bool getChar(char* ret) {
    if (buffer_cur_ == buffer_end_) return false;
    *ret = (buffer_cur_++)[0];
    return true;
  }

  template <typename T = int>
  bool parseInt(char delimiter, T* ret) {
    return human_serial::parseInt(&buffer_cur_, buffer_end_, delimiter, ret);
  }

  template <typename T = float>
  bool parseFloat(char delimiter, T* ret) {
    return human_serial::parseFloat(&buffer_cur_, buffer_end_, delimiter, ret);
  }
};
