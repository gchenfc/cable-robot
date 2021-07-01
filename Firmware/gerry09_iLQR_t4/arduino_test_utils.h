#pragma once

#include <algorithm>
#include <cmath>

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
