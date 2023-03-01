#pragma once

#include <iostream>

#include "../src/Vector.h"

template <size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<N>& x) {
  for (auto v : x) os << v << ", ";
  return os;
}
