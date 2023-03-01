#include "arduino_test_utils.h"

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using Clock = std::chrono::high_resolution_clock;
Clock::time_point t_start = Clock::now();
uint64_t manual_micros = 0;
uint64_t millis() {
  return (manual_micros / 1000)
             ? (manual_micros / 1000)
             : duration_cast<milliseconds>(Clock::now() - t_start).count();
}
uint64_t micros() {
  return manual_micros
             ? manual_micros
             : (duration_cast<microseconds>(Clock::now() - t_start).count());
}
void set_time_us(uint64_t t_us) { manual_micros = t_us; }
uint64_t& getMicros() { return manual_micros; }
