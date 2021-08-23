/**
 * Test can_utils.h
 * Connect Teensy to an ODrive on CAN1.
 *
 * Will only periodically print heartbeat and endcoder messages.  Will respond
 * with torque command & vbus query on encoder message reception.
 */

#include <Metro.h>

#include "can_simple.h"
#include "can_utils.h"

CanUtils<CAN1> Can0;

Metro timer_print_info(200);

template <typename T>
struct Statistics;
struct NodeInfo;
void parsePacket(CAN_message_t inMsg);
void printInfo(bool reset_nodes = true);

// -------------------------------------------------------------
void setup(void) {
  while (!Serial)  // Wait for serial monitor to come up
    ;
  Serial.println("begin can_utils test");

  Can0.begin();
  Can0.setBaudRate(1000000);
}

// -------------------------------------------------------------
void loop(void) {
  static CAN_message_t inMsg;
  while (Can0.read(inMsg)) {
    parsePacket(inMsg);
  }

  if (timer_print_info.check()) {
    printInfo();
  }
}

// -------------------------------------------------------------
template <typename T>
struct Statistics {
  T sum_x, sum_x2;
  uint64_t samples;
  void add(T sample) {
    sum_x += sample;
    sum_x2 += sample * sample;
    ++samples;
  }
  double mean() const {
    return samples ? (static_cast<double>(sum_x) / samples) : -1;
  }
  double std() const {
    if (samples == 0) return -1;
    const double m = mean();
    const double var = (static_cast<double>(sum_x2) / samples) - (m * m);
    return sqrt(var);
  }
};
struct NodeInfo {
  template <typename T>
  using Time_Message = std::pair<uint64_t, T>;
  Time_Message<std::pair<uint32_t, uint32_t>> last_heartbeat;
  Time_Message<std::pair<float, float>> last_encoder;
  Time_Message<float> last_query_response;
  Statistics<uint64_t> stats_heartbeat, stats_encoder, stats_query_response;
  void reset_stats() {
    stats_heartbeat = {};
    stats_encoder = {};
    stats_query_response = {};
  }
};

// -------------------------------------------------------------
NodeInfo nodes[4];

// -------------------------------------------------------------
void parsePacket(CAN_message_t inMsg) {
  uint8_t nodei = inMsg.id >> 5;
  uint8_t cmd = inMsg.id & 0b11111;
  NodeInfo& node = nodes[nodei];
  uint64_t time = micros();
  switch (cmd) {
    case MSG_ODRIVE_HEARTBEAT:
      if (node.last_heartbeat.first != 0) {
        node.stats_heartbeat.add(time - node.last_heartbeat.first);
      }
      node.last_heartbeat = {time, Can0.parseInt32s(inMsg.buf)};
      break;
    case MSG_GET_ENCODER_ESTIMATES:
      if (node.last_encoder.first != 0) {
        node.stats_encoder.add(time - node.last_encoder.first);
      }
      node.last_encoder = {time, Can0.parseFloats(inMsg.buf)};
      Can0.send(nodei, MSG_SET_INPUT_TORQUE,
                static_cast<float>(millis() / 100.0));
      Can0.requestInfo(nodei, MSG_GET_VBUS_VOLTAGE, true);
      break;
    case MSG_GET_VBUS_VOLTAGE:
      node.stats_query_response.add(time - node.last_encoder.first);
      // Serial.println(time - node.last_encoder.first);
      node.last_query_response = {time, Can0.parseFloat(inMsg.buf)};
      break;
    default: {
      Serial.print("CAN bus 0:\tnode=");
      Serial.print(nodei);
      Serial.print("\tcmd=");
      Serial.print(cmd, HEX);
      Serial.print("\t\tdata = ");
      Serial.print(Can0.parseInt32(inMsg.buf));
      Serial.print('\t');
      Serial.print(Can0.parseInt32(inMsg.buf + 4));
      Serial.print('\t');
      Serial.print(Can0.parseFloat(inMsg.buf));
      Serial.print('\t');
      Serial.println(Can0.parseFloat((inMsg.buf + 4)));
      break;
    }
  }
}

// -------------------------------------------------------------
template <typename T>
void printStats(const char* fmt, Statistics<T> stats,
                double unit_multiplier = 1) {
  Serial.printf(fmt, "", stats.mean() * unit_multiplier,
                stats.std() * unit_multiplier, stats.samples);
};
void printInfo(bool reset_nodes) {
  //            0123456789012345678901234567890
  Serial.printf("Response Time (us):   mean   std count  |  ");
  Serial.printf("Heartbeat Period (ms):   mean   std count  |  ");
  Serial.printf("Encoder Period (ms):   mean   std count  |  ");
  Serial.printf("Last: error state   pos   vel  vbus\n");
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.printf("Node %d", i);
    const NodeInfo& node = nodes[i];
    printStats("%13s %6.1f %5.2f %5d  |  ", node.stats_query_response);
    printStats("%22s %6.1f %5.2f %5d  |  ", node.stats_heartbeat, 0.001);
    printStats("%20s %6.1f %5.2f %5d  |  ", node.stats_encoder, 0.001);
    Serial.printf(
        "%5s %5d %5d %5.1f %5.2f %5.2f\n", "",  //
        node.last_heartbeat.second.first, node.last_heartbeat.second.second,
        node.last_encoder.second.first, node.last_encoder.second.second,
        node.last_query_response.second);
    nodes[i].reset_stats();
  }
}
