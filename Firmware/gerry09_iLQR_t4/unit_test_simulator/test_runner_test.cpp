#include "test_runner.h"
#include "../src/communication/can_simple.h"

union Data {
  uint8_t buffer[8];
  std::pair<int32_t, int32_t> ints;
  std::pair<float, float> floats;
};

bool callback(CableRobotTester::Message msg) {
  uint8_t a = std::get<0>(msg);
  uint8_t b = std::get<1>(msg);
  std::array<uint8_t, 8> c = std::get<2>(msg);
  bool d = std::get<3>(msg);

  Data data{{0}};
  std::memcpy(data.buffer, c.begin(), 8);

  std::cout << "Got CAN message: node: " << std::to_string(a)         //
            << ",\tcmd: " << std::to_string(b)                        //
            << ",\tas ints: " << std::to_string(data.ints.first)      //
            << ",\t" << std::to_string(data.ints.second)              //
            << ",\tas floats: " << std::to_string(data.floats.first)  //
            << ",\t" << std::to_string(data.floats.second)            //
            << ",\trtr? " << std::to_string(d)                        //
            << std::endl;
  return true;
};

void sendEncoderForXY(CableRobotTester& tester, float x, float y, float vx = 0,
                      float vy = 0) {
  float v[2][1] = {{vx}, {vy}};
  // kinematics
  float lengths[4], vels[4][1], wrench_matrix[4][2];
  Kinematics::IK(x, y, lengths);
  Kinematics::jacobian(x, y, wrench_matrix);
  matmul<4, 2, 1>(wrench_matrix, v, vels);

  // convert lengths to motor rotations
  float thetas_rot[4], omegas_rps[4];
  std::transform(lengths, lengths + 4, kZeros, thetas_rot,
                 [](float l0, float z) { return l0 / (2 * M_PI * kR) + z; });
  std::transform(*vels, *vels + 4, omegas_rps,
                 [](float v) { return v / (2 * M_PI * kR); });

  for (int winchi = 0; winchi < 4; ++winchi) {
    static std::array<uint8_t, 8> data;
    std::memcpy(data.begin(), &thetas_rot[winchi], 4);
    std::memcpy(data.begin() + 4, &omegas_rps[winchi], 4);
    tester.addMsgToQueue({winchi, MSG_GET_ENCODER_ESTIMATES, data, false});
  }
}

float actual_x(float t) { return 1.6 + 0.5 * cosf(t * M_PI / 5); }
float actual_y(float t) { return 1.2 + 0.5 * sinf(t * M_PI / 5); }

int main() {
  CableRobotTester tester(&callback, std::cerr);
  auto s = [](uint64_t us) { return us * 1e-6; };
  uint64_t& t_us = getMicros();
  t_us = 0;

  // First we expect torque (id:14) = 0 replies
  tester.update();
  for (; t_us < 100e3; t_us += 10e3) {
    sendEncoderForXY(tester, actual_x(s(t_us)), actual_y(s(t_us)));
    tester.update();
  }

  // Now we put the controller in mode "hold" and expect torque = 0.2 replies
  tester.enterComputerInput("g5\n");
  tester.update();
  for (; t_us < 200e3; t_us += 10e3) {
    sendEncoderForXY(tester, actual_x(s(t_us)), actual_y(s(t_us)));
    tester.update();
  }

  // Finally we put the controller in mode "RUNNING_TRAJ" and expect
  // "interesting" torque replies
  // Specifically, the actual position is set as 0.1 units up and 0.1 units
  // right of the setpoint position
  tester.enterComputerInput("g1\n");
  tester.update();
  for (; t_us < 300e3; t_us += 10e3) {
    sendEncoderForXY(tester, actual_x(s(t_us)), actual_y(s(t_us)));
    tester.update();
  }
}
