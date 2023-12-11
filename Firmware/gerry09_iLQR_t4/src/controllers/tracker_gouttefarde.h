#pragma once

#include "../communication/ascii_parser.h"
#include "tracker_safe.h"
#include "../pid.h"
#include "../kinematics.h"

/******** DEFAULT GAINS AND PARAMETERS **********/
#if defined(KLAUS) || defined(DFL)
static constexpr float kKp = 15000, kKi = 15, kKd = 30,  // gains
    kIntegratorLimit = 10,                               // integrator limit
    kTMin_N = 8, kTMid_N = 54, kTMax_N = 100,            // tensions
    kEeMass_kg = 3;                                      // end effector mass
#endif
#ifdef AIR
static constexpr float kKp = 10000, kKi = 0, kKd = 500,  // gains
    kIntegratorLimit = 0,                                // integrator limit
    kTMin_N = 16, kTMid_N = 150, kTMax_N = 300,          // tensions
    kEeMass_kg = 3;                                      // end effector mass
#endif
#ifdef HYDROPONICS
static constexpr float kKp = 30000, kKi = 50, kKd = 50,  // gains
    kIntegratorLimit = 10,                               // integrator limit
    kTMin_N = 5, kTMid_N = 50, kTMax_N = 140,            // tensions
    kEeMass_kg = 3;                                      // end effector mass
#endif
/******** END DEFAULT GAINS AND PARAMETERS **********/

/**
 * TrackerGouttefarde implements using Gouttefarde's dual-space FF controller.
 * Uses Pott's tension distribution algorithm.
 */
class TrackerGouttefarde : public TrackerSafe {
 public:
  TrackerGouttefarde(Robot& robot, SetpointInterface* setpoint,
                     StateEstimatorInterface* state_estimator)
      : TrackerSafe(robot, setpoint, state_estimator), kinematics_(robot) {}

  static void print_name(Stream& serialOut) {
    serialOut.print("TrackerGouttefarde");
  }

  virtual bool readSerial(AsciiParser parser, Stream& serial_out) override;
  virtual float calcTension_N(uint8_t winchnum) override;
  virtual bool initialize() override;  // reset PIDs

 protected:
  Kinematics kinematics_;
  float fs_ = 0., fv_ = 0., mu_ = 0.;
  float kp_ = kKp, ki_ = kKi, kd_ = kKd, integratorLimit_ = kIntegratorLimit;
  float minTension_ = kTMin_N, midTension_ = kTMid_N, maxTension_ = kTMax_N;
  float eeMass_kg_ = kEeMass_kg;
  Pid pid_[4]{Pid(kp_, ki_, kd_, integratorLimit_),
              Pid(kp_, ki_, kd_, integratorLimit_),
              Pid(kp_, ki_, kd_, integratorLimit_),
              Pid(kp_, ki_, kd_, integratorLimit_)};

  void setKp(float kp);
  void setKi(float ki);
  void setKd(float kd);
  void setIntegratorLimit(float integratorLimit);
};

bool TrackerGouttefarde::readSerial(AsciiParser parser, Stream& serial_out) {
  if (TrackerSafe::readSerial(parser, serial_out)) return true;
  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::TRACKING));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  if (cmd == TrackingCommands::SET_GAINS) {
    AsciiParser parser_gains = parser;
    UNWRAP_PARSE_CHECK(char c, parser_gains.getChar(&c));
    UNWRAP_PARSE_CHECK(float a, parser_gains.parseFloat('\n', &a));
    switch (c) {
      case 'p':
        setKp(a);
        return true;
      case 'i':
        setKi(a);
        return true;
      case 'd':
        setKd(a);
        return true;
      case 'w':  // prevent integrator windup
        setIntegratorLimit(a);
        return true;
      case 's':
        fs_ = a;
        return true;
      case 'v':
        fv_ = a;
        return true;
      case 'u':
        mu_ = a;
        return true;
      case 'm':
        midTension_ = a;
        return true;
      case 'M':
        maxTension_ = a;
        return true;
      case 'L':
        minTension_ = a;
        return true;
      case 'g':
        eeMass_kg_ = a;
        return true;
    }
  }
  return false;
}

void TrackerGouttefarde::setKp(float kp) {
  kp_ = kp;
  for (Pid& pid : pid_) {
    pid.setKp(kp);
  }
}
void TrackerGouttefarde::setKi(float ki) {
  ki_ = ki;
  for (Pid& pid : pid_) {
    pid.setKi(ki);
  }
}
void TrackerGouttefarde::setKd(float kd) {
  kd_ = kd;
  for (Pid& pid : pid_) {
    pid.setKd(kd);
  }
}
void TrackerGouttefarde::setIntegratorLimit(float integratorLimit) {
  integratorLimit_ = integratorLimit;
  for (Pid& pid : pid_) {
    pid.setIntegratorLimit(integratorLimit);
  }
}

bool TrackerGouttefarde::initialize() {
  std::for_each(std::begin(pid_), std::end(pid_),
                [](Pid& pid) { pid.reset(); });
  return true;
}

float TrackerGouttefarde::calcTension_N(uint8_t winchnum) {
  // TODO: switch this over to `Vector` interface
  if (winchnum >= 4) return 0.0f;
  // Setpoint
  const auto& xdes_ = setpointPos();
  const auto& vdes_ = setpointVel();
  const float xdes[2] = {std::get<0>(xdes_), std::get<1>(xdes_)};
  const float vdes[2] = {std::get<0>(vdes_), std::get<1>(vdes_)};

  // SerialD.printf("xdes = %.3f %.3f, vdes = %.3f %.3f\n", xdes[0], xdes[1],
  //                vdes[0], vdes[1]);

  // IK
  float ldes[4];
  kinematics_.IK(xdes[0], xdes[1], ldes);
  // const float(&lff)[4] = LQG_GAINS[k].lff;
  // SerialD.printf("ldes = %.3f %.3f %.3f %.3f, lff = %.3f %.3f %.3f %.3f\n",
  //                ldes[0], ldes[1], ldes[2], ldes[3], lff[0], lff[1], lff[2],
  //                lff[3]);

  // float feedback_torque_Nm[4];
  float feedback_tension_N[4];
  for (int i = 0; i < 4; ++i) {
    float lerr = ldes[i] - robot_.len(i);
    feedback_tension_N[i] = pid_[i].update(lerr);
  }
  // save_pid_output(feedback_tension_N);

  // Convert feedback tension to task-space force
  float W[2][4];
  kinematics_.wrenchMatrix(W);
  float feedback_force_N[2];
  matmul(W, feedback_tension_N, feedback_force_N);
  // save_feedback_force(feedback_force_N);

  // feedforward force
  // TODO(gerry): SE(2)
  float ff_force_N[2];
  // TODO(gerry): save previous vdes
  // const float(&vdes_prev)[3] = LQG_GAINS[k == 0 ? k : k - 1].vff;
  // static constexpr float kMass = 1.0;
  ff_force_N[0] = 0;  //(vdes[0] - vdes_prev[0]) / dt * kMass;
  // ff_force_N[1] = 0;  //(vdes[1] - vdes_prev[1]) / dt * kMass;
  ff_force_N[1] = eeMass_kg_ * 9.8;  //(vdes[1] - vdes_prev[1]) / dt * kMass;
  // TODO: use setpointAcc()
  float fc_N[2];
  matadd(feedback_force_N, ff_force_N, fc_N);
  // save_total_force(fc_N);

  // Tension distribution
  // TODO(gerry): use better tension distribution algorithm
  float tensionPid_N[4];
  kinematics_.forceSolverPott(fc_N[0], fc_N[1], W, tensionPid_N, midTension_);
  // SerialD.printf("Tension Distribution: %.3f %.3f %.3f %.3f\n",
  //                tensionPid_N[0],
  //                tensionPid_N[1], tensionPid_N[2],
  //                tensionPid_N[3]);

  // Feedforward tensions
  // The feedforward tensions are assumed to be primarily a function of the
  // motor, not the winch/drum, so the feedforward tensions are computed without
  // cableLengthCorrectionParams
  float ldotdes[4];
  float J[4][2];
  kinematics_.jacobian(J);
  matmul(J, vdes, ldotdes);
  float tensionFf_N[4], tension_N[4];
  for (int i = 0; i < 4; ++i) {
    // TODO: add rotor inertia (need lddot)
    auto tauFf_Nm = (-ldotdes[i] * fv_ - tanh(mu_ * ldotdes[i]) * fs_) * kR;
    tensionFf_N[i] = robot_.winches.at(i).tension_N(tauFf_Nm);
  }
  matadd(tensionPid_N, tensionFf_N, tension_N);
  // save_torque(tensionFf_N);

  // Safety & return
  float tension = tension_N[winchnum];
  clamp(&tension, minTension_, maxTension_);
  return tension;
}
