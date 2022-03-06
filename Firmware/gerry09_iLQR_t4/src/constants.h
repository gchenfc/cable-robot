#pragma once

#define ESTOP 11
#define ESTOP_HIGHx
#define SerialD Serial
#define btSerial Serial5

#ifndef DEBUG_CONSTANTS

/******** Winch Parameters **********/
float kZeros[4] = {4.868, 4.672, 6.490, 7.980};
static constexpr float kR = 0.0254 / 2;
// static constexpr float kSprayDelay_s = 0.40;
static constexpr float kSprayDelay_s = 0.0;

/******** Frame Geometry **********/
// Klaus lab measurements:
//     w        h    aspect ratio     diag
// 0: 2.94  -  2.22  (1.3214 : 1)  -  3.75
// 1: 2.99  -  2.27  (1.3288 : 1)  -  3.79
// 2: 2.95  -  2.24  (1.3158 : 1)  -  3.76
// 3: 2.91  -  2.20  (1.3318 : 1)  -  3.73
static constexpr float kFrameWidth = 2.9475, kFrameHeight = 2.2325;

// DFL Measurements
// 0: 3.05  -  2.26
// 1: 3.12  -  2.35
// 2: 3.14  -  2.38
// 3: 3.05  -  2.29
// float kWidth = 3.09, kHeight = 2.32;
// static constexpr float kFrameWidth = 3.05, kFrameHeight = 2.32;

// Constants including carriage
static constexpr float kCarriageWidth = 0.187, kCarriageHeight = 0.122;
static constexpr float kWidth = kFrameWidth - kCarriageWidth,
                       kHeight = kFrameHeight - kCarriageHeight / 2;

/******** Mathematical Constants **********/
static constexpr float kTau = 2 * 3.1415926535897932384626433832795;

#endif

#define CALIBRATIONx
#ifdef CALIBRATION
static constexpr float kMountPoints[4][2] = {
    {kFrameWidth, 0},
    {kFrameWidth, kFrameHeight},
    {0, kFrameHeight},
    {0, 0},
};
static constexpr float kEEMountPoints[4][2] = {
    {kCarriageWidth, 0},
    {kCarriageWidth, kCarriageHeight},
    {0, kCarriageHeight},
    {0, 0},
};
float lengthCorrection(uint8_t winch_num, float len) { return len; }
#else
static constexpr float kMountPoints[4][2] = {
  {2.863707736755236, 0.04860609352193113},
  {2.8848719151510935, 2.2183350120755874},
  {0.1126533163423053, 2.2185915662589384},
  {0.07143183255518266, 0.04752962540962345},
};
static constexpr float kLenCorrectionParams[4][3] = {
  {0, 0.9435254648255599, 0.07135455558345628},
  {0, 0.9395986722484311, 0.10166151161483318},
  {0, 0.9418143396102304, 0.0726389227849403},
  {0, 0.9342846013260662, 0.11146005401104868},
};
// static constexpr float kMountPoints[4][2] = {
//   {2.8569092469594004, 0.0697609080167824},
//   {2.886881456698374, 2.2426676210652823},
//   {0.07477860789397986, 2.2276860603677737},
//   {0.04247446571291975, 0.0025415302633857136},
// };
// static constexpr float kLenCorrectionParams[4][3] = {
//   {0, 0.9408428639497599, 0.07326092516560527},
//   {0, 0.935451075050366, 0.15083435413733556},
//   {0, 0.9394487770575536, 0.10466303224837221},
//   {0, 0.9292022987799741, 0.18893707730145656},
// };
// static constexpr float kLenCorrectionParams[4][3] = {
//     {0.00000000000000, 1.00000000000000, 0.00000000000000},
//     {-0.01188844089481, 0.92855624871954, 0.08171694405092},
//     {-0.01133362762128, 0.95572779537438, -0.04781298818415},
//     {0.00000000000000, 1.00000000000000, 0.00000000000000},
// };
// static constexpr float kMountPoints[4][2] = {
//     {kWidth, 0}, {2.87172620453055, 2.33246622786011}, {0.16871802351569, 2.28522716180476}, {0, 0}};
// static constexpr float kMountPoints[4][2] = {
//     {kWidth, 0},
//     {2.84039031486005, 2.28342877752207},
//     {0.04935230028695, 2.28413482012064},
//     {0, 0}};

// static constexpr float kLenCorrectionParams[4][3] = {
//     {0.00000000000000, 1.00000000000000, 0.00000000000000},
//     {-0.00684466128647, 0.96611076832519, 0.09590875245054},
//     {-0.02711839240663, 1.04651099781021, 0.09091361100963},
//     {0.00000000000000, 1.00000000000000, 0.00000000000000},
// };

float lengthCorrection(uint8_t winch_num, float len) {
    const auto& params = kLenCorrectionParams[winch_num];
    return params[0] * len * len + params[1] * len + params[2];
}
#endif

#define LQG_CALIBx
#ifdef LQG_CALIB
float lengthCorrectionLqg(uint8_t winch_num, float len) { return len; }
float lengthDotCorrectionLqg(uint8_t winch_num, float len, float ldot) {
  return ldot;
}
#else
static constexpr float kLengthCorrectionParamsLqg[4][4] = {
{1.2389479472868004, 1.0764637689435752, 0.9410871759325514, 0.0642813422522884},
{1.5177999608069015, 0.9241012838837206, 0.9372150814095981, 0.04959901641198605},
{1.4241000189055877, 0.9496752574553159, 0.9406414253611549, 0.04669591943490133},
{1.7253999656488601, 0.9247991600263843, 0.9300630987103626, 0.015522319773123102},
};
// length params are of the form:
// f(length - a, b, c) + a + d
//  where f(x, m1, m2) := (x < 0) ? m1*x : m2*x
float lengthCorrectionLqg(uint8_t winch_num, float len) {
  const float(&params)[4] = kLengthCorrectionParamsLqg[winch_num];
  float l0 = len - params[0];
  return ((l0 < 0) ? (params[1] * l0) : (params[2] * l0)) + params[0] +
         params[3];
}
float lengthDotCorrectionLqg(uint8_t winch_num, float len, float lendot) {
  const float(&params)[4] = kLengthCorrectionParamsLqg[winch_num];
  float l0 = len - params[0];
  return ((l0 < 0) ? params[1] : params[2]) * lendot;
}
#endif
