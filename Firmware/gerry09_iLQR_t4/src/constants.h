#pragma once

#define ESTOP 11
#define ESTOP_HIGHx
#define SerialD Serial
#define SerialComputer SerialUSB1
#define btSerial Serial5

#ifndef DEBUG_CONSTANTS

/******** Winch Parameters **********/
float kZeros[4] = {4.868, 4.672, 6.490, 7.980};
static constexpr float kR = 0.0254 / 2;
// static constexpr float kSprayDelay_s = 0.40;
static constexpr float kSprayDelay_s = 0.0;

/******** Frame Geometry **********/
// Klaus lab measurements:
//      w          h
// 0: 1.9815  -  1.7286
// 1: 2.1146  -  1.8300
// 2: 2.0470  -  1.7721
// 3: 2.1305  -  1.8637
static constexpr float kFrameWidth = 3.05, kFrameHeight = 2.44;

// Constants including carriage
static constexpr float kCarriageWidth = 0.22377, kCarriageHeight = 0.22377;
static constexpr float kWidth = kFrameWidth - kCarriageWidth,
                       kHeight = kFrameHeight - kCarriageHeight;

/******** Mathematical Constants **********/
static constexpr float kTau = 2 * 3.1415926535897932384626433832795;

#endif

#define CALIBRATIONx
#ifdef CALIBRATION
static constexpr bool kUseLenCorrectionParamsFromEeprom = false;
static constexpr bool kUseMountPointsFromEeprom = false;
static constexpr bool kOverwriteLenCorrectionParamsToEeprom = true;
static constexpr bool kOverwriteMountPointsToEeprom = true;
#else
static constexpr bool kUseLenCorrectionParamsFromEeprom = true;
static constexpr bool kUseMountPointsFromEeprom = true;
static constexpr bool kOverwriteLenCorrectionParamsToEeprom = false;
static constexpr bool kOverwriteMountPointsToEeprom = false;
#endif

static float lenCorrectionParamsAll[4][3] = {
    {0., 1., 0.},
    {0., 1., 0.},
    {0., 1., 0.},
    {0., 1., 0.},
};
static float mountPoints[4][2] = {
    {kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};

float lenCorrection(float len, const float (&params)[3]) {
  return params[0] * len * len + params[1] * len + params[2];
}
float lenDotCorrection(float len, float lenDot, const float (&params)[3]) {
  return 2 * params[0] * len * lenDot + params[1] * lenDot;
}
float lenCorrectionInv(float calibratedLen, const float (&params)[3]) {
  const float &a = params[0], &b = params[1], c = params[2] - calibratedLen;
  if (a * a < 1e-8) {  // use a^2 as shorthand instead of abs
    return -c / b;
  }
  return (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
}
float lenDotCorrectionInv(float calibratedLen, float calibratedLenDot,
                          const float (&params)[3]) {
  float uncalibratedLen = lenCorrectionInv(calibratedLen, params);
  return calibratedLenDot / (2 * params[0] * uncalibratedLen + params[1]);
}
