#pragma once

#define ESTOP 11
#define ESTOP_HIGHx
#define SerialD Serial
#define SerialComputer SerialUSB1
#define btSerial Serial5

// Select which robot:
#define KLAUS
#define DFLx
#define AIRx
#define HYDROPONICSx

#ifndef DEBUG_CONSTANTS

/******** Winch Parameters **********/
float kZeros[4] = {4.868, 4.672, 6.490, 7.980};  // these will get reset anyway
static constexpr float kR = 0.0254 / 2;
// static constexpr float kSprayDelay_s = 0.40;
static constexpr float kSprayDelay_s = 3.0;

/******** Safety parameters **********/
#if defined(KLAUS) || defined(DFL) || defined(HYDROPONICS)
static constexpr float kDefaultPaddingWidth = 0.1f;
static constexpr float kDefaultPaddingHeight = 0.1f;
static constexpr float kDefaultPaddingTol = 0.1f;
#endif
#ifdef AIR
static constexpr float kDefaultPaddingWidth = 0.3f;
static constexpr float kDefaultPaddingHeight = 0.3f;
static constexpr float kDefaultPaddingTol = 0.1f;
#endif

/******** Frame Geometry **********/
/* NOTE: THESE ACTUALLY GET OVERWRITTEN BY EEPROM */
#ifdef KLAUS
static constexpr float kFrameWidth = 3.05, kFrameHeight = 2.44;
static constexpr float kCarriageWidth = 0.187, kCarriageHeight = 0.122;
#endif
#ifdef DFL
static constexpr float kFrameWidth = 3.05, kFrameHeight = 2.44;
static constexpr float kCarriageWidth = 0.187, kCarriageHeight = 0.122;
#endif
#ifdef AIR
static constexpr float kFrameWidth = 6.17, kFrameHeight = 2.64;
static constexpr float kCarriageWidth = 0.228, kCarriageHeight = 0.228;
#endif
#ifdef HYDROPONICS
static constexpr float kFrameWidth = 2.20, kFrameHeight = 2.00;
static constexpr float kCarriageWidth = 0.22377, kCarriageHeight = 0.22377;
#endif

// Constants including carriage
static constexpr float kWidth = kFrameWidth - kCarriageWidth,
                       kHeight = kFrameHeight - kCarriageHeight;

/******** Mathematical Constants **********/
static constexpr float kTau = 2 * 3.1415926535897932384626433832795;

#endif

#define CALIBRATION 
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

/********** LENGTH CORRECTION ***********/
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
