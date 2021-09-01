#pragma once

#define ESTOP 11
#define ESTOP_HIGHx
#define SerialD Serial
#define btSerial Serial5

#ifndef DEBUG_CONSTANTS

/******** Winch Parameters **********/
float kZeros[4] = {4.868, 4.672, 6.490, 7.980};
static constexpr float kR = 0.0254 / 2;
static constexpr float kSprayDelay_s = 0.40;

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

static constexpr float kMountPoints[4][2] = {
    {kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};
