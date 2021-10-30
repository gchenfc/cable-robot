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

#define CALIBRATIONx
#ifdef CALIBRATION
static constexpr float kMountPoints[4][2] = {
    {kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};
float lengthCorrection(uint8_t winch_num, float len) {
    return len;
}
#else
static constexpr float kLenCorrectionParams[4][3] = {
    {0.00000000000000, 1.00000000000000, 0.00000000000000},
    {-0.01188844089481, 0.92855624871954, 0.08171694405092},
    {-0.01133362762128, 0.95572779537438, -0.04781298818415},
    {0.00000000000000, 1.00000000000000, 0.00000000000000},
};
static constexpr float kMountPoints[4][2] = {
    {kWidth, 0}, {2.87172620453055, 2.33246622786011}, {0.16871802351569, 2.28522716180476}, {0, 0}};
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
