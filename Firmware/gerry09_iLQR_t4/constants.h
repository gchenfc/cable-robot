#pragma once

/******** Winch Parameters **********/
float zeros[4] = {40.24, 43.93, 33.59, 10.96};
static constexpr float kR = 0.0254 / 2;

/******** Frame Geometry **********/
// Klaus lab measurements:
//     w        h    aspect ratio     diag
// 0: 2.96  -  2.24  (1.3214 : 1)  -  3.77
// 1: 2.91  -  2.19  (1.3288 : 1)  -  3.72
// 2: 3.00  -  2.28  (1.3158 : 1)  -  3.82
// 3: 2.89  -  2.17  (1.3318 : 1)  -  3.70
// float kWidth = 2.94, kHeight = 2.22;

// DFL Measurements
// 0: 3.05  -  2.26
// 1: 3.12  -  2.35
// 2: 3.14  -  2.38
// 3: 3.05  -  2.29
// float kWidth = 3.09, kHeight = 2.32;
static constexpr float kWidth = 3.05 - 0.17, kHeight = 2.32;

/******** Mathematical Constants **********/
static constexpr float kTau = 2 * 3.1415926535897932384626433832795;
