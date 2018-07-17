/**
 * definitions.h
 *
 *  Created on: 13.02.2012
 *      Author: jung
 */

#include <opencv2/opencv.hpp>

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

static const int MAT_TYPE = CV_32F;
#define SIN(x) sinf(x)
#define ASIN(x) asinf(x)
#define COS(x) cosf(x)
#define ACOS(x) acosf(x)
#define TAN(x) tanf(x)
#define ATAN(x) atanf(x)
#define ATAN2(y, x) atan2f(y, x)
#define FABS(x) fabsf(x)
#define POW(x, y) powf(x, y)
#define SQRT(x) sqrtf(x)
#define EXP(x) expf(x)
#define COPYSIGN(x, y) copysignf(x, y)
#define REMAINDER(x, y) remainderf(x, y)
#define FLOOR(x) floorf(x)
#define CEIL(x) ceilf(x)
#define ROUND(x) roundf(x)
#define TRUNC(x) truncf(x)

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#define MOD2(x) (x - ((x >> 1) << 1))
#define MOD4(x) (x - ((x >> 2) << 2))

static const float g_PI = 3.1415926535897932384626433832795029;
static const float g_PI_4 = (double)(3.1415926535897932384626433832795029 / 4.0);
static const float g_PI_2 = (double)(3.1415926535897932384626433832795029 / 2.0);
static const float g_PI2 = (double)(3.1415926535897932384626433832795029 * 2.0);
static const float g_180_PI = (double)(180.0/3.1415926535897932384626433832795029);

static const double sqrt_2 = 1.4142135623730950488016887242096981;

static const size_t UDP_BUFFER_SIZE = 1024;

#endif /* DEFINITIONS_H_ */
