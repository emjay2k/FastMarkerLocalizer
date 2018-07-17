#include <limits>

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2		1.57079632679489661923
#endif
#ifndef DBL_EPSILON
#define DBL_EPSILON numeric_limits<double>::epsilon()
#endif
#ifndef DBL_MIN
#define DBL_MIN numeric_limits<double>::min()
#endif
#ifndef DBL_MAX
#define DBL_MAX numeric_limits<double>::max()
#endif
#ifndef FLT_EPSILON
#define FLT_EPSILON numeric_limits<float>::epsilon()
#endif
#ifndef FLT_MIN
#define FLT_MIN numeric_limits<float>::min()
#endif
#ifndef FLT_MAX
#define FLT_MAX numeric_limits<float>::max()
#endif
