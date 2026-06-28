#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__
#ifndef M_PI
#ifndef PI
#define PI M_PI
#endif
#define M_PI 3.14159265358979323846
#endif
/* Preprocessor to avoid doxygen error */
#if !defined(DOXYGEN_SHOULD_SKIP_THIS)

float periodicmod(float x, float y);

float inrange(float x, float min, float max);

float saturate(float x, float min, float max);

float sign(float x);

#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif // __MATHUTILS_H__
