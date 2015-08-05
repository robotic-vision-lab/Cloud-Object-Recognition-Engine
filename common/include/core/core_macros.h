/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef CORE_MACROS_H_
#define CORE_MACROS_H_

#include <cstdlib>
#include <boost/cstdint.hpp>
#include <core/core_config.h>

namespace core 
{
  using boost::uint8_t;
  using boost::int8_t;
  using boost::int16_t;
  using boost::uint16_t;
  using boost::int32_t;
  using boost::uint32_t;
  using boost::int64_t;
  using boost::uint64_t;
}

#if defined __INTEL_COMPILER
  #pragma warning disable 2196 2536 279
#endif

#if defined _MSC_VER
  #pragma warning (disable: 4521 4251)
#endif

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

// MSCV doesn't have std::{isnan,isfinite}
#if defined _WIN32 && defined _MSC_VER

// If M_PI is not defined, then probably all of them are undefined
#ifndef M_PI
// Copied from math.h
# define M_PI    3.14159265358979323846                 // pi
# define M_PI_2  1.57079632679489661923                 // pi/2
# define M_PI_4  0.78539816339744830962                 // pi/4
# define M_PIl   3.1415926535897932384626433832795029L  // pi
# define M_PI_2l 1.5707963267948966192313216916397514L  // pi/2
# define M_PI_4l 0.7853981633974483096156608458198757L  // pi/4
#endif

# define core_isnan(x)    _isnan(x)
# define core_isfinite(x) (_finite(x) != 0)
# define core_isinf(x)    (_finite(x) == 0)

# define __PRETTY_FUNCTION__ __FUNCTION__
# define __func__ __FUNCTION__

#elif ANDROID
// Use the math.h macros
# include <math.h>
# define core_isnan(x)    isnan(x)
# define core_isfinite(x) isfinite(x)
# define core_isinf(x)    isinf(x)

#elif _GLIBCXX_USE_C99_MATH
// Are the C++ cmath functions enabled?
# include <cmath>
# define core_isnan(x)    std::isnan(x)
# define core_isfinite(x) std::isfinite(x)
# define core_isinf(x)    std::isinf(x)

#elif __PATHCC__
# include <cmath>
# include <stdio.h>
template <typename T> int
core_isnan (T &val)
{
  return (val != val);
}
# define core_isfinite(x) std::isfinite(x)
# define core_isinf(x)    std::isinf(x)

#else
// Use the math.h macros
# include <math.h>
# define core_isnan(x)    isnan(x)
# define core_isfinite(x) isfinite(x)
# define core_isinf(x)    isinf(x)

#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

/* Win32 doesn't seem to have rounding functions.
 * Therefore, implement our own versions of these functions here.
 */
__inline double
core_round (double number)
{
  return (number < 0.0 ? ceil (number - 0.5) : floor (number + 0.5));
}
__inline float
core_round (float number)
{
  return (number < 0.0f ? ceilf (number - 0.5f) : floorf (number + 0.5f));
}

#define core_lrint(x) (static_cast<long int>(core_round(x)))
#define core_lrintf(x) (static_cast<long int>(core_round(x)))

#ifdef _WIN32
__inline float
log2f (float x)
{
  return (static_cast<float> (logf (x) * M_LOG2E));
}
#endif

#ifdef WIN32
#define core_sleep(x) Sleep(1000*(x))
#else
#define core_sleep(x) sleep(x)
#endif

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif
#ifndef PVARN
#define PVARN(s) \
  #s << " = " << (s) << "\n"
#endif
#ifndef PVARC
#define PVARC(s) \
  #s << " = " << (s) << ", " << std::flush
#endif
#ifndef PVARS
#define PVARS(s) \
  #s << " = " << (s) << " " << std::flush
#endif
#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif
#ifndef PVARAN
#define PVARAN(s) \
  #s << " = " << RAD2DEG(s) << "deg\n"
#endif
#ifndef PVARAC
#define PVARAC(s) \
  #s << " = " << RAD2DEG(s) << "deg, " << std::flush
#endif
#ifndef PVARAS
#define PVARAS(s) \
  #s << " = " << RAD2DEG(s) << "deg " << std::flush
#endif

#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)

#ifndef ERASE_STRUCT
#define ERASE_STRUCT(var) memset(&var, 0, sizeof(var))
#endif

#ifndef ERASE_ARRAY
#define ERASE_ARRAY(var, size) memset(var, 0, size*sizeof(*var))
#endif

#ifndef SET_ARRAY
#define SET_ARRAY(var, value, size) { for (int i = 0; i < static_cast<int> (size); ++i) var[i]=value; }
#endif

/* //This is copy/paste from http://gcc.gnu.org/wiki/Visibility */
/* #if defined _WIN32 || defined __CYGWIN__ */
/*   #ifdef BUILDING_DLL */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllexport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #else */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllimport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #endif */
/*   #define DLL_LOCAL */
/* #else */
/*   #if __GNUC__ >= 4 */
/* #define DLL_PUBLIC __attribute__ ((visibility("default"))) */
/* #define DLL_LOCAL  __attribute__ ((visibility("hidden"))) */
/*   #else */
/*     #define DLL_PUBLIC */
/*     #define DLL_LOCAL */
/*   #endif */
/* #endif */

#ifndef CORE_EXTERN_C
    #ifdef __cplusplus
        #define CORE_EXTERN_C extern "C"
    #else
        #define CORE_EXTERN_C
    #endif
#endif

#if defined WIN32 || defined _WIN32 || defined WINCE || defined __MINGW32__
    #ifdef COREAPI_EXPORTS
        #define CORE_EXPORTS __declspec(dllexport)
    #else
        #define CORE_EXPORTS
    #endif
#else
    #define CORE_EXPORTS
#endif

#if defined WIN32 || defined _WIN32
    #define CORE_CDECL __cdecl
    #define CORE_STDCALL __stdcall
#else
    #define CORE_CDECL
    #define CORE_STDCALL
#endif

#ifndef COREAPI
    #define COREAPI(rettype) CORE_EXTERN_C CORE_EXPORTS rettype CORE_CDECL
#endif

#if defined (__GNUC__) || defined (__PGI) || defined (__IBMCPP__) || defined (__SUNPRO_CC)
  #define CORE_ALIGN(alignment) __attribute__((aligned(alignment)))
#elif defined (_MSC_VER)
  #define CORE_ALIGN(alignment) __declspec(align(alignment))
#else
  #error Alignment not supported on your platform
#endif

#if defined(__GLIBC__) && ((__GLIBC__>=2 && __GLIBC_MINOR__ >= 8) || __GLIBC__>2) \
 && defined(__LP64__)
  #define GLIBC_MALLOC_ALIGNED 1
#else
  #define GLIBC_MALLOC_ALIGNED 0
#endif

#if defined(__FreeBSD__) && !defined(__arm__) && !defined(__mips__)
  #define FREEBSD_MALLOC_ALIGNED 1
#else
  #define FREEBSD_MALLOC_ALIGNED 0
#endif

#if defined(__APPLE__) || defined(_WIN64) || GLIBC_MALLOC_ALIGNED || FREEBSD_MALLOC_ALIGNED
  #define MALLOC_ALIGNED 1
#else
  #define MALLOC_ALIGNED 0
#endif

#endif  // CORE_MACROS_H_
