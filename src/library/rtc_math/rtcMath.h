/*
 * Copyright (C) 2009
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: rtcMathUtil.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_MATHUTIL_H
#define RTC_MATHUTIL_H

//== INCLUDES ==================================================================
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>

// rtc
#include <rtcException.h>
#include <rtcStringTools.h>

#ifdef _WIN32
  //#include <sys/timeb.h>
  #include <windows.h>
  #define M_PI 3.14159265358979323846
#else
  #include <sys/time.h>
#endif

//#define MATMATH_CHECK_BOUNDS 1

/// Conversion factor for degrees to radians
#define EPS       1E-5

/// Conversion factor for degrees to radians
#define D2R       0.0174532925199432957692

/// Conversion factor for radians to degrees
#define R2D       57.295779513082320876

/// Some useful mathematical constants
#define LN2       0.693147180559945309417
#define LN10      2.30258509299404568402
#define PI        3.14159265358979323846
#define TWOPI     6.28318530717958647692
#define PI_2      1.57079632679489661923
#define PI_4      0.785398163397448309616
#define SQRT2     1.41421356237309504880
#define SQRT1_2   0.707106781186547524401

//== NAMESPACES ================================================================
namespace rtc {

/// clamp the given variable x to passed range [a,b]
template <class T> inline T rtc_clamp(const T x, const T a, const T b) {
  if (x < a) return a;
  else return (x>b)?b:x;
}

/// returns true if the given variable x is in passed range [a,b]
template <class T> inline bool rtc_within(const T x, const T a, const T b) {
  return (x>=a && x<=b);
}

/// return the sign of a number
template <class T> inline T rtc_sign(const T x) {
  return ((x != T(0))? x/fabs(x) : T(1));
}

/// returns the integral digits of the specified number x
template <class T> inline int rtc_trunc(const T x) {
  return (int)x;
}

/// rounds a value to the nearest integer.
template <class T> inline int rtc_round(const T x) {
  if (x >= 0)
    return (int)(x + T(0.5));
  else
    return (int)(x - T(0.5));
}

template <class T> inline	T rtc_lerp(const T& t0, const T& t1, float a) {
  return t0 * (1.0f - a) + t1 * a;
}

/// return (a^2 + b^2)^(1/2) without over/underflow
template <class T> inline T rtc_pythag(T a, T b) {
  T absa=fabs(a); T absb=fabs(b);
  if (absa > absb) return absa*sqrt(1.0+(absb/absa)*(absb/absa));
  else return (absb==T(0) ? T(0) : absb*sqrt(T(1)+(absa/absb)*(absa/absb)));
}

/// square a number
template <class T> inline T rtc_sqr(const T x) { return x*x; }

/// cube a number
template <class T> inline T rtc_cube(const T x) { return x*x*x; }

/// take number to the 4th power
template <class T> inline T rtc_quart(const T x) { return x*x*x*x; }

/// take number to the 5th power
template <class T> inline T rtc_quint(const T x) { return x*x*x*x*x; }

/// a number^3/2
template <class T> inline T rtc_cuberoot(const T x) { return cube(sqrt(x)); }

/// return the maximum
template <class T> inline T rtc_max(const T x1, const T x2) {
  return (x1>x2)?x1:x2;
}

/// return the minimum
template <class T> inline T rtc_min(const T x1, const T x2) {
  return (x1<x2)?x1:x2;
}

/// swap values
template <class T> inline void rtc_swap(T& x1, T& x2) {
  T temp = x1; x1 = x2; x2 = temp;
}

/// compare floating point numbers (for sorts)
template <class T> inline int rtc_compare(const void * a, const void * b) {
  if (*(T*)a < *(T*)b) return -1;
  else if (*(T*)a == *(T*)b) return 0;
  else return 1;
}

// random number functions

/// Returns a sample from a normal distribution
template <class T> inline T rtc_normal_rand(T mean = T(0), T stdev = T(1)) {
  const double norm = 1.0/(RAND_MAX + 1.0);
  double u = 1.0 - std::rand()*norm;
  double v = std::rand()*norm;
  double z = std::sqrt(-2.0*log(u))*std::cos(2.0*M_PI*v);
  return T(mean + stdev*z);
}

/*
/// Returns a sample from a multivariate distribution
template <class T, int M> Vec<T,M> multivariateGauss(const Vec<T,M>& mean = Vec<T,M>(0),
                                                     const SMat<T,M>& cov = SMat<T,M>(1)) {
  Vec<T,M> v;
  SMat<T,M> S(cov);
  int n=S.choleskyDecomp();
  assert(n==0);
  S.transpose();
  v.normalRand();
  v = S*v;
  v += mean;
  return v;
}
*/

/// Returns a sample from a uniform distribution
template <class T> inline T rtc_uniform_rand(T lower = T(0), T upper = T(1)) {
  return lower + T(double(upper - lower)*std::rand()/(RAND_MAX+1.0));
}

/// Seed pseudo-random number generator
#ifdef _WIN32
inline void seedRand(void) {
  SYSTEMTIME SystemTime; GetSystemTime(&SystemTime);
unsigned int n = int(SystemTime.wSecond*1000000 + SystemTime.wMilliseconds); std::srand(n);
}
#else
inline void rtc_seed_rand(void) {
  timeval tv; gettimeofday(&tv,NULL);
  unsigned int n = int(tv.tv_sec*1000000 + tv.tv_usec); std::srand(n);
}
#endif

/// Absolute value
inline double rtc_abs(const double& x) { return std::fabs(x); }
inline float rtc_abs(const float& x) { return (float)std::fabs(static_cast<double>(x)); }
inline int rtc_abs(const int& x) { return std::abs(x); }
inline long int rtc_abs(const long int& x) { return std::labs(x); }
#ifndef _WIN32
inline long long int rtc_abs(const long long int& x) { return std::llabs(x); }
#endif

/// Square root
template <class T> inline T rtc_sqrt(const T& x) {
  return static_cast<T>(std::sqrt(static_cast<double>(x)));
}

/// x by the power of y
template <class T, class U> inline T rtc_pow(const T& x, const U& y) {
  return static_cast<T>(std::pow(static_cast<double>(x),static_cast<double>(y)));
}

/// x by the power of y
template<> inline float rtc_pow<float,int>(const float &x, const int &y)
{
  float ret=1;
  for (int i=0;i<y;i++) ret*=x;
  return ret;
}

/// normalizes theta to a [-PI,PI] interval
template <class T> inline T rtc_normalize_theta(T theta)
{
  int multiplier;

  if (theta >= -T(PI) && theta < T(PI))
    return theta;

  multiplier = (int)(theta / T(TWOPI));
  theta = theta - multiplier*T(TWOPI);
  if (theta >= T(PI))
    theta -= T(TWOPI);
  if (theta < -T(PI))
    theta += T(TWOPI);

  return theta;
}

template <class T> inline T rtc_safe_acos(T costheta)
{
  if (costheta>T(1)) costheta=T(1);
  if (costheta<T(-1)) costheta=T(-1);
  return (T)acos(double(costheta));
}

/// converts cartesian coordinates to spherical coordinates
template <class T> inline void rtc_cartesian_to_spherical(const T& x, const T& y, const T& z, T& r, T& theta, T& phi)
{
  r = sqrt(rtc_sqr(x)+rtc_sqr(y)+rtc_sqr(z));
  if(r<EPS) {
    r = theta = phi = 0;
  }
  else {
    theta = atan2(y,x);
    phi = acos(z/r);
  }
}

/// converts spherical coordinates to cartesian coordinates
template <class T> inline void rtc_spherical_to_cartesian(const T& r, const T& theta, const T& phi, T& x, T& y, T& z)
{
  float ct = cos(theta);
  float st = sin(theta);
  float sp = sin(phi);
  float cp = cos(phi);
  x = r*ct*sp;
  y = r*st*sp;
  z = r*cp;
}

/// r in [0,inf), theta in [0,2PI), and phi in [0,PI],
template <class T> inline void rtc_normalize_spherical(T& theta, T& phi)
{
  int multiplier;

  // normalize phi
  if (phi < T(0) || phi > T(PI)) {
    multiplier = (int)(phi / T(TWOPI));
    phi = phi - multiplier*T(TWOPI);
    if (phi > T(PI)) {
      phi = T(TWOPI)-phi;
      theta += T(PI);
    }
    else if (phi < T(0)) {
      phi*=-1;
      theta += T(PI);
    }
  }

  // normalize theta
  theta = rtc_normalize_theta(theta);
}

/// Sinus
inline float rtc_sin(float x) { return static_cast<float>(std::sin(x)); }
/// Sinus
inline double rtc_sin(double x) { return std::sin(x); }
/// Cosinus
inline float rtc_cos(float x) { return static_cast<float>(std::cos(x)); }
/// Cosinus
inline double rtc_cos(double x) { return std::cos(x); }
/// Tangent
inline float rtc_tan(float x) { return static_cast<float>(std::tan(x)); }
/// Tangent
inline double rtc_tan(double x) { return std::tan(x); }
/// Arc sine
inline float rtc_asin(float x) { return static_cast<float>(std::asin(x)); }
/// Arc sine
inline double rtc_asin(double x) { return std::asin(x); }
/// Arc cosine
inline float rtc_acos(float x) { return static_cast<float>(std::acos(x)); }
/// Arc cosine
inline double rtc_acos(double x) { return std::acos(x); }
/// Arc tangent
inline float rtc_atan(float x) { return static_cast<float>(std::atan(x)); }
/// Arc tangent
inline double rtc_atan(double x) { return std::atan(x); }
/// Arc tangent of y/x
inline float rtc_atan2(float y,float x) { return static_cast<float>(std::atan2(y,x)); }
/// Arc tangent of y/x
inline double rtc_atan2(double y,double x) { return std::atan2(y,x); }

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_MATHUTIL_H defined
//==============================================================================

