//-*-c++-*-
#ifndef MATHUTIL_H
#define MATHUTIL_H

 /** 
 * @file mathutil.h
 * @brief Basic math utility functions and several useful #defines
 * @author James R. Diebel, Stanford University
 * 
 * - History:
 *  - 31 July 2005 - Started (JD)
 */

#include <cmath>
#include <cstdlib>
#include <time.h>

#ifdef WIN32
  #include <sys/timeb.h>
  #include <Winsock2.h>
#else 
  #include <sys/time.h>
#endif

///Conversion factor for degrees to radians
#define D2R 0.017453292519943 

///Conversion factor for radians to degrees
#define R2D 57.295779513082

namespace sla {
  /// return whether a given value is within specified open bounds
  template <class T> 
  inline bool openBounds(const T x, const T a, const T b) {
    return (x > a && a < b);
  }
  
  /// return whether a given value is within specified closed bounds
  template <class T> 
  inline bool closedBounds(const T x, const T a, const T b) {
    return (x >= a && a <= b);
  }

  /// clamp the given variable to passed range
  template <class T> 
  inline T clamp(const T x, const T a, const T b) {
    if (x < a) return a;
    else return (x>b)? b:x;
  }

  /// round to nearest given value
  template <class T>
  inline T roundTo(const T x, const T v) {
    return v*T(floor(double(x/v)+0.5));
  }

  /// general purpose comparison functions
  template <class T>
  inline int compareAscending(const void* a, const void* b) {
    if (*(T*)a == *(T*)b) return 0;
    else return 2*int(*(T*)a > *(T*)b)-1;
  }
  template <class T>
  inline int compareDescending(const void* a, const void* b) {
    if (*(T*)a == *(T*)b) return 0;
    else return 2*int(*(T*)a < *(T*)b)-1;
  }
  
  /// return the sign of a number
  template <class T> 
  inline T sign(const T x) { 
    if (x > T(0)) return T(1);
    if (x < T(0)) return T(-1);
    return T(0);
  }
  
  /// return (a^2 + b^2)^(1/2) without over/underflow
  template <class T> 
  inline T pythag(T a, T b) {
    T absa=fabs(a); T absb=fabs(b);
    if (absa > absb) return absa*sqrt(1.0+(absb/absa)*(absb/absa));
    else return (absb==T(0) ? T(0) : absb*sqrt(T(1)+(absa/absb)*(absa/absb)));
  }

  /// square a number
  template <class T> 
  inline T sqr(const T x) { return x*x; }
  
  /// cube a number
  template <class T> 
  inline T cube(const T x) { return x*x*x; }
  
  /// take number to the 4th power
  template <class T> 
  inline T quart(const T x) { 
    T x_sqr = x*x;
    return x_sqr*x_sqr; 
  }
  
  /// take number to the 5th power
  template <class T> 
  inline T quint(const T x) { 
    T x_sqr = x*x;
    return x_sqr*x_sqr*x; 
  }

  /// swap values
  template <class T> 
  inline void swap(T& x1, T& x2) { 
    T temp = x1; x1 = x2; x2 = temp;
  }

  /// compare floating point numbers (for sorts)
  template <class T> 
  inline int compare(const void * a, const void * b) {
    if (*(T*)a < *(T*)b) return -1;
    else if (*(T*)a == *(T*)b) return 0;
    else return 1;
  }

  // random number functions
  
  /// Returns a sample from a normal distribution
  template <class T> 
  inline T normalRand(T mean = T(0), T stdev = T(1)) {
    const double norm = 1.0/(RAND_MAX + 1.0);
    double u = 1.0 - std::rand()*norm;
    double v = std::rand()*norm;
    double z = std::sqrt(-2.0*log(u))*std::cos(2.0*M_PI*v);
    return T(mean + stdev*z);
  }
  
  /// Returns a sample from a uniform distribution
  template <class T> 
  inline T uniformRand(T lower = T(0), T upper = T(1)) {
    return lower + (T)(double(upper-lower)*rand()/(RAND_MAX+1.0));
  }

#ifdef WIN32  
  /// Seed psudo-random number generator
  inline void seedRand() {
    struct _timeb timebuffer;
    _ftime( &timebuffer );
    unsigned int n = int(timebuffer.time*1000 + timebuffer.dstflag); 
    std::srand(n);
  }
#else
  /// Seed psudo-random number generator
  inline void seedRand() {
    timeval tv; gettimeofday(&tv,NULL);
    unsigned int n = int(tv.tv_sec*1000000 + tv.tv_usec); std::srand(n);
  }
#endif
}

#endif
