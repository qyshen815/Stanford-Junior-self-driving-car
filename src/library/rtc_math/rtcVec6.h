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
 * file .......: rtcVec6.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 04/03/2008
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VEC6_H
#define RTC_VEC6_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T> class Vec6; // 6d Vector

/**
 * A 6-D vector.
 */
template <class T>
class Vec6: public Vec<T,6> {
public:
  // inherit member data and functions of parent
  using Vec<T,6>::x;
  using Vec<T,6>::set;

  // Constructors
  Vec6();
  Vec6(const T* d);
  Vec6(const T a);
  Vec6(const T x0, const T x1, const T x2,
       const T x3, const T x4, const T x5);
  Vec6(const Vec<T,6>& v);

  // Cast Operation
  template <class U> Vec6(const Vec<U,6>& v);

  // Mutators
  void set(const T x0, const T x1, const T x2,
           const T x3, const T x4, const T x5);
};

// Declare a few common typdefs
typedef Vec6<bool> Vec6b;
typedef Vec6<signed char> Vec6c;
typedef Vec6<unsigned char> Vec6uc;
typedef Vec6<signed short int> Vec6s;
typedef Vec6<unsigned short int> Vec6us;
typedef Vec6<int> Vec6i;
typedef Vec6<unsigned int> Vec6ui;
typedef Vec6<float> Vec6f;
typedef Vec6<double> Vec6d;

//==============================================================================
// Vec6<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize.
 */
template <class T>
inline Vec6<T>::Vec6() {}

/** Ctor that intalizes from array.
 */
template <class T>
inline Vec6<T>::Vec6(const T* d) : Vec<T,6>(d) {}

/** Ctor that intalizes all elements from a scalar.
 */
template <class T>
inline Vec6<T>::Vec6(const T a) : Vec<T,6>(a) {}

/** Ctor that initializes vector with given values.
 */
template <class T>
inline Vec6<T>::Vec6(const T x0, const T x1, const T x2,
                     const T x3, const T x4, const T x5) {
  set(x0,x1,x2,x3,x4,x5);
}

/** Ctor that initializes an Vec6<T> with an Vec<T,6>.
 */
template <class T>
inline Vec6<T>::Vec6(const Vec<T,6>& v) : Vec<T,6>(v) {}

// Casting Operation

/** Casting Ctor that initializes an Vec6<T> with a Vec<U,6>.
 */
template <class T> template <class U>
inline Vec6<T>::Vec6(const Vec<U,6>& v) : Vec<T,6>(v) {}

// Mutators

/** Set vector.
 */
template <class T>
inline void Vec6<T>::set(const T x0, const T x1, const T x2,
                         const T x3, const T x4, const T x5) {
  x[0] = x0; x[1] = x1; x[2] = x2;
  x[3] = x3; x[4] = x4; x[5] = x5;
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VEC6_H defined
//==============================================================================
