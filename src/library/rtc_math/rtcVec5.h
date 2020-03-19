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
 * file .......: rtcVec5.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 04/03/2008
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VEC5_H
#define RTC_VEC5_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T> class Vec5; // 5d Vector

/**
 * A 5-D vector.
 */
template <class T>
class Vec5: public Vec<T,5> {
public:
  // inherit member data and functions of parent
  using Vec<T,5>::x;
  using Vec<T,5>::set;

  // Constructors
  Vec5();
  Vec5(const T* d);
  Vec5(const T a);
  Vec5(const T x0, const T x1, const T x2, const T x3, const T x4);
  Vec5(const Vec<T,5>& v);

  // Cast Operation
  template <class U> Vec5(const Vec<U,5>& v);

  // Mutators
  void set(const T x0, const T x1, const T x2, const T x3, const T x4);
};

// Declare a few common typdefs
typedef Vec5<bool> Vec5b;
typedef Vec5<signed char> Vec5c;
typedef Vec5<unsigned char> Vec5uc;
typedef Vec5<signed short int> Vec5s;
typedef Vec5<unsigned short int> Vec5us;
typedef Vec5<int> Vec5i;
typedef Vec5<unsigned int> Vec5ui;
typedef Vec5<float> Vec5f;
typedef Vec5<double> Vec5d;

//==============================================================================
// Vec5<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize.
 */
template <class T>
inline Vec5<T>::Vec5() {}

/** Ctor that intalizes from array.
 */
template <class T>
inline Vec5<T>::Vec5(const T* d) : Vec<T,5>(d) {}

/** Ctor that intalizes all elements from a scalar.
 */
template <class T>
inline Vec5<T>::Vec5(const T a) : Vec<T,5>(a) {}

/** Ctor that initializes vector with given values.
 */
template <class T>
inline Vec5<T>::Vec5(const T x0, const T x1, const T x2, const T x3, const T x4) {
  set(x0,x1,x2,x3,x4);
}

/** Ctor that initializes an Vec5<T> with an Vec<T,5>.
 */
template <class T>
inline Vec5<T>::Vec5(const Vec<T,5>& v) : Vec<T,5>(v) {}

// Casting Operation

/** Casting Ctor that initializes an Vec5<T> with a Vec<U,5>.
 */
template <class T> template <class U>
inline Vec5<T>::Vec5(const Vec<U,5>& v) : Vec<T,5>(v) {}

// Mutators

/** Set vector.
 */
template <class T>
inline void Vec5<T>::set(const T x0, const T x1, const T x2, const T x3, const T x4) {
  x[0] = x0; x[1] = x1; x[2] = x2; x[3] = x3; x[4] = x4;
}

//==============================================================================
} // NAMESPACE puma
//==============================================================================
#endif // RTC_VEC5_H defined
//==============================================================================

