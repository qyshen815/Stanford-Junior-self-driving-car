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
 * file .......: rtcVec2.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VEC2_H
#define RTC_VEC2_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T> class Vec2; // 2d Vector

/**
 * A 2-D vector.
 */
template <class T>
class Vec2: public Vec<T,2> {
public:
  // Constructors
  Vec2();
  Vec2(const T* d);
  Vec2(const T a);
  Vec2(const T x0, const T x1);
  Vec2(const Vec<T,2>& v);

  // Cast Operation
  template <class U> Vec2(const Vec<U,2>& v);

  // Mutators
  void set(const T x0, const T x1);

  // Perp Operator
  Vec2<T> perp() const;

  // inherit member data and functions of parent
  using Vec<T,2>::x;
  using Vec<T,2>::set;
};

// Declare a few common typdefs
typedef Vec2<bool> Vec2b;
typedef Vec2<char> Vec2c;
typedef Vec2<unsigned char> Vec2uc;
typedef Vec2<int> Vec2i;
typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;

//==============================================================================
// Vec2<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize.
 */
template <class T>
inline Vec2<T>::Vec2() {
}

/** Ctor that intalizes from array.
 */
template <class T>
inline Vec2<T>::Vec2(const T* d) : Vec<T,2>(d) {}

/** Ctor that intalizes all elements from a scalar.
 */
template <class T>
inline Vec2<T>::Vec2(const T a) : Vec<T,2>(a) {}

/** Ctor that initializes vector with given values.
 */
template <class T>
inline Vec2<T>::Vec2(const T x0, const T x1) {
  set(x0,x1);
}

/** Ctor that initializes an Vec2<T> with a Vec<T,2>.
 */
template <class T>
inline Vec2<T>::Vec2(const Vec<T,2>& v) : Vec<T,2>(v) {}

// Casting Operation

/** Casting Ctor that initializes an Vec2<T> with a Vec<U,2>.
 */
template <class T> template <class U>
inline Vec2<T>::Vec2(const Vec<U,2>& v) : Vec<T,2>(v) {}

// Mutators

/** Set vector.
 */
template <class T>
inline void Vec2<T>::set(const T x0, const T x1) {
  x[0] = x0; x[1] = x1;
}

/** Perp Operator
 */
template <class T>
inline Vec2<T> Vec2<T>::perp() const {
  return Vec2<T>(-x[1],x[0]);
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VEC2_H defined
//==============================================================================
