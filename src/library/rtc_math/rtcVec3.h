/*
 * Copyright (C) 2008
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: Vec3.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VEC3_H
#define RTC_VEC3_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T> class Vec3; // 3d Vector
template <class T> class SMat3; // 3x3 square matrix

/**
 * A 3-D vector.
 */
template <class T>
class Vec3: public Vec<T,3> {
public:
  // Constructors
  Vec3();
  Vec3(const T* d);
  Vec3(const T a);
  Vec3(const T x0, const T x1, const T x2);
  Vec3(const Vec<T,3>& v);

  // Cast Operation
  template <class U> Vec3(const Vec<U,3>& v);

  // Mutators
  void set(const T x0, const T x1, const T x2);

  // Cross Product
  Vec3<T> cross(const Vec3<T>& v) const;

  // Cross Product operator
  inline Vec3<T> operator%(const Vec3<T>& v) const;

  // inherit member data and functions of parent
  using Vec<T,3>::x;
  using Vec<T,3>::set;
};

// Declare a few common typdefs
typedef Vec3<bool> Vec3b;
typedef Vec3<signed char> Vec3c;
typedef Vec3<unsigned char> Vec3uc;
typedef Vec3<signed short int> Vec3s;
typedef Vec3<unsigned short int> Vec3us;
typedef Vec3<int> Vec3i;
typedef Vec3<unsigned int> Vec3ui;
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;

//==============================================================================
// Vec3<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize.
 */
template <class T>
inline Vec3<T>::Vec3() {}

/** Ctor that intalizes from array.
 */
template <class T>
inline Vec3<T>::Vec3(const T* d) : Vec<T,3>(d) {}

/** Ctor that intalizes all elements from a scalar.
 */
template <class T>
inline Vec3<T>::Vec3(const T a) : Vec<T,3>(a) {}

/** Ctor that initializes vector with given values.
 */
template <class T>
inline Vec3<T>::Vec3(const T x0, const T x1, const T x2) {
  set(x0,x1,x2);
}

/** Ctor that initializes an Vec3<T> with an Vec<T,3>.
 */
template <class T>
inline Vec3<T>::Vec3(const Vec<T,3>& v) : Vec<T,3>(v){}

// Casting Operation

/** Casting Ctor that initializes an Vec3<T> with a Vec<U,3>.
 */
template <class T> template <class U>
inline Vec3<T>::Vec3(const Vec<U,3>& v) : Vec<T,3>(v) {}


// Mutators

/** Set vector.
 */
template <class T>
inline void Vec3<T>::set(const T x0, const T x1, const T x2) {
  x[0] = x0; x[1] = x1; x[2] = x2;
}

// Cross Product

/** Calculate the cross product.
 * @param v a 3-vector
 * @return the cross product: "this" cross "v"
 */
template <class T>
inline Vec3<T> Vec3<T>::cross(const Vec3<T>& v) const {
  return Vec3<T>(x[1]*v.x[2]-x[2]*v.x[1],
                 x[2]*v.x[0]-x[0]*v.x[2],
                 x[0]*v.x[1]-x[1]*v.x[0]);
}

/// cross product: only defined for Vec3* as specialization
/// \see Vec3::cross
template <class T>
inline Vec3<T> Vec3<T>::operator%(const Vec3<T>& v) const {
  return cross(v);
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VEC3_H defined
//==============================================================================
