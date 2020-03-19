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
 * file .......: rtcArray1.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ARRAY1_H
#define RTC_ARRAY1_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcArray.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int K> class Array; // K-dimensional array
template <class T> class Array1; // 1-dimensional array

/**
 * A 1-Dimensional Array
 */
template <class T>
class Array1: public Array<T,1> {
public:
  /// Constructors/Destructor
  Array1();
  Array1(int len);
  Array1(int len, const T* d);
  Array1(int len, const T a);
  Array1(const Array<T,1>& a);

  /// Mutators
  void setSize(int len);

  /// Accessors
  int size() const;

  /// inherit member data and functions of parent
  using Array<T,1>::x;
  using Array<T,1>::reset;
  using Array<T,1>::at;
protected:
  /// inherit member data and functions of parent
  using Array<T,1>::dim;
  using Array<T,1>::mul;
  using Array<T,1>::len;
};

// Declare a few common typdefs
typedef Array1<bool> Array1b;
typedef Array1<char> Array1c;
typedef Array1<unsigned char> Array1uc;
typedef Array1<int> Array1i;
typedef Array1<float> Array1f;
typedef Array1<double> Array1d;

//==============================================================================
// Array1<T>
//==============================================================================
// Constructors/Destructor

/** Ctor that does no initalization.
 */
template <class T>
inline Array1<T>::Array1() : Array<T,1>() {}

/** Ctor that starts with given dimensions
 * @param len_ size of one dimensional array
 */
template <class T>
inline Array1<T>::Array1(int len_) : Array<T,1>() {
  setSize(len_);
}

/** Ctor that initializes elements from an array.
 * @param _len size of one dimensional array
 * @param d pointer to the initalization array
 */
template <class T>
inline Array1<T>::Array1(int _len, const T* d) {
  setSize(_len);
  set(d);
}

/** Ctor that initializes all elements from a scalar.
 * @param _len size of one dimensional array
 * @param a the value to assign to all elements
 */
template <class T>
inline Array1<T>::Array1(int _len, const T a) {
  setSize(_len);
  set(a);
}

/** Ctor that initializes an Array1<T> with a Array<T,1>.
 * @param a is the array to duplicate
 */
template <class T>
inline Array1<T>::Array1(const Array<T,1>& a) : Array<T,1>(a) {}

/** Set the size of the array
 * @param len_ size of one dimensional array
 */
template <class T>
inline void Array1<T>::setSize(int len_)  {
  Array<T,1>::setSize(Vec<int,1>(len_));
}

// Accessors

/** Set the size of the array
 * @return size of one dimensional array
 */
template <class T>
inline int Array1<T>::size() const {
  return len;
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_ARRAY1_H defined
//==============================================================================

