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
 * file .......: rtcArray3.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ARRAY3_H
#define RTC_ARRAY3_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec3.h"
#include "rtcArray.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int K> class Array;  // K-dimensional array
template <class T> class Array3;        // 3-dimensional array
template <class T> class Vec3;          // 3d Vector

/**
 * A 3-Dimensional Array
 */
template <class T>
class Array3: public Array<T,3> {
public:
  // Constructors/Destructor
  Array3();
  Array3(int n1, int n2, int n3);
  Array3(const Array<T,3>& a);

  // Mutators
  void setSize(int n1, int n2, int n3);
  T& operator () (int i1, int i2, int i3);

  // Accessors
  T operator () (int i1, int i2, int i3) const;

  // Helper functions
  int indexOf(int i1, int i2, int i3) const;

  // inherit member data and functions of parent
  using Array<T,3>::x;
  using Array<T,3>::reset;
  using Array<T,3>::operator ();

protected:
  // inherit member data and functions of parent
  using Array<T,3>::dim;
  using Array<T,3>::mul;
  using Array<T,3>::len;
};

// Declare a few common typdefs
typedef Array3<bool> Array3b;
typedef Array3<char> Array3c;
typedef Array3<unsigned char> Array3uc;
typedef Array3<int> Array3i;
typedef Array3<float> Array3f;
typedef Array3<double> Array3d;

//==============================================================================
// Array3<T>
//==============================================================================

// Constructors/Destructor

/** Ctor that does no initalization.
 */
template <class T>
inline Array3<T>::Array3() : Array<T,3>() {}

/** Ctor that starts with given dimensions
 * @param n1, n2, n3 is the dimensions
 */
template <class T>
inline Array3<T>::Array3(int n1, int n2, int n3) : Array<T,3>()  {
  setSize(n1,n2,n3);
}

/** Ctor that initializes an Array3<T> with a Array<T,3>.
*/
template <class T>
inline Array3<T>::Array3(const Array<T,3>& a) : Array<T,3>(a) {
}

// Mutators

/** Set the size of the array
 * @param n1, n2, n3 are the sizes of the array in each dimension
 */
template <class T>
inline void Array3<T>::setSize(int n1, int n2, int n3) {
  Array<T,3>::setSize(Vec3i(n1,n2,n3));
}

/** Returns mutable reference to array element
 * @param i1, i2, i3 are the indices
 * @return a reference to the array element
 */
template <class T>
inline T& Array3<T>::operator () (int i1, int i2, int i3) {
  return x[indexOf(i1,i2,i3)];
}

// Accessors

/** Returns inmutable reference to array element
 * @param i1, i2, i3 are the indices
 * @return the value of the array element
 */
template <class T>
inline T Array3<T>::operator () (int i1, int i2, int i3) const {
  return x[indexOf(i1,i2,i3)];
}

// Helper functions (used internally only)

/** Returns linear index of given array indices
 * @param i1, i2, i3 are the indices
 * @return the linear index in the data array x
 */
template <class T>
inline int Array3<T>::indexOf(int i1, int i2, int i3) const {
#if AR_CHECK_BOUNDS
  if (i1<0 || i1>=dim(0) ||
      i2<0 || i2>=dim(1) ||
      i3<0 || i3>=dim(2)) {
    std::stringstream ss;
    ss << "Error. Indices" << i1 << ", " << i2 << ", " << i3;
    ss << " exceed bounds [0, ";
    ss << dim(0) << "] or [0, " << dim(1);
    ss << "] or [0, " << dim(2) << "].";
    ss << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return i1*mul(0)+i2*mul(1)+i3;
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_ARRAY_H defined
//==============================================================================

