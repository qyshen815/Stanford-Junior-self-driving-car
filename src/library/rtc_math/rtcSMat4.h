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
 * file .......: SMat4.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_SMAT4_H
#define RTC_SMAT4_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec4.h"
#include "rtcSMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class SMat; // MxM Square Matrix
template <class T> class Vec4; // 4d Vector
template <class T> class SMat4; // 4x4 Matrix

/**
 * A 4x4 matrix.
 * A specialization of a square matrix.
 */
template <class T>
class SMat4: public SMat<T,4> {
public:
  // Constructors
  SMat4();
  SMat4(const T* d);
  SMat4(const T diagVal);
  SMat4(const Vec4<T>& diagVec);
  SMat4(const Mat<T,4,4>& m);
  SMat4(const T x11, const T x12, const T x13, const T x14,
        const T x21, const T x22, const T x23, const T x24,
        const T x31, const T x32, const T x33, const T x34,
        const T x41, const T x42, const T x43, const T x44);
  SMat4(const Vec4<T>& q0,const Vec4<T>& q1,
  const Vec4<T>& q2,const Vec4<T>& q3);


  // Casting Operation
  template <class U> SMat4(const Mat<U,4,4>& m);

  // Named Constructors
  static SMat4<T> fromRows(const Vec4<T>& q0, const Vec4<T>& q1, const Vec4<T>& q2, const Vec4<T>& q3);
  static SMat4<T> fromCols(const Vec4<T>& q0, const Vec4<T>& q1, const Vec4<T>& q2, const Vec4<T>& q3);

  // Mutators
  void set(const T x11, const T x12, const T x13, const T x14,
           const T x21, const T x22, const T x23, const T x24,
           const T x31, const T x32, const T x33, const T x34,
           const T x41, const T x42, const T x43, const T x44);
  void setRows(const Vec4<T>& q0, const Vec4<T>& q1, const Vec4<T>& q2, const Vec4<T>& q3);
  void setCols(const Vec4<T>& q0, const Vec4<T>& q1, const Vec4<T>& q2, const Vec4<T>& q3);

  // Data
  using SMat<T,4>::x;
  using SMat<T,4>::set;
};

// Declare a few common typdefs
typedef SMat4<bool> SMat4b;
typedef SMat4<char> SMat4c;
typedef SMat4<unsigned char> SMat4uc;
typedef SMat4<int> SMat4i;
typedef SMat4<float> SMat4f;
typedef SMat4<double> SMat4d;

//==============================================================================
// SMat4<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T>
inline SMat4<T>::SMat4() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length 4
 */
template <class T>
inline SMat4<T>::SMat4(const T* d) : SMat<T,4>(d) {}

/** Ctor that makes a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline SMat4<T>::SMat4(const T diagVal) : SMat<T,4>(diagVal) {}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline SMat4<T>::SMat4(const Vec4<T>& diagVec) : SMat<T,4>(diagVec) {}

/** Ctor that initializes from a Mat<T,4,4>.
 */
template <class T>
inline SMat4<T>::SMat4(const Mat<T,4,4>& m) : SMat<T,4>(m) {}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline SMat4<T>::SMat4(const T x11, const T x12, const T x13, const T x14,
     const T x21, const T x22, const T x23, const T x24,
     const T x31, const T x32, const T x33, const T x34,
     const T x41, const T x42, const T x43, const T x44) {
  set(x11, x12, x13, x14,
      x21, x22, x23, x24,
      x31, x32, x33, x34,
      x41, x42, x43, x44);
}

/** Ctor that initializes matrix from columns.
 */
template <class T>
inline SMat4<T>::SMat4(const Vec4<T>& q0,const Vec4<T>& q1,
     const Vec4<T>& q2, const Vec4<T>& q3) {
  setCols(q0,q1,q2,q3);
}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,4,4>
 */
template <class T> template <class U>
inline SMat4<T>::SMat4(const Mat<U,4,4>& m) : SMat<T,4>(m) {}

// Named Constructors

/** Create matrix from rows.
 */
template <class T>
inline SMat4<T> SMat4<T>::fromRows(const Vec4<T>& q0, const Vec4<T>& q1,
           const Vec4<T>& q2, const Vec4<T>& q3) {
  SMat4<T> m;
  m.setRows(q0,q1,q2,q3);
  return m;
}

/** Create matrix from columns.
 */
template <class T>
inline SMat4<T> SMat4<T>::fromCols(const Vec4<T>& q0, const Vec4<T>& q1,
           const Vec4<T>& q2, const Vec4<T>& q3) {
  SMat4<T> m;
  m.setCols(q0,q1,q2,q3);
  return m;
}

// Mutators

/** Set matrix given entries directly.
 */
template <class T>
inline void SMat4<T>::set(const T x11,const T x12,const T x13,const T x14,
        const T x21,const T x22,const T x23,const T x24,
        const T x31,const T x32,const T x33,const T x34,
        const T x41,const T x42,const T x43,const T x44){
  x[0] = x11; x[1] = x12; x[2] = x13; x[3] = x14;
  x[4] = x21; x[5] = x22; x[6] = x23; x[7] = x24;
  x[8] = x31; x[9] = x32; x[10] = x33; x[11] = x34;
  x[12] = x41; x[13] = x42; x[14] = x43; x[15] = x44;
}

/** Set matrix from rows.
 */
template <class T>
inline void SMat4<T>::setRows(const Vec4<T>& q0, const Vec4<T>& q1,
      const Vec4<T>& q2, const Vec4<T>& q3) {
  setRow(0,q0); setRow(1,q1); setRow(2,q2); setRow(3,q3);
}

/** Set matrix from columns.
 */
template <class T>
inline void SMat4<T>::setCols(const Vec4<T>& q0, const Vec4<T>& q1,
      const Vec4<T>& q2, const Vec4<T>& q3) {
  setCol(0,q0); setCol(1,q1); setCol(2,q2); setCol(3,q3);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_SMAT4_H defined
//==============================================================================

