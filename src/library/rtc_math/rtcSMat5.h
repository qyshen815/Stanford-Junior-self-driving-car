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
 * file .......: rtcSMat5.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_SMAT5_H
#define RTC_SMAT5_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "Vec5.h"
#include "SMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class SMat; // MxM Square Matrix
template <class T> class Vec5; // 4d Vector
template <class T> class SMat5; // 4x4 Matrix

/**
 * A 4x4 matrix.
 * A specialization of a square matrix.
 */
template <class T>
class SMat5: public SMat<T,5> {
public:
  // Data
  using SMat<T,5>::x;
  using SMat<T,5>::set;

  // Constructors
  SMat5();
  SMat5(const T* d);
  SMat5(const T diagVal);
  SMat5(const Vec5<T>& diagVec);
  SMat5(const Mat<T,5,5>& m);
  SMat5(const T x11, const T x12, const T x13, const T x14, const T x15,
        const T x21, const T x22, const T x23, const T x24, const T x25,
        const T x31, const T x32, const T x33, const T x34, const T x35,
        const T x41, const T x42, const T x43, const T x44, const T x45,
        const T x51, const T x52, const T x53, const T x54, const T x55);
  SMat5(const Vec5<T>& q0,const Vec5<T>& q1, const Vec5<T>& q2,const Vec5<T>& q3,const Vec5<T>& q4);


  // Casting Operation
  template <class U> SMat5(const Mat<U,5,5>& m);

  // Named Constructors
  static SMat5<T> fromRows(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4);
  static SMat5<T> fromCols(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4);

  // Mutators
  void set(const T x11, const T x12, const T x13, const T x14, const T x15,
           const T x21, const T x22, const T x23, const T x24, const T x25,
           const T x31, const T x32, const T x33, const T x34, const T x35,
           const T x41, const T x42, const T x43, const T x44, const T x45,
           const T x51, const T x52, const T x53, const T x54, const T x55);
  void setRows(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4);
  void setCols(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4);
}; // end class SMat5<T>

// Declare a few common typdefs
typedef SMat5<bool> SMat5b;
typedef SMat5<char> SMat5c;
typedef SMat5<unsigned char> SMat5uc;
typedef SMat5<int> SMat5i;
typedef SMat5<float> SMat5f;
typedef SMat5<double> SMat5d;


////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
////////////////////////////////////////////////////////////////////////////

//////////////////// SMat5 ////////////////////

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T>
inline SMat5<T>::SMat5() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length 4
 */
template <class T>
inline SMat5<T>::SMat5(const T* d) : SMat<T,5>(d) {}

/** Ctor that makes a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline SMat5<T>::SMat5(const T diagVal) : SMat<T,5>(diagVal) {}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline SMat5<T>::SMat5(const Vec5<T>& diagVec) : SMat<T,5>(diagVec) {}

/** Ctor that initializes from a Mat<T,5,5>.
 */
template <class T>
inline SMat5<T>::SMat5(const Mat<T,5,5>& m) : SMat<T,5>(m) {}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline SMat5<T>::SMat5(const T x11, const T x12, const T x13, const T x14, const T x15,
                       const T x21, const T x22, const T x23, const T x24, const T x25,
                       const T x31, const T x32, const T x33, const T x34, const T x35,
                       const T x41, const T x42, const T x43, const T x44, const T x45,
                       const T x51, const T x52, const T x53, const T x54, const T x55) {
  set(x11, x12, x13, x14, x15,
      x21, x22, x23, x24, x25,
      x31, x32, x33, x34, x35,
      x41, x42, x43, x44, x45,
      x51, x52, x53, x54, x55);
}

/** Ctor that initializes matrix from columns.
 */
template <class T>
inline SMat5<T>::SMat5(const Vec5<T>& q0,const Vec5<T>& q1,const Vec5<T>& q2,const Vec5<T>& q3,const Vec5<T>& q4) {
  setCols(q0,q1,q2,q3,q4);
}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,5,5>
 */
template <class T> template <class U>
inline SMat5<T>::SMat5(const Mat<U,5,5>& m) : SMat<T,5>(m) {}

// Named Constructors

/** Create matrix from rows.
 */
template <class T>
inline SMat5<T> SMat5<T>::fromRows(const Vec5<T>& q0, const Vec5<T>& q1,const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4) {
  SMat5<T> m;
  m.setRows(q0,q1,q2,q3,q4);
  return m;
}

/** Create matrix from columns.
 */
template <class T>
inline SMat5<T> SMat5<T>::fromCols(const Vec5<T>& q0, const Vec5<T>& q1,const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4) {
  SMat5<T> m;
  m.setCols(q0,q1,q2,q3,q4);
  return m;
}

// Mutators

/** Set matrix given entries directly.
 */
template <class T>
inline void SMat5<T>::set(const T x11, const T x12, const T x13, const T x14, const T x15,
                          const T x21, const T x22, const T x23, const T x24, const T x25,
                          const T x31, const T x32, const T x33, const T x34, const T x35,
                          const T x41, const T x42, const T x43, const T x44, const T x45,
                          const T x51, const T x52, const T x53, const T x54, const T x55){
  x[0]  = x11; x[1]  = x12; x[2]  = x13; x[3]  = x14; x[4]  = x15;
  x[5]  = x21; x[6]  = x22; x[7]  = x23; x[8]  = x24; x[9]  = x25;
  x[10] = x31; x[11] = x32; x[12] = x33; x[13] = x34; x[14] = x35;
  x[15] = x41; x[16] = x42; x[17] = x43; x[18] = x44; x[19] = x45;
  x[20] = x51; x[21] = x52; x[22] = x53; x[23] = x54; x[24] = x55;
}

/** Set matrix from rows.
 */
template <class T>
inline void SMat5<T>::setRows(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4) {
  setRow(0,q0); setRow(1,q1); setRow(2,q2); setRow(3,q3); setRow(4,q4);
}

/** Set matrix from columns.
 */
template <class T>
inline void SMat5<T>::setCols(const Vec5<T>& q0, const Vec5<T>& q1, const Vec5<T>& q2, const Vec5<T>& q3, const Vec5<T>& q4) {
  setCol(0,q0); setCol(1,q1); setCol(2,q2); setCol(3,q3); setCol(4,q4);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_SMAT5_H defined
//==============================================================================
