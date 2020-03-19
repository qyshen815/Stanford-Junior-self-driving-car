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
 * file .......: rtcSMat6.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_SMAT6_H
#define RTC_SMAT6_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "Vec6.h"
#include "SMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class SMat; // MxM Square Matrix
template <class T> class Vec6; // 4d Vector
template <class T> class SMat6; // 4x4 Matrix

//////////////////// SMat6 ////////////////////
/**
 * A 4x4 matrix.
 * A specialization of a square matrix.
 */
template <class T>
class SMat6: public SMat<T,6> {
public:
  // Constructors
  SMat6();
  SMat6(const T* d);
  SMat6(const T diagVal);
  SMat6(const Vec6<T>& diagVec);
  SMat6(const Mat<T,6,6>& m);
  SMat6(const T x11, const T x12, const T x13, const T x14, const T x15, const T x16,
        const T x21, const T x22, const T x23, const T x24, const T x25, const T x26,
        const T x31, const T x32, const T x33, const T x34, const T x35, const T x36,
        const T x41, const T x42, const T x43, const T x44, const T x45, const T x46,
        const T x51, const T x52, const T x53, const T x54, const T x55, const T x56,
        const T x61, const T x62, const T x63, const T x64, const T x65, const T x66);
  SMat6(const Vec6<T>& q0,const Vec6<T>& q1, const Vec6<T>& q2,const Vec6<T>& q3,const Vec6<T>& q4,const Vec6<T>& q5);


  // Casting Operation
  template <class U> SMat6(const Mat<U,6,6>& m);

  // Named Constructors
  static SMat6<T> fromRows(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4, const Vec6<T>& q5);
  static SMat6<T> fromCols(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4, const Vec6<T>& q5);

  // Mutators
  void set(const T x11, const T x12, const T x13, const T x14, const T x15, const T x16,
           const T x21, const T x22, const T x23, const T x24, const T x25, const T x26,
           const T x31, const T x32, const T x33, const T x34, const T x35, const T x36,
           const T x41, const T x42, const T x43, const T x44, const T x45, const T x46,
           const T x51, const T x52, const T x53, const T x54, const T x55, const T x56,
           const T x61, const T x62, const T x63, const T x64, const T x65, const T x66);
  void setRows(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4,const Vec6<T>& q5);
  void setCols(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4,const Vec6<T>& q5);

  // Data
  using SMat<T,6>::x;
  using SMat<T,6>::set;
};

// Declare a few common typdefs
typedef SMat6<bool> SMat6b;
typedef SMat6<char> SMat6c;
typedef SMat6<unsigned char> SMat6uc;
typedef SMat6<int> SMat6i;
typedef SMat6<float> SMat6f;
typedef SMat6<double> SMat6d;


////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
////////////////////////////////////////////////////////////////////////////

//////////////////// SMat6 ////////////////////

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T>
inline SMat6<T>::SMat6() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length 4
 */
template <class T>
inline SMat6<T>::SMat6(const T* d) : SMat<T,6>(d) {}

/** Ctor that makes a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline SMat6<T>::SMat6(const T diagVal) : SMat<T,6>(diagVal) {}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline SMat6<T>::SMat6(const Vec6<T>& diagVec) : SMat<T,6>(diagVec) {}

/** Ctor that initializes from a Mat<T,6,6>.
 */
template <class T>
inline SMat6<T>::SMat6(const Mat<T,6,6>& m) : SMat<T,6>(m) {}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline SMat6<T>::SMat6(const T x11, const T x12, const T x13, const T x14, const T x15, const T x16,
                       const T x21, const T x22, const T x23, const T x24, const T x25, const T x26,
                       const T x31, const T x32, const T x33, const T x34, const T x35, const T x36,
                       const T x41, const T x42, const T x43, const T x44, const T x45, const T x46,
                       const T x51, const T x52, const T x53, const T x54, const T x55, const T x56,
                       const T x61, const T x62, const T x63, const T x64, const T x65, const T x66) {
  set(x11, x12, x13, x14, x15, x16,
      x21, x22, x23, x24, x25, x26,
      x31, x32, x33, x34, x35, x36,
      x41, x42, x43, x44, x45, x46,
      x51, x52, x53, x54, x55, x56,
      x61, x62, x63, x64, x65, x66);
}

/** Ctor that initializes matrix from columns.
 */
template <class T>
inline SMat6<T>::SMat6(const Vec6<T>& q0,const Vec6<T>& q1,const Vec6<T>& q2,const Vec6<T>& q3,const Vec6<T>& q4,const Vec6<T>& q5) {
  setCols(q0,q1,q2,q3,q4,q5);
}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,6,6>
 */
template <class T> template <class U>
inline SMat6<T>::SMat6(const Mat<U,6,6>& m) : SMat<T,6>(m) {}

// Named Constructors

/** Create matrix from rows.
 */
template <class T>
inline SMat6<T> SMat6<T>::fromRows(const Vec6<T>& q0, const Vec6<T>& q1,const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4,const Vec6<T>& q5) {
  SMat6<T> m;
  m.setRows(q0,q1,q2,q3,q4,q5);
  return m;
}

/** Create matrix from columns.
 */
template <class T>
inline SMat6<T> SMat6<T>::fromCols(const Vec6<T>& q0, const Vec6<T>& q1,const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4, const Vec6<T>& q5) {
  SMat6<T> m;
  m.setCols(q0,q1,q2,q3,q4,q5);
  return m;
}

// Mutators

/** Set matrix given entries directly.
 */
template <class T>
inline void SMat6<T>::set(const T x11, const T x12, const T x13, const T x14, const T x15, const T x16,
                          const T x21, const T x22, const T x23, const T x24, const T x25, const T x26,
                          const T x31, const T x32, const T x33, const T x34, const T x35, const T x36,
                          const T x41, const T x42, const T x43, const T x44, const T x45, const T x46,
                          const T x51, const T x52, const T x53, const T x54, const T x55, const T x56,
                          const T x61, const T x62, const T x63, const T x64, const T x65, const T x66){
  x[0]  = x11; x[1]  = x12; x[2]  = x13; x[3]  = x14; x[4]  = x15; x[5]  = x16;
  x[6]  = x21; x[7]  = x22; x[8]  = x23; x[9]  = x24; x[10] = x25; x[11] = x26;
  x[12] = x31; x[13] = x32; x[14] = x33; x[15] = x34; x[16] = x35; x[17] = x36;
  x[18] = x41; x[19] = x42; x[20] = x43; x[21] = x44; x[22] = x45; x[23] = x46;
  x[24] = x51; x[25] = x52; x[26] = x53; x[27] = x54; x[28] = x55; x[29] = x56;
  x[30] = x61; x[31] = x62; x[32] = x63; x[33] = x64; x[34] = x65; x[35] = x66;
}

/** Set matrix from rows.
 */
template <class T>
inline void SMat6<T>::setRows(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4, const Vec6<T>& q5) {
  setRow(0,q0); setRow(1,q1); setRow(2,q2); setRow(3,q3); setRow(4,q4); setRow(5,q5);
}

/** Set matrix from columns.
 */
template <class T>
inline void SMat6<T>::setCols(const Vec6<T>& q0, const Vec6<T>& q1, const Vec6<T>& q2, const Vec6<T>& q3, const Vec6<T>& q4, const Vec6<T>& q5) {
  setCol(0,q0); setCol(1,q1); setCol(2,q2); setCol(3,q3); setCol(4,q4); setCol(5,q5);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_SMAT6_H defined
//==============================================================================
