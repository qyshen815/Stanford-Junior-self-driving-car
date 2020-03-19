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
 * file .......: rtcSMat2.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_SMAT2_H
#define RTC_SMAT2_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"
#include "rtcSMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declaration
template <class T, int M> class SMat; // MxM Square Matrix
template <class T> class Vec2; // 2d Vector
template <class T> class SMat2; // 2x2 Matrix

/**
 * A 2x2 matrix.
 * A specialization of a square matrix.
 */
template <class T>
class SMat2: public SMat<T,2> {
public:
  // Data
  using SMat<T,2>::x;
  using SMat<T,2>::set;

  // Constructors
  SMat2();
  SMat2(const T* d);
  SMat2(const T diagVal);
  SMat2(const Vec2<T>& diagVec);
  SMat2(const Mat<T,2,2>& m);
  SMat2(const T x11, const T x12,
  const T x21, const T x22);
  SMat2(const Vec2<T>& q0, const Vec2<T>& q1);

  // Casting Operation
  template <class U> SMat2(const Mat<U,2,2>& m);

  // Named Constructors
  static SMat2<T> fromRows(const Vec2<T>& v0, const Vec2<T>& v1);
  static SMat2<T> fromCols(const Vec2<T>& v0, const Vec2<T>& v1);

  // Mutators
  void set(const T x11, const T x12, const T x21, const T x22);
  void setRows(const Vec2<T>& v0, const Vec2<T>& v1);
  void setCols(const Vec2<T>& v0, const Vec2<T>& v1);

  // Determinant, inverse, etc.
  T det() const;
  SMat2<T> inverted() const;
  int invert();
};

// Declare a few common typdefs
typedef SMat2<bool> SMat2b;
typedef SMat2<char> SMat2c;
typedef SMat2<unsigned char> SMat2uc;
typedef SMat2<int> SMat2i;
typedef SMat2<float> SMat2f;
typedef SMat2<double> SMat2d;

//==============================================================================
// SMat2<T>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T>
inline SMat2<T>::SMat2() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length 4
 */
template <class T>
inline SMat2<T>::SMat2(const T* d) : SMat<T,2>(d) { }

/** Ctor that makes a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline SMat2<T>::SMat2(const T diagVal) : SMat<T,2>(diagVal) {}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline SMat2<T>::SMat2(const Vec2<T>& diagVec) : SMat<T,2>(diagVec) {}


/** Ctor that initializes from a Mat<T,2,2>.
 */
template <class T>
inline SMat2<T>::SMat2(const Mat<T,2,2>& m) : SMat<T,2>(m) {}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline SMat2<T>::SMat2(const T x11, const T x12,
     const T x21, const T x22) {
  set(x11, x12, x21, x22);
}

/** Ctor that initializes matrix from col vectors.
 */
template <class T>
inline SMat2<T>::SMat2(const Vec2<T>& v0,
     const Vec2<T>& v1) {
  setCols(v0,v1);
}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,2,2> with type cast
 */
template <class T> template <class U>
inline SMat2<T>::SMat2(const Mat<U,2,2>& m) : SMat<T,2>(m) {}

// Named Constructors

/** Construct matrix given the rows
 */
template <class T>
inline SMat2<T> SMat2<T>::fromRows(const Vec2<T>& v0, const Vec2<T>& v1) {
  SMat2<T> m;
  m.setRows(v0,v1);
  return m;
}

/** Construct matrix given the cols
 */
template <class T>
inline SMat2<T> SMat2<T>::fromCols(const Vec2<T>& v0,
           const Vec2<T>& v1) {
  SMat2<T> m;
  m.setCols(v0,v1);
  return m;
}

// Mutators

/** Set the matrix entries directly.
 */
template <class T>
inline void SMat2<T>::set(const T x11, const T x12,
        const T x21, const T x22) {
  x[0] = x11; x[1] = x12;
  x[2] = x21; x[3] = x22;
}

/** Set the rows of the matrix.
 */
template <class T>
inline void SMat2<T>::setRows(const Vec2<T>& v0,
      const Vec2<T>& v1) {
  setRow(0,v0); setRow(1,v1);
}

/** Set the columns of the matrix.
 */
template <class T>
inline void SMat2<T>::setCols(const Vec2<T>& v0,
      const Vec2<T>& v1) {
  setCol(0,v0); setCol(1,v1);
}

// Determinant and Inverse

/** Calculate the determinant.
 */
template <class T>
inline T SMat2<T>::det() const {
  return (x[0]*x[3] - x[1]*x[2]);
}

/** Invert the matrix. (matrix set to its inverse)
 * @throws domain_error when matrix nearly singular
 */
template <class T>
inline SMat2<T> SMat2<T>::inverted() const {
  T d = det();
  if (d == 0.0) {
    throw Exception("SMat2::inverted(): error, can't take inverse of singular matrix.");
  }
  T di = T(1)/d;
  return SMat2<T>(x[3]*di,-x[1]*di,-x[2]*di,x[0]*di);
}

/** Calculate the inverse in place.
 */
template <class T>
inline int SMat2<T>::invert() {
  T d = det();
  if (d == 0.0) {
    throw Exception("SMat2::invert(): error, can't take inverse of singular matrix.");
  }
  T di = T(1)/d;
  set(x[3]*di,-x[1]*di,-x[2]*di,x[0]*di);
  return 0;
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_SMAT2_H defined
//==============================================================================
