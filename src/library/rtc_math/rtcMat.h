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
 * file .......: rtcMat.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_MAT_H
#define RTC_MAT_H

//== INCLUDES ==================================================================
#include <rtcIOObject.h>
#include "rtcMath.h"
#include "rtcVec.h"
#include "rtcSMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T, int M, int N> class Mat; // MxN Matrix
template <class T, int M> class SMat; // MxM Matrix

/**
 * An MxN matrix.
 */
template <class T, int M, int N>
class Mat : public IOObject {
public:
  // Data
  T x[M*N]; ///< storage array

  // Constructors
  Mat();
  Mat(const T* d);
  Mat(const T a);
  Mat(const Mat<T,M,N>& m);

  // Cast Operation
  template <class U> Mat(const Mat<U,M,N>& m);

  // Mutators
  void set(const T* d);
  void set(const T a);
  void set(const Mat<T,M,N>& m);
  Mat<T,M,N>& operator = (const T* d);
  Mat<T,M,N>& operator = (const T a);
  Mat<T,M,N>& operator = (const Mat<T,M,N>& m);
  void setRow(const int i, const Vec<T,N>& v);
  void setCol(const int j, const Vec<T,M>& v);
  template <int P, int Q> void setSubMat(const int r, const int c, const Mat<T,P,Q>& m);

  // Casting Mutators
  template <class U> void set(const Mat<U,M,N>& m);
  template <class U> Mat<T,M,N>& operator = (const Mat<U,M,N>& m);

  // Accessors
  T operator () (const int r, const int c) const;
  T& operator () (const int r, const int c);
  Vec<T,N> getRow(int r) const;
  Vec<T,M> getCol(int c) const;
  template <int P, int Q> Mat<T,P,Q> getSubMat(const int i, const int j);

  // Addition and subtraction: <matrix> +/- <matrix>
  Mat<T,M,N>& add(const Mat<T,M,N>& m);
  Mat<T,M,N>& subtract(const Mat<T,M,N>& m);
  Mat<T,M,N> operator + (const Mat<T,M,N>& m) const;
  void operator += (const Mat<T,M,N>& m);
  Mat<T,M,N> operator - (const Mat<T,M,N>& m) const;
  void operator -= (const Mat<T,M,N>& m);
  Mat<T,M,N> operator - () const;

  // Addition and subtraction: <matrix> +/- <scalar>
  Mat<T,M,N>& add(const T a);
  Mat<T,M,N>& subtract(const T a);
  Mat<T,M,N> operator + (const T a) const;
  void operator += (const T a);
  Mat<T,M,N> operator - (const T a) const;
  void operator -= (const T a);

  // Multiplication and division: <matrix> *// <scalar>
  Mat<T,M,N> operator * (const T a) const;
  void operator *= (const T a);
  Mat<T,M,N> operator / (const T a) const;
  void operator /= (const T a);

  // Multiplication: <matrix> * <vector>
  Vec<T,M> operator * (const Vec<T,N>& v) const;
  template <int P> Mat<T,M,P> operator * (const Mat<T,N,P>& m) const;
  void operator *= (const Mat<T,N,N>& m);

  // Equality and inequality tests
  Mat<bool,M,N> operator == (const Mat<T,M,N>& m) const;
  Mat<bool,M,N> operator != (const Mat<T,M,N>& m) const;
  Mat<bool,M,N> operator >= (const Mat<T,M,N>& m) const;
  Mat<bool,M,N> operator <= (const Mat<T,M,N>& m) const;
  Mat<bool,M,N> operator > (const Mat<T,M,N>& m) const;
  Mat<bool,M,N> operator < (const Mat<T,M,N>& m) const;
  int compareTo(const Mat<T,M,N>& m) const;
  bool equalTo(const Mat<T,M,N>& m, const T tol = T(0)) const;

  // Other matrix operations
  Mat<T,N,M> transposed() const;

  // Random matrix
  static Mat<T,M,N> uniformRand(const T a = T(0), const T b = T(1));
  static Mat<T,M,N> normalRand(const T mean = T(0), const T stdev = T(1));
  static Mat<T,M,N> multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov);

  // General elementwise operations
  void perform(T (*mathFun)(T));
  //void perform(T (*mathFun)(T,T), const T arg2);
  //void perform(T (*mathFun)(T,T), const Vec<T,M>& v);
  //Vec<T,M> performed(T (*mathFun)(T));
  //Vec<T,M> performed(T (*mathFun)(T,T), const T arg2);
  //Vec<T,M> performed(T (*mathFun)(T,T), const Vec<T,M>& v);

  // Reductions: Max/Min, Sum/Product
  //T max() const;
  //T min() const;
  Vec<T,N> sum() const;
  //T prod() const;

  // Bilinear Interpolation
  T interpolate(const float i, const float j) const;

  // statistical operations
  Vec<T,N> meanOfRows() const;
  SMat<T,N> covarianceMatrixOfRows() const;

  // Serialization
  virtual bool write(OutputHandler& oh) const;
  virtual bool read(InputHandler& ih);
};

// Global operators to for cases where Mat<T,M,N>
// is the second argument in a binary operator
template <class T, int M, int N> Mat<T,M,N> operator * (const T a, const Mat<T,M,N> &m);
template <class T, int M, int N> Vec<T,N> operator * (const Vec<T,M>& v, const Mat<T,M,N> &m);

// ASCII stream IO
template <class T, int M, int N> std::ostream& operator<<(std::ostream& os, const Mat<T,M,N>& vec);
template <class T, int M, int N> std::istream& operator>>(std::istream& is, Mat<T,M,N>& vec);


//==============================================================================
// Mat<T,M,N>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T, int M, int N>
inline Mat<T,M,N>::Mat() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length M*N
 */
template <class T, int M, int N>
inline Mat<T,M,N>::Mat(const T* d) {
  set(d);
}

/** Ctor that initializes ALL ELEMENTS to a scalar value.
 * @param a the value to which all elements will be set
 */
template <class T, int M, int N>
inline Mat<T,M,N>::Mat(const T a) {
  set(a);
}

/** Ctor that initializes from passed matrix.
 * @param m the matrix to be copied.
 */
template <class T, int M, int N>
inline Mat<T,M,N>::Mat(const Mat<T,M,N>& m) {
  set(m);
}

// Casting Operation

/** Casting Ctor that initializes from passed matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T, int M, int N> template <class U>
inline Mat<T,M,N>::Mat(const Mat<U,M,N>& m) {
  set<U>(m);
}

// Mutators

/** Set from an array.
 * @param d the (row major) data array of length M*N
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::set(const T* d) {
  for (int k=0;k<M*N;k++) x[k] = d[k];
}

/** Set all elements to a scalar value.
 * @param a the value to which all elements will be set
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::set(const T a) {
  if (a == T(0)) memset(x,0,M*N*sizeof(T));
  else for (int i=0;i<M*N;++i) x[i] = a;
}

/** Set from another matrix.
 * @param m the matrix to replicate.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::set(const Mat<T,M,N>& m) {
  memcpy((void*)x,(void*)m.x,M*N*sizeof(T));
}

/** Set from an array.
 * @param d the (row major) data array of length M*N
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::operator = (const T* d) {
  set(d);
  return *this;
}

/** Set all elements to a scalar value.
 * @param a the value to which all elements will be set
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::operator = (const T a) {
  set(a);
  return *this;
}

/** Set from another matrix.
 * @param m the matrix to replicate.
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::operator = (const Mat<T,M,N>& m) {
  set(m);
  return *this;
}

/** Set a row.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of row to set
 * @param v source vector
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::setRow(const int i, const Vec<T,N>& v) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::setRow(" << i;
    ss << ", v): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int j=0,k=i*N;j<N;j++,k++) x[k] = v.x[j];
}

/** Set a column.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param j zero based index of col to set
 * @param v source vector
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::setCol(const int j, const Vec<T,M>& v) {
#if MATMATH_CHECK_BOUNDS
  if (j < 0 || j > N) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::setCol(" << j;
    ss << ", v) out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int i=0,k=j;i<M;i++,k+=N) x[k] = v.x[i];
}

/** Set a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 * @param m matrix to be placed with UL corner at (i,j)
 */
template <class T, int M, int N> template <int P, int Q>
inline void Mat<T,M,N>::setSubMat(const int i, const int j,
          const Mat<T,P,Q>& m) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+P > M || j < 0 || j+Q > N) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::setSubMat(" << i;
    ss << ", " << j << ", m): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int id=i,is=0;id<M && is<P;id++,is++)
    for (int jd=j,js=0;jd<N && js<Q;jd++,js++)
x[id*N+jd] = m.x[is*Q+js];
}

// Casting Mutators

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T, int M, int N> template <class U>
inline void Mat<T,M,N>::set(const Mat<U,M,N>& m) {
  for (int k=0;k<M*N;k++) x[k] = T(m.x[k]);
}

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T, int M, int N> template <class U>
inline Mat<T,M,N>& Mat<T,M,N>::operator = (const Mat<U,M,N>& m) {
  set<U>(m);
}

// Accessors

/** Reference operator (value).
 * Gets an element of the vector by value.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based row index
 * @param j zero based col index
 */
template <class T, int M, int N>
inline T Mat<T,M,N>::operator () (const int i, const int j) const {
#if MATMATH_CHECK_BOUNDS
  if (i<0 || i>M || j<0 || j>N) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::operator (" << i;
    ss << ", " << j << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  return x[i*N + j];
}

/** Reference operator (reference).
 * Gets an element of the vector by reference.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based row index
 * @param j zero based col index
 */
template <class T, int M, int N>
inline T& Mat<T,M,N>::operator () (const int i, const int j) {
#if MATMATH_CHECK_BOUNDS
  if (i<0 || i>M || j<0 || j>N) {
    std::stringstream ss;
    ss << "&Mat<" << M << "," << N << ">::operator (" << i;
    ss << ", " << j << "): index out of range\n";
    ss << std::flush;
    puma::Exception e(ss.str());
    throw e;
  }
#endif
  return x[i*N + j];
}

/** Get a row.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of row to get
 */
template <class T, int M, int N>
inline Vec<T,N> Mat<T,M,N>::getRow(int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i >= M) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::getRow(" << i;
    ss << "): index out of range\n";
    ss << std::cerr << std::flush;
    throw Exception(ss.str());
  }
#endif
  Vec<T,N> v;
  for (int j=0,k=i*N;j<N;j++,k++) v.x[j] = x[k];
  return v;
}

/** Get a column.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param j zero based index of column to get
 */
template <class T, int M, int N>
inline Vec<T,M> Mat<T,M,N>::getCol(int j) const {
#if MATMATH_CHECK_BOUNDS
  if (j < 0 || j >= N) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::getCol(" << j;
    ss << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  Vec<T,M> v;
  for (int i=0,k=j;i<M;i++,k+=N) v.x[i] = x[k];
  return v;
}

/** Get a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 */
template <class T, int M, int N> template <int P, int Q>
inline Mat<T,P,Q> Mat<T,M,N>::getSubMat(const int i, const int j) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+P > M || j < 0 || j+Q > N) {
    std::stringstream ss;
    ss << "Mat<" << M << "," << N << ">::getSubMat(" << i;
    ss << ", " << j << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  Mat<T,P,Q> m(T(0));
  for (int id=i,is=0;id<M && is<P;id++,is++)
    for (int jd=j,js=0;jd<N && js<Q;jd++,js++)
m.x[is*Q+js] = x[id*N+jd];
  return m;
}

// Addition and subtraction: <matrix> +/- <matrix>

/** Addition.
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::add(const Mat<T,M,N>& m) {
  for (int k=0;k<M*N;k++) x[k] = x[k] + m.x[k];
  return *this;
}

/** Subtraction.
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::subtract(const Mat<T,M,N>& m) {
  for (int k=0;k<M*N;k++) x[k] = x[k] - m.x[k];
  return *this;
}

// Addition and subtraction operators: <matrix> +/- <matrix>

/** Addition operator.
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::operator + (const Mat<T,M,N>& m) const {
  Mat<T,M,N> mp;
  for (int k=0;k<M*N;k++) mp.x[k] = x[k] + m.x[k];
  return mp;
}

/** Addition assignment operator.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::operator += (const Mat<T,M,N>& m) {
  for (int k=0;k<M*N;k++) x[k] += m.x[k];
}

/** Subtraction operator.
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::operator - (const Mat<T,M,N>& m) const {
  Mat<T,M,N> mp;
  for (int k=0;k<M*N;k++) mp.x[k] = x[k] - m.x[k];
  return mp;
}

/** Subtraction assignment operator.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::operator -= (const Mat<T,M,N>& m) {
  for (int k=0;k<M*N;k++) x[k] -= m.x[k];
}

/** Negation operator.
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::operator - () const {
  Mat<T,M,N> mp;
  for (int k=0;k<M*N;k++) mp.x[k] = -x[k];
  return mp;
}

// Addition and subtraction: <matrix> +/- <scalar>

/** Addition.
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::add(const T a) {
  for (int k=0;k<M*N;k++) x[k] = x[k] + a;
  return *this;
}

/** Subtraction.
 */
template <class T, int M, int N>
inline Mat<T,M,N>& Mat<T,M,N>::subtract(const T a) {
  for (int k=0;k<M*N;k++) x[k] = x[k] - a;
  return *this;
}

// Addition and subtraction operators: <matrix> +/- <scalar

 /** Addition operator.
  */
 template <class T, int M, int N>
 inline Mat<T,M,N> Mat<T,M,N>::operator + (const T a) const {
   Mat<T,M,N> mp;
   for (int k=0;k<M*N;k++) mp.x[k] = x[k] + a;
   return mp;
 }

 /** Addition assignment operator.
  */
 template <class T, int M, int N>
 inline void Mat<T,M,N>::operator += (const T a) {
   for (int k=0;k<M*N;k++) x[k] += a;
 }

 /** Subtraction operator.
  */
 template <class T, int M, int N>
 inline Mat<T,M,N> Mat<T,M,N>::operator - (const T a) const {
   Mat<T,M,N> mp;
   for (int k=0;k<M*N;k++) mp.x[k] = x[k] - a;
   return mp;
 }

 /** Subtraction assignment operator.
  */
 template <class T, int M, int N>
 inline void Mat<T,M,N>::operator -= (const T a) {
   for (int k=0;k<M*N;k++) x[k] -= a;
 }

// Multiplication and division: <matrix> *// <scalar>

/** Matrix-Scalar multiplication operator.
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::operator * (const T a) const {
  Mat<T,M,N> m;
  for (int k=0;k<M*N;k++) m.x[k] = x[k]*a;
  return m;
}

/** Matrix-Scalar multiplication assignement operator.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::operator *= (const T a) {
  for (int k=0;k<M*N;k++) x[k] *= a;
}

/** Matrix-Scalar division operator.
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::operator / (const T a) const {
  Mat<T,M,N> m;
  for (int k=0;k<M*N;k++) m.x[k] = x[k]/a;
  return m;
}

/** Matrix-Scalar division assignement operator.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::operator /= (const T a) {
  for (int k=0;k<M*N;k++) x[k] /= a;
}

/** Scalar-Matrix multiplication operator.
 */
template <class T, int M, int N>
Mat<T,M,N> operator * (const T a, const Mat<T,M,N> &m) {
  Mat<T,M,N> mp;
  for (int k=0;k<M*N;k++) mp.x[k] = m.x[k]*a;
  return mp;
}

// Multiplication: <matrix> * <vector>

/** Matrix-Vector multiplication operator.
 */
template <class T, int M, int N>
inline Vec<T,M> Mat<T,M,N>::operator * (const Vec<T,N>& v) const {
  Vec<T,M> vp(T(0));
  for (int i=0,k=0;i<M;i++)
    for (int j=0;j<N;j++,k++)
      vp.x[i] += x[k]*v.x[j];
  return vp;
}

/** Vector-Matrix multiplication operator.
 */
template <class T, int M, int N>
Vec<T,N> operator * (const Vec<T,M>& v, const Mat<T,M,N> &m) {
  Vec<T,N> vp(T(0));
  for (int i=0,k=0;i<M;i++)
    for (int j=0;j<N;j++,k++)
      vp.x[j] += m.x[k]*v.x[i];
  return vp;
}

// Multiplication: <matrix> * <matrix>

/** Matrix-Matrix multiplication operator.
 */
template <class T, int M, int N> template <int P>
inline Mat<T,M,P> Mat<T,M,N>::operator * (const Mat<T,N,P>& m) const {
  Mat<T,M,P> mp(T(0));
  for (int i=0;i<M;i++)
    for (int j=0;j<P;j++)
      for (int k=0;k<N;k++)
        mp.x[i*P+j] += x[i*N+k]*m.x[k*P+j];
  return mp;
}


/** Matrix-Matrix multiplication assignement operator.
 */
template <class T, int M, int N>
inline void Mat<T,M,N>::operator *= (const Mat<T,N,N>& m) {
  (*this) = (*this)*m;
}

// Equality and inequality tests.

/** Element-wise test for equality.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator == (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] == m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for inequality.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator != (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] != m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater or equal.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator >= (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] >= m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less or equal.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator <= (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] <= m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator > (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] > m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less.
 * @return matrix of boolean results
 */
template <class T, int M, int N>
inline Mat<bool,M,N> Mat<T,M,N>::operator < (const Mat<T,M,N>& m) const {
  Mat<bool,M,N> b(false);
  for (int i=0;i<M*N;i++) if (x[i] < m.x[i]) b.x[i] = true;
  return b;
}

/** Compare the entire matrix to the passed matrix.
 * @returns 1 if all element in this matrix are greater than corresponding
 * elements in the passed matrix, -1 if they are all less, and 0 otherwise
 */
template <class T, int M, int N>
inline int Mat<T,M,N>::compareTo(const Mat<T,M,N>& m) const {
  int g=0, l=0;
  for (int i=0;i<M*N;i++) {
    if (x[i] < m.x[i]) l++;
    if (x[i] > m.x[i]) g++;
  }
  if (l==(M*N)) return -1;
  else if (g==(M*N)) return 1;
  else return 0;
}

/** Compare the entire matrix to the passed matrix.
 * @returns true of all corresponding elements are within the given tolerance
 */
template <class T, int M, int N>
inline bool Mat<T,M,N>::equalTo(const Mat<T,M,N>& m, const T tol) const {
  bool t = true;
  for (int i=0;i<M*N;i++) if (fabs(x[i] - m.x[i]) > tol) t = false;
  return t;
}

// Transpose

/** Return the transpose of the matrix without modifying matrix.
 */
template <class T, int M, int N>
inline Mat<T,N,M> Mat<T,M,N>::transposed() const {
  Mat<T,N,M> mp;
  for (int i=0;i<M;i++)
    for (int j=0;j<N;j++)
      mp.x[j*M+i] = x[i*N+j];
  return mp;
}

// Random matrices

/** Create matrix of samples from a uniform distribution on [a,b].
 * @param a start of range
 * @param b end of range
 * @return matrix of uniform samples
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::uniformRand(const T a, const T b) {
  Mat<T,M,N> m;
  for (int i=0;i<M*N;i++) m.x[i] = rtc_uniform_rand<T>(a,b);
  return m;
}

/** Create matrix of samples from a normal distribution.
 * @param mean mean of normal distribution
 * @param stdev standard deviation of normal distribution
 * @return matrix of normal samples
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::normalRand(const T mean, const T stdev) {
  Mat<T,M,N> m;
  for (int i=0;i<M*N;i++) m.x[i] = rtc_normal_rand<T>(mean,stdev);
  return m;
}

/** Create matrix of samples from a multivariate gaussian distribution.
 * @param mean mean of normal distribution
 * @param cov covariance normal distribution
 * @return matrix of normal samples
 */
template <class T, int M, int N>
inline Mat<T,M,N> Mat<T,M,N>::multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov) {
  Mat<T,M,N> m;
  SMat<T,M> S(cov);
  int n=S.choleskyDecomp();
  S.transpose();
  Mat<T,M,N> X = normalRand();
  for(int j=0;j<N;++j) m.setCol(j,mean);
  m = m + S*X;
  return m;
}

// General elementwise operations

/** Pass each element through the given single-arg math function,
replacing the current vector with the result.
*/
template <class T, int M, int N>
inline void Mat<T,M,N>::perform(T (*mathFun)(T)) {
  for (int i=0;i<M*N;i++) x[i] = (*mathFun)(x[i]);
}

/** Find the sum of the elements in each column in the matrix
 * @returns the sum of the elements in each column in the matrix
 */
template <class T, int M, int N>
inline Vec<T,N> Mat<T,M,N>::sum() const {
  Vec<T,N> s(T(0));
  for (int j=0;j<N;j++)
    for (int i=0;i<M;i++) s.x[j] += x[i*N+j];
  return s;
}

/**Use bilinear interpolation to approximate values
 * between the elements of matrices.
 * @returns the bilinear interpolation at i,j
 */
template <class T, int M, int N>
inline T Mat<T,M,N>::interpolate(const float i, const float j) const {
  const int truncR = rtc_clamp(static_cast<int>(i),0,M-1);
  const int truncR1 = rtc_clamp(truncR+1,0,M-1);
  const float fractR = i - static_cast<float>(truncR);
  const int truncC = rtc_clamp(static_cast<int>(j),0,N-1);
  const int truncC1 = rtc_clamp(truncC+1,0,N-1);
  const float fractC = j - static_cast<float>(truncC);

  // do the interpolation
  const T syx = x[truncR*N+truncC];
  const T syx1 = x[truncR*N+truncC1];
  const T sy1x = x[truncR1*N+truncC];
  const T sy1x1 = x[truncR1*N+truncC1];
  // normal interpolation within range
  const T tmp1 = syx  + (syx1-syx)*T(fractC);
  const T tmp2 = sy1x + (sy1x1-sy1x)*T(fractC);
  return (tmp1 + (tmp2-tmp1)*T(fractR));
}

/** The meanOfRows means the mean of all rows, i.e. a row vector containing
  * (for the arithmetical mean) the sum of all row vectors divided by the
  * number of rows.
  * @returns the mean of all rows
  */
template <class T, int M, int N>
inline Vec<T,N> Mat<T,M,N>::meanOfRows() const {
  Vec<T,N> m(T(0));
  for (int j=0;j<N;j++)
    for (int i=0;i<M;i++) m.x[j] += x[i*N+j];
  m/=static_cast<T>(M);
  return m;
}

/*
  * This is a new, faster version, written by Gu Xin:
  */
template <class T, int M, int N>
inline SMat<T,N> Mat<T,M,N>::covarianceMatrixOfRows() const {
  SMat<T,N> dest(T(0));

  //Implementation for the sum of Matrix[X];
  if (M<2) {  //just one row
   throw Exception("matrix has to have more than one row");
  }

  // mean vector
  Vec<T,N> mu(T(0));

  /*
  * the loop gets out the result of the Matrix[X];
  * Matrix[X]=sum(Matrix[X(i)]);-- i from 0 to n-1--
  */
  for(int i=0;i<M;i++){
   Vec<T,N> tmpV = getRow(i);
   mu.add(tmpV);
   SMat<T,N> tmp = tmpV.outer(tmpV);
   dest.add(tmp);          //add the Matrix[X(i)] to Matrix[sumMatrix];
  }
  //End of the Implementation for the sum of Matrix[X];

  //Implementation for the Matrix[meanOf];
  mu/=static_cast<T>(M);

  SMat<T,N> tmp = mu.outer(mu);
  tmp *= static_cast<T>(M);

  //get the result of the difference from Matrix[X] and Matrix[meanOf];
  dest.subtract(tmp);
  dest/=static_cast<T>(M-1);
  return dest;
}


// Serialization routines

/** Write state to a stream.
 */
template <class T, int M, int N>
inline bool Mat<T,M,N>::write(OutputHandler& oh) const {
  return oh.stream().write((char *)(x),M*N*sizeof(T));
}

/** Restore state from a stream.
 */
template <class T, int M, int N>
inline bool Mat<T,M,N>::read(InputHandler& ih) {
  return ih.stream().read((char *)(x),M*N*sizeof(T));
}

/** Write state to stream as formated ASCII
 */
template <class T, int M, int N>
std::ostream& operator<<(std::ostream& os, const Mat<T,M,N>& mat) {

  int minFieldWidth = os.precision()+2;

  //case with 1 row
  if (M == 1){
    os << "[" ;
    for (int i=0; i<N; ++i)
      os << std::setw(minFieldWidth) << mat.x[i] << " ";

    os << "]" << std::endl;
  }
  //case with 2+ rows
  else{
    //write first row
    os << "[" ;
    for (int j=0; j<N-1; ++j){
      os << std::setw(minFieldWidth) << mat.x[j] << " ";
    }
    os << std::setw(minFieldWidth) << mat.x[N-1] << ";" << std::endl;
    //write middle rows
    for (int i=1;i<M-1;++i){
      for (int j=0;j<N;++j){
        os << " " << std::setw(minFieldWidth) << mat.x[i*N+j];
        }
      os << ";" << std::endl;
      }
    //write last row
    for (int j=0; j<N; ++j){
      os << " " << std::setw(minFieldWidth) << mat.x[N*(M-1)+j];
    }
    os << "]" << std::endl;
  }

  return os;
}

/** Restores state data from formated ASCII stream
 */
template <class T, int M, int N>
std::istream& operator>>(std::istream& is, Mat<T,M,N>& mat){
  using namespace std;
  vector<T> data;
  string matString;
  stringstream matStringStream;
  string rowString;
  stringstream rowStringStream;

  getline(is, matString, ']');
  int sPos = matString.find('[');
  if (sPos == (int)string::npos)
    throw Exception("format error: expecting formated matrix to start with '['");

  //erase the starting '['
  //note the ending ']' was removed by the getline function as the delim
  matString.erase(0,sPos+1);
  matStringStream.str(matString);

  //determine num of rows and cols
  int rowCount = 0, colCount = 0;
  int cols = -1;
  float tmpVal;
  while(getline(matStringStream, rowString, ';')){
    rowStringStream << rowString;

    colCount = 0;
    while(rowStringStream.good()){
      rowStringStream >> tmpVal;
      data.push_back(tmpVal);
      ++colCount;
    }
    rowStringStream.clear();

    //check that we have same num of entries in each row
    if (cols == -1)
      cols = colCount;
    else{
      if (colCount != cols){
        throw Exception("format error: different number of elements in rows.");
      }
    }

    ++rowCount;
  }

  //check that dimensions agree
  if (rowCount != M || colCount != N){
    std::stringstream ss;
    ss << "format error: formated text is " << rowCount << "x" << colCount;
    ss << " while destination matrix is " << M << "x" << N << endl;
    throw Exception(ss.str());
  }

  //copy extracted data
  for (int i=0;i<M*N;i++){
    mat.x[i] = data[i];
  }

  return is;
}

/**
* handler functions with standard storable interface
*/
template <class T, int M, int N>
bool rtc_write(OutputHandler& oh, const Mat<T,M,N>& data)
{
  return data.write(oh);
};

/**
* handler functions with standard storable interface
*/
template <class T, int M, int N>
bool rtc_read(InputHandler& ih, Mat<T,M,N>& data)
{
  return data.read(ih);
};

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_MAT_H defined
//==============================================================================
