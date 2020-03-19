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
 * file .......: rtcVec.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-02-16 17:39:28 -0800 (Mon, 16 Feb 2009) $
 * changed by .: $Author: kls1pal $
 * revision ...: $Revision: 48 $
 */
#ifndef RTC_VEC_H
#define RTC_VEC_H

//== INCLUDES ==================================================================
#include <vector>
#include <rtcIOObject.h>
#include "rtcMath.h"
#include "rtcMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec;        // M-d vector
template <class T, int M, int N> class Mat; // MxN Matrix
template <class T, int M> class SMat;       // MxM Square Matrix

/**
 * An M-Dimensional statically allocated vector.
 * Represents an M dimensional vector of type T
 */
template <class T, int M>
class Vec : public IOObject {
public:
  // Constructors
  Vec();
  Vec(const T* d);
  Vec(const T a);
  Vec(const Vec<T,M>& v);

  // Cast Operation
  template <class U> Vec(const Vec<U,M>& v);

  // Mutators
  void set(const T* d);
  void set(const T a);
  void set(const Vec<T,M>& v);
  Vec<T,M>& operator = (const T a);
  Vec<T,M>& operator = (const T* d);
  Vec<T,M>& operator = (const Vec<T,M>& v);
  template <int N> void setSubVec(const int i, const Vec<T,N>& v);

  // Casting Mutators
  template <class U> void set(const Vec<U,M>& v);
  template <class U> Vec<T,M>& operator = (const Vec<U,M>& v);

  // Accessors
  T operator () (const int i) const;
  T& operator () (const int i);
  T operator [] (const int i) const;
  T& operator [] (const int i);
  T& at(const int i);
  const T& at(const int i) const;

  template <int N> Vec<T,N> getSubVec(const int i) const;

  // Norms and Normalize
  T norm() const;
  T normSqr() const;
  T pnorm(float p) const;
  T normalize();
  Vec<T,M> normalized() const;

  // Reductions: Max/Min, Sum/Product
  T max() const;
  T min() const;
  T sum() const;
  T prod() const;
  Vec<T,M> cumsum() const;

  // General elementwise operations
  void perform(T (*mathFun)(T));
  void perform(T (*mathFun)(T,T), const T arg2);
  void perform(T (*mathFun)(T,T), const Vec<T,M>& v);
  Vec<T,M> performed(T (*mathFun)(T));
  Vec<T,M> performed(T (*mathFun)(T,T), const T arg2);
  Vec<T,M> performed(T (*mathFun)(T,T), const Vec<T,M>& v);
  Vec<T,M> minimize(const Vec<T,M>& other);
  Vec<T,M> maximize(const Vec<T,M>& other);

  // Dot and Outer Products
  T dot(const Vec<T,M>& v) const;
  SMat<T,M> outer(const Vec<T,M>& v) const;
  SMat<T,M> outer() const;

  // Random vectors and sorting
  static Vec<T,M> uniformRand(const T a = T(0), const T b = T(1));
  static Vec<T,M> normalRand(const T mean = T(0), const T stdev = T(1));
  static Vec<T,M> multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov);

  void sort(bool ascending = true);
  Vec<T,M> sorted(bool ascending = true);

  // Addition and subtraction
  Vec<T,M>& add(const Vec<T,M>& v);
  Vec<T,M>& subtract(const Vec<T,M>& v);

  // Addition and subtraction operator
  Vec<T,M> operator + (const Vec<T,M>& v) const;
  void operator += (const Vec<T,M>& v);
  Vec<T,M> operator - (const Vec<T,M>& v) const;
  void operator -= (const Vec<T,M>& v);
  Vec<T,M> operator - () const;
  Vec<T,M> add(const T a);
  Vec<T,M> operator + (const T a) const;
  void operator += (const T a);
  Vec<T,M> subtract(const T a);
  Vec<T,M> operator - (const T a) const;
  void operator -= (const T a);

  // Multiplication and division operator
  Vec<T,M> operator * (const T a) const;
  Vec<T,M> operator / (const T a) const;
  void operator *= (const T a);
  void operator /= (const T a);

  // Equality and inequality tests
  int compareTo(const Vec<T,M>& v) const;
  bool equalTo(const Vec<T,M>& v, const T tol = T(0)) const;

  // Equality and inequality tests operator
  bool operator == (const Vec<T,M>& v) const;
  bool operator != (const Vec<T,M>& v) const;
//    Vec<bool,M> operator == (const Vec<T,M>& v) const;
//    Vec<bool,M> operator != (const Vec<T,M>& v) const;
  Vec<bool,M> operator >= (const Vec<T,M>& v) const;
  Vec<bool,M> operator <= (const Vec<T,M>& v) const;
  Vec<bool,M> operator > (const Vec<T,M>& v) const;
  Vec<bool,M> operator < (const Vec<T,M>& v) const;

  // Element-wise operations
  Vec<T,M> operator * (const Vec<T,M>& v) const;
  Vec<T,M> operator / (const Vec<T,M>& v) const;
  void operator *= (const Vec<T,M>& v);
  void operator /= (const Vec<T,M>& v);

  // Serialization
  bool write(OutputHandler& oh) const;
  bool read(InputHandler& ih);

  //data
  T x[M]; ///< storage array
};

// Global operators for cases where Vec<T,M>
// is the second argument in a binary operator
template <class T, int M> Vec<T,M> operator + (const T a, const Vec<T,M>& v);
template <class T, int M> Vec<T,M> operator - (const T a, const Vec<T,M>& v);
template <class T, int M> Vec<T,M> operator * (const T a, const Vec<T,M>& v);

// ASCII stream IO
template <class T, int M> std::ostream& operator<<(std::ostream& os, const Vec<T,M>& vec);
template <class T, int M> std::istream& operator>>(std::istream& is, Vec<T,M>& vec);

//==============================================================================
// Vec<T,M>
//==============================================================================

// Constructors

/** Ctor that does no initalization.
 */
template <class T, int M>
inline Vec<T,M>::Vec() {}

/** Ctor that initializes elements from an array.
 * @param d pointer to the initalization array
 */
template <class T, int M>
inline Vec<T,M>::Vec(const T* d) {
  set(d);
}

/** Ctor that initializes all elements from a scalar.
 * @param a the value to assign to all elements
 */
template <class T, int M>
inline Vec<T,M>::Vec(const T a) {
  set(a);
}

/** Ctor that initialized from vector of different type
 * @param v is the vector to duplicate
 */
template <class T, int M>
inline Vec<T,M>::Vec(const Vec<T,M>& v) : IOObject() {
  set(v);
}

// Casting Operation

/** Casting Ctor that initializes from passed vector.
 * @param v is the vector to duplicate
 */
template <class T, int M> template <class U>
inline Vec<T,M>::Vec(const Vec<U,M>& v) {
  set<U>(v);
}

// Mutators

/** Set all elements equal to a scalar.
 * @param a the value to assign to all elements
 */
template <class T, int M>
inline void Vec<T,M>::set(const T a) {
  if (a == T(0))
    memset(x,0,M*sizeof(T));
  else
    for (int i=0;i<M;++i)
      x[i] = a;
}

/** Set all elements from an array.
 * @param d pointer to the initalization array
 */
template <class T, int M>
inline void Vec<T,M>::set(const T* d) {
  for (int i=0;i<M;i++) x[i] = d[i];
}

/** Set this vector equal to passed vector
 * @param v the vector to replicate
 */
template <class T, int M>
inline void Vec<T,M>::set(const Vec<T,M>& v) {
  memcpy((void*)x,(void*)v.x,M*sizeof(T));
}

/** Set all elements equal to a scalar.
 * @param a the value to assign to all elements
 */
template <class T, int M>
inline Vec<T,M>& Vec<T,M>::operator = (const T a) {
  set(a);
  return *this;
}

/** Set all elements from an array.
 * @param d pointer to the initalization array
 */
template <class T, int M>
inline Vec<T,M>& Vec<T,M>::operator = (const T* d) {
  set(d);
  return *this;
}

/** Set this vector equal to passed vector
 * @param v the vector to replicate
 */
template <class T, int M>
inline Vec<T,M>& Vec<T,M>::operator = (const Vec<T,M>& v) {
  set(v);
  return *this;
}

/** Set a subvector by replacing elements from i onward with the
 * elements from v.
 * Replaces elements till the end of either vector is reached.
 * @param v the vector from which to copy values
 * @param i the (zero based) index of the start of the subvector
 */
template <class T, int M> template <int N>
inline void Vec<T,M>::setSubVec(const int i, const Vec<T,N>& v) {
  for (int j=i,k=0;j<M && k<N;j++,k++) x[j] = v.x[k];
}

// Casting Mutators

/** Set this vector equal to passed vector with type cast.
 * @param v the vector to replicate
 */
template <class T, int M> template <class U>
inline void Vec<T,M>::set(const Vec<U,M>& v) {
  for (int i=0;i<M;i++) x[i] = T(v.x[i]);
}

/** Set this vector equal to passed vector with type cast.
 * @param v the vector to replicate
 */
template <class T, int M> template <class U>
inline Vec<T,M>& Vec<T,M>::operator = (const Vec<U,M>& v) {
  set<U>(v);
  return *this;
}

// Accessors

/** Get an element of the vector (by value).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline T Vec<T,M>::operator () (const int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
}
#endif
  return x[i];
}

/** Get an element of the vector (by reference).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline T& Vec<T,M>::operator () (const int i) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "&Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
  }
#endif
  return x[i];
}

/** Get an element of the vector (by value).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline T Vec<T,M>::operator [] (const int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
}
#endif
  return x[i];
}

/** Get an element of the vector (by reference).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline T& Vec<T,M>::operator [] (const int i) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "&Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
  }
#endif
  return x[i];
}

/** Get an element of the vector (by reference).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline T& Vec<T,M>::at(const int i) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "&Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
  }
#endif
  return x[i];
}

/** Get an element of the vector (by reference).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T, int M>
inline const T& Vec<T,M>::at(const int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > M) {
    std::cerr << "&Vec<" << M << ">::(" << i << "): index out of range\n";
    std::cerr << std::flush;
    exit(1);
  }
#endif
  return x[i];
}

/** Get the subvector of length N starting at element i
 * @param i the (zero based) index of the first element of subvector
 */
template <class T, int M> template <int N>
inline Vec<T,N> Vec<T,M>::getSubVec(const int i) const {
  Vec<T,N> v(T(0));
  for (int j=i,k=0;j<M && k<N;j++,k++) v.x[k] = x[j];
  return v;
}

// Norms and Normalize

/** Calculate the euclidian norm (2-norm)
 * @return the square root of the sum of squares of the elements
 */
template <class T, int M>
inline T Vec<T,M>::norm() const {
  return T(sqrt(double(normSqr())));
}

/** Calculate the square of the euclidan norm (2-norm).
 * @return the sum of squares of the elements
 */
template <class T, int M>
inline T Vec<T,M>::normSqr() const {
  T sum = T(0);
  for (int i=0;i<M;i++) sum += x[i]*x[i];
  return sum;
}

/** Calculate general p-norms
 * @param p
 * @return the pth root of the sum of pth power of the elmements
 */
template <class T, int M>
inline T Vec<T,M>::pnorm(float p) const {
  T sum = T(0);
  for (int i=0;i<M;i++) sum += pow(fabs((double)x[i]), (double)p);
  return pow((double)sum, (double)1.0/p);
}

/** Normalize a vector.
 * @return the 2-norm of the vector before it was normalized
 */
template <class T, int M>
inline T Vec<T,M>::normalize() {
  T l = norm();
  if (l > T(0)) for (int i=0;i<M;i++) x[i] /= l;
  return l;
}

/** Get a normalized version of the vector.
 * @return the vector normalized
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::normalized() const {
  Vec<T,M> v(x);
  v.normalize();
  return v;
}

// Maximum/Minimum value and Sum

/** Find the maximum value of the vector
 * @returns the maximum value
 */
template <class T, int M>
inline T Vec<T,M>::max() const {
  T m = x[0];
  for (int i=1;i<M;i++) if (x[i]>m) m = x[i];
  return m;
}

/** Find the minimum value of the vector
 * @returns the minimum value
 */
template <class T, int M>
inline T Vec<T,M>::min() const {
  T m = x[0];
  for (int i=1;i<M;i++) if (x[i]<m) m = x[i];
  return m;
}

/** Find the sum of the elements in the vector
 * @returns the sum of the elements in the vector
 */
template <class T, int M>
inline T Vec<T,M>::sum() const {
  T s = T(0);
  for (int i=0;i<M;i++) s += x[i];
  return s;
}

/** Calculates the cumulative sum of the elements in the vector
 * @returns the cumulative of the elements in the vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::cumsum() const {
  Vec<T,M> v;
  T s = T(0);
  for (int i=0;i<M;i++) {
    s += x[i];
    v.x[i]=s;
  }
  return v;
}

/** Find the product of the elements in the vector
 * @returns the product of the elements in the vector
 */
template <class T, int M>
inline T Vec<T,M>::prod() const {
  T p = T(1);
  for (int i=0;i<M;i++) p *= x[i];
  return p;
}

// General elementwise operations

/** Pass each element through the given single-arg math function,
    replacing the current vector with the result.
 */
template <class T, int M>
inline void Vec<T,M>::perform(T (*mathFun)(T)) {
  for (int i=0;i<M;i++) {
    x[i] = (*mathFun)(x[i]);
  }
}

/** Pass each element through the given double-arg math function,
    replacing the current vector with the result.
*/
template <class T, int M>
inline void Vec<T,M>::perform(T (*mathFun)(T,T), const T arg2) {
  for (int i=0;i<M;i++) {
    x[i] = (*mathFun)(x[i], arg2);
  }
}

/** Pass each element through the given double-arg math function,
    replacing the current vector with the result.
*/
template <class T, int M>
inline void Vec<T,M>::perform(T (*mathFun)(T,T), const Vec<T,M>& v) {
  for (int i=0;i<M;i++) {
    x[i] = (*mathFun)(x[i], v.x[i]);
  }
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::performed(T (*mathFun)(T)) {
  Vec<T,M> v;
  for (int i=0;i<M;i++) {
    v.x[i] = (*mathFun)(x[i]);
  }
  return v;
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::performed(T (*mathFun)(T,T), const T arg2) {
  Vec<T,M> v;
  for (int i=0;i<M;i++) {
    v.x[i] = (*mathFun)(x[i], arg2);
  }
  return v;
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::performed(T (*mathFun)(T,T), const Vec<T,M>& vp) {
  Vec<T,M> v;
  for (int i=0;i<M;i++) {
    v.x[i] = (*mathFun)(x[i], vp.x[i]);
  }
  return v;
}

/** Minimize values: same as *this = min(*this, _rhs), but faster
 * @returns a vector of the results
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::minimize(const Vec<T,M>& other) {
  for (int i=0;i<M;i++) {
    if (other.x[i] < x[i]) x[i] = other.x[i];
  }
  return *this;
}

/** Maximize values: same as *this = max(*this, _rhs), but faster
 * @returns a vector of the results
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::maximize(const Vec<T,M>& other) {
  for (int i=0;i<M;i++) {
    if (other.x[i] > x[i]) x[i] = other.x[i];
  }
  return *this;
}

// Dot, Cross, and Outer Products

/** Calculate the dot (inner) product with another vector.
 * @param v the other vector
 * @return the inner product
 */
template <class T, int M>
inline T Vec<T,M>::dot(const Vec<T,M>& v) const {
  T sum = T(0);
  for (int i=0;i<M;i++) sum += x[i]*v.x[i];
  return sum;
}

/** Calculate the outer product with another vector.
 * @param v the other vector
 * @return the outer product matrix
 */
template <class T, int M>
inline SMat<T,M> Vec<T,M>::outer(const Vec<T,M>& v) const {
  SMat<T,M> m;
  for (int i=0,k=0;i<M;i++) for (int j=0;j<M;j++,k++) m.x[k] = x[i]*v.x[j];
  return m;
}

/** Calculate the outer product with itself.
 * @return the outer product matrix
 */
template <class T, int M>
inline SMat<T,M> Vec<T,M>::outer() const {
  SMat<T,M> m;
  for (int i=0,k=0;i<M;i++) for (int j=0;j<M;j++,k++) m.x[k] = x[i]*x[j];
  return m;
}

// Random vectors and sorting

/** Creates vector with samples from a uniform distribution on [a,b].
 * @param a start of range
 * @param b end of range
 * @return vector of uniform samples
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::uniformRand(const T a, const T b) {
  Vec<T,M> v;
  for (int i=0;i<M;i++) v.x[i] = rtc_uniform_rand<T>(a,b);
  return v;
}

/** Create vector with samples from a normal distribution.
 * @param mean mean of normal distribution
 * @param stdev standard deviation of normal distribution
 * @return vector of normal samples
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::normalRand(const T mean, const T stdev) {
  Vec<T,M> v;
  for (int i=0;i<M;i++) v.x[i] = rtc_normal_rand<T>(mean, stdev);
  return v;
}

/** Create vector from a multivariate gaussian distribution.
 * @param mean mean of normal distribution
 * @param cov covariance of normal distribution
 * @return vector of a multivariate gaussian distribution
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov) {
  Vec<T,M> v;
  SMat<T,M> S(cov);
  int n=S.choleskyDecomp();
  S.transpose();
  Vec<T,M> X = normalRand();
  v = mean + S*X;
  return v;
}


/** Sort elements in increasing order.
 * @return sorted vector
 */
template <class T, int M>
inline void Vec<T,M>::sort(bool ascending) {
  int count;
  do {
    count = 0;
    for (int i=0;i<(M-1);i++) {
if (ascending) {
  if (x[i] > x[i+1]) {
    rtc_swap(x[i],x[i+1]); count++;
  }
} else {
  if (x[i] < x[i+1]) {
    rtc_swap(x[i+1],x[i]); count++;
  }
}
    }
  } while (count > 0);
}

/** Create a copy of the vector with elements sorted in increasing order.
 * @return sorted vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::sorted(bool ascending) {
  Vec<T,M> v(x);
  v.sort(ascending);
  return v;
}

// Addition and subtraction

/** Vector-Vector addition.
 */
template <class T, int M>
inline Vec<T,M>& Vec<T,M>::add(const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] = x[i] + v.x[i];
  return *this;
}

/** Vector-Vector subtraction.
 */
template <class T, int M>
inline Vec<T,M>& Vec<T,M>::subtract(const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] = x[i] - v.x[i];
  return *this;
}

// Addition and subtraction operator

/** Vector-Vector addition operator.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator + (const Vec<T,M>& v) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i] + v.x[i];
  return vp;
}

/** Vector-Vector increment operator.
 */
template <class T, int M>
inline void Vec<T,M>::operator += (const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] += v.x[i];
}

/** Vector-Vector subtraction operator.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator - (const Vec<T,M>& v) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i] - v.x[i];
  return vp;
}

/** Vector-Vector decrement operator.
 */
template <class T, int M>
inline void Vec<T,M>::operator -= (const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] -= v.x[i];
}

/** Vector negation operator.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator - () const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = -x[i];
  return vp;
}

/** Vector-Scalar addition.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::add(const T a) {
  for (int i=0;i<M;i++) x[i] = x[i] - a;
  return *this;
}

/** Vector-Scalar addition operator.
 * @param a the scalar to add to each element of vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator + (const T a) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i] + a;
  return vp;
}

/** Vector-Scalar increment operator.
 * @param a the scalar by which to increment each element of vector
 */
template <class T, int M>
inline void Vec<T,M>::operator += (const T a) {
  for (int i=0;i<M;i++) x[i] += a;
}

/** Vector-Scalar subtraction.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::subtract(const T a) {
  for (int i=0;i<M;i++) x[i] = x[i] - a;
  return *this;
}

/** Vector-Scalar subtraction operator.
 * @param a the scalar to subtract from each element of vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator - (const T a) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i] - a;
  return vp;
}

/** Vector-Scalar decrement operator.
 * @param a the scalar by which to decrement each element of vector
 */
template <class T, int M>
inline void Vec<T,M>::operator -= (const T a) {
  for (int i=0;i<M;i++) x[i] -= a;
}

/** Scalar-Vector addition operator.
 * @param a the scalar to add to each element of vector
 * @param v the vector
 */
template <class T, int M>
inline Vec<T,M> operator + (const T a, const Vec<T,M>& v) {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = a + v.x[i];
  return vp;
}

/** Scalar-Vector subtraction operator.
 * @param a the scalar to subtract from each element of vector
 * @param v the vector
 */
template <class T, int M>
inline Vec<T,M> operator - (const T a, const Vec<T,M>& v) {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = a - v.x[i];
  return vp;
}

// Multiplication and division

/** Vector-Scalar multiplication operator.
 * @param a the scalar with which to multiply each element of vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator * (const T a) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i]*a;
  return vp;
}

/** Vector-Scalar division operator.
 * @param a the scalar by which to divide each element of vector
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator / (const T a) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i]/a;
  return vp;
}

/** Vector-Scalar multiplication assignment operator.
 * @param a the scalar with which to multiply each element of vector
 */
template <class T, int M>
inline void Vec<T,M>::operator *= (const T a) {
  for (int i=0;i<M;i++) x[i] *= a;
}

/** Vector-Scalar division assignment operator.
 * @param a the scalar by which to divide each element of vector
 */
template <class T, int M>
inline void Vec<T,M>::operator /= (const T a) {
  for (int i=0;i<M;i++) x[i] /= a;
}

/** Scalar-Vector multiplication operator.
 * @param a the scalar with which to multiply each element of vector
 * @param v the vector
 */
template <class T, int M>
inline Vec<T,M> operator * (const T a, const Vec<T,M>& v) {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = a*v.x[i];
  return vp;
}

// Equality and inequality tests.

/** Element-wise test for equality.
 * @return vector of boolean results
 */
template <class T, int M>
inline bool Vec<T,M>::operator == (const Vec<T,M>& v) const {
  for (int i=0;i<M;i++)
    if (x[i] != v.x[i])
      return(false);
  return true;
}

/** Element-wise test for inequality.
 * @return vector of boolean results
 */
template <class T, int M>
inline bool Vec<T,M>::operator != (const Vec<T,M>& v) const {
  for (int i=0;i<M;i++)
    if (x[i] != v.x[i])
      return(true);
  return false;
}

//  /** Element-wise test for equality.
//   * @return vector of boolean results
//   */
//  template <class T, int M>
//  inline Vec<bool,M> Vec<T,M>::operator == (const Vec<T,M>& v) const {
//    Vec<bool,M> b(false);
//    for (int i=0;i<M;i++) if (x[i] == v.x[i]) b.x[i] = true;
//    return b;
//  }

//  /** Element-wise test for inequality.
//   * @return vector of boolean results
//   */
//  template <class T, int M>
//  inline Vec<bool,M> Vec<T,M>::operator != (const Vec<T,M>& v) const {
//    Vec<bool,M> b(false);
//    for (int i=0;i<M;i++) if (x[i] != v.x[i]) b.x[i] = true;
//    return b;
//  }

/** Element-wise test for greater or equal.
 * @return vector of boolean results
 */
template <class T, int M>
inline Vec<bool,M> Vec<T,M>::operator >= (const Vec<T,M>& v) const {
  Vec<bool,M> b(false);
  for (int i=0;i<M;i++) if (x[i] >= v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less or equal.
 * @return vector of boolean results
 */
template <class T, int M>
inline Vec<bool,M> Vec<T,M>::operator <= (const Vec<T,M>& v) const {
  Vec<bool,M> b(false);
  for (int i=0;i<M;i++) if (x[i] <= v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater.
 * @return vector of boolean results
 */
template <class T, int M>
inline Vec<bool,M> Vec<T,M>::operator > (const Vec<T,M>& v) const {
  Vec<bool,M> b(false);
  for (int i=0;i<M;i++) if (x[i] > v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less.
 * @return vector of boolean results
 */
template <class T, int M>
inline Vec<bool,M> Vec<T,M>::operator < (const Vec<T,M>& v) const {
  Vec<bool,M> b(false);
  for (int i=0;i<M;i++) if (x[i] < v.x[i]) b.x[i] = true;
  return b;
}

/** Compare the entire vector to the passed vector.
 * @returns 1 if all element in this vector are greater than corresponding
 * elements in the passed vector, -1 if they are all less, and 0 otherwise
 */
template <class T, int M>
inline int Vec<T,M>::compareTo(const Vec<T,M>& v) const {
  int g=0, l=0;
  for (int i=0;i<M;i++) {
    if (x[i] < v.x[i]) l++;
    if (x[i] > v.x[i]) g++;
  }
  if (l==M) return -1;
  else if (g==M) return 1;
  else return 0;
}

/** Compare the entire vector to the passed vector.
 * @returns true of all corresponding elements are within the given tolerance
 */
template <class T, int M>
inline bool Vec<T,M>::equalTo(const Vec<T,M>& v, const T tol) const {
  bool t = true;
  for (int i=0;i<M;i++) if (rtc_abs(x[i] - v.x[i]) > tol) t = false;
  return t;
}

// Element-wise multiplication and division

/** Element-wise multiplication.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator * (const Vec<T,M>& v) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i]*v.x[i];
  return vp;
}

/** Element-wise division.
 */
template <class T, int M>
inline Vec<T,M> Vec<T,M>::operator / (const Vec<T,M>& v) const {
  Vec<T,M> vp;
  for (int i=0;i<M;i++) vp.x[i] = x[i]/v.x[i];
  return vp;
}

/** Element-wise multiplication assigment.
 */
template <class T, int M>
inline void Vec<T,M>::operator *= (const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] *= v.x[i];
}

/** Element-wise division assignment.
 */
template <class T, int M>
inline void Vec<T,M>::operator /= (const Vec<T,M>& v) {
  for (int i=0;i<M;i++) x[i] /= v.x[i];
}

// Serialization routines

/** Writes state to a binary stream.
 */
template <class T, int M>
inline bool Vec<T,M>::write(OutputHandler& oh) const {
  return oh.write((char *)(x),M*sizeof(T));
}

/** Restores state from a binary stream.
 */
template <class T, int M>
inline bool Vec<T,M>::read(InputHandler& ih) {
  return ih.read((char *)(x),M*sizeof(T));
}

/** Write state to stream as formated ASCII
 */
template <class T, int M>
std::ostream& operator<<(std::ostream& os, const Vec<T,M>& vec) {
int minFieldWidth = os.precision()+2;

os << "[";
for (int i=0; i<M; ++i)
  os << std::setw(minFieldWidth) << vec.x[i] << " ";
os << "]";

return os;
}

/** Restores state from formated ASCII stream
 */
template <class T, int M>
std::istream& operator>>(std::istream& is, Vec<T,M>& vec) {
  using namespace std;
  vector<T> data;
  string vecString;
  stringstream vecStringStream;

  getline(is, vecString, ']');
  int sPos = (int)vecString.find('[');
  if (sPos == (int)string::npos)
    throw Exception("format error: expecting formated vector to start with '['");

  //erase the starting '['
  //note the ending ']' was removed by the getline function as the delim
  vecString.erase(0,sPos+1);
  trim(vecString);
  vecStringStream.str(vecString);

  //determine num of rows and cols
  int colCount = 0;
  T tmpVal;
  while(vecStringStream.good()){
    vecStringStream >> tmpVal;
    data.push_back(tmpVal);
    ++colCount;
  }

  //check that dimensions agree
  if (colCount != M){
    std::stringstream ss;
    ss << "format error: formated text has " << colCount << " columns";
    ss << " while destination vector has " << M  << " columns" << endl;
    throw Exception(ss.str());
  }

  //copy extracted data
  for (int i=0;i<M;i++){
    vec.x[i] = data[i];
  }

  return is;
}

/**
* handler functions with standard storable interface
*/
template <class T, int M>
bool rtc_write(OutputHandler& oh, const Vec<T,M>& data)
{
  return data.write(oh);
};

/**
* handler functions with standard storable interface
*/
template <class T, int M>
bool rtc_read(InputHandler& ih, Vec<T,M>& data)
{
  return data.read(ih);
};

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VEC_H defined
//==============================================================================
