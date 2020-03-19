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
 * file .......: rtcVarVec.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VARVEC_H
#define RTC_VARVEC_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcArray1.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T> class VarVec; // len-d vector
template <class T> class VarSMat; // MxM Square Matrix

/**
 * A dynamically allocated vector of type T
 */
template <class T>
class VarVec : public Array1<T> {
public:
  // Constructors
  VarVec();
  VarVec(int len);
  VarVec(int len, const T* d);
  VarVec(int len, const T a);
  VarVec(const Array1<T>& a);

  // Cast Operation
  template <class U> VarVec(const VarVec<U>& v);
  template <class U, int N> VarVec(const Vec<U,N>& v);

  // Mutators
  void set(const T* d);
  void set(const T a);
  void set(const VarVec<T>& v);
  VarVec<T>& operator = (const T a);
  VarVec<T>& operator = (const T* d);
  VarVec<T>& operator = (const VarVec<T>& v);
  void setSubVec(const int i, const VarVec<T>& v);

  // Casting Mutators
  template <class U> void set(const VarVec<U>& v);
  template <class U, int N> void set(const Vec<U,N>& v);
  template <class U> VarVec<T>& operator = (const VarVec<U>& v);

  // Accessors
  T operator [] (const int i) const;
  T& operator [] (const int i);
  VarVec<T> getSubVec(const int i, int sub_len) const;

  // Norms and Normalize
  T norm() const;
  T normSqr() const;
  T pnorm(float p) const;
  T normalize();
  VarVec<T> normalized() const;

  // Reductions: Max/Min, Sum/Product
  T max() const;
  T min() const;
  T sum() const;
  T prod() const;
  VarVec<T> cumsum() const;

  // General elementwise operations
  void perform(T (*mathFun)(T));
  void perform(T (*mathFun)(T,T), const T arg2);
  void perform(T (*mathFun)(T,T), const VarVec<T>& v);
  VarVec<T> performed(T (*mathFun)(T));
  VarVec<T> performed(T (*mathFun)(T,T), const T arg2);
  VarVec<T> performed(T (*mathFun)(T,T), const VarVec<T>& v);
  VarVec<T> minimize(const VarVec<T>& other);
  VarVec<T> maximize(const VarVec<T>& other);

  // Dot and Outer Products
  T dot(const VarVec<T>& v) const;
  VarSMat<T> outer(const VarVec<T>& v) const;
  VarSMat<T> outer() const;

  // Random vectors and sorting
//    static VarVec<T> uniformRand(const T a = T(0), const T b = T(1));
//    static VarVec<T> normalRand(const T mean = T(0), const T stdev = T(1));
//    static VarVec<T> multivariateGauss(const VarVec<T>& mean, const SMat<T,len>& cov);

  void sort(bool ascending = true);
  VarVec<T> sorted(bool ascending = true);

  // Addition and subtraction
  VarVec<T>& add(const VarVec<T>& v);
  void addSubVec(const int i, const VarVec<T>& v);
  VarVec<T>& subtract(const VarVec<T>& v);
  void subtractSubVec(const int i, const VarVec<T>& v);

  // Addition and subtraction operator
  VarVec<T> operator + (const VarVec<T>& v) const;
  void operator += (const VarVec<T>& v);
  VarVec<T> operator - (const VarVec<T>& v) const;
  void operator -= (const VarVec<T>& v);
  VarVec<T> operator - () const;
  VarVec<T> add(const T a);
  VarVec<T> operator + (const T a) const;
  void operator += (const T a);
  VarVec<T> subtract(const T a);
  VarVec<T> operator - (const T a) const;
  void operator -= (const T a);

  // Multiplication and division operator
  VarVec<T> operator * (const T a) const;
  VarVec<T> operator / (const T a) const;
  void operator *= (const T a);
  void operator /= (const T a);

  // Equality and inequality tests
  int compareTo(const VarVec<T>& v) const;
  bool equalTo(const VarVec<T>& v, const T tol = T(0)) const;

  // Equality and inequality tests operator
  bool operator == (const VarVec<T>& v) const;
  bool operator != (const VarVec<T>& v) const;
//    VarVec<bool> operator == (const VarVec<T>& v) const;
//    VarVec<bool> operator != (const VarVec<T>& v) const;
  VarVec<bool> operator >= (const VarVec<T>& v) const;
  VarVec<bool> operator <= (const VarVec<T>& v) const;
  VarVec<bool> operator > (const VarVec<T>& v) const;
  VarVec<bool> operator < (const VarVec<T>& v) const;

  // Element-wise operations
  VarVec<T> operator * (const VarVec<T>& v) const;
  VarVec<T> operator / (const VarVec<T>& v) const;
  void operator *= (const VarVec<T>& v);
  void operator /= (const VarVec<T>& v);

  // Serialization
  bool write(std::ostream& os) const;
  bool read(std::istream& is);

  // inherit member data and functions of parent
  using Array1<T>::x;
  using Array1<T>::reset;
  using Array1<T>::at;
  using Array1<T>::setSize;

protected:
  // inherit member data and functions of parent
  using Array1<T>::dim;
  using Array1<T>::mul;
  using Array1<T>::len;
};

// Declare a few common typdefs
typedef VarVec<bool> VarVecb;
typedef VarVec<char> VarVecc;
typedef VarVec<unsigned char> VarVecuc;
typedef VarVec<int> VarVeci;
typedef VarVec<float> VarVecf;
typedef VarVec<double> VarVecd;

// Global operators for cases where VarVec<T>
// is the second argument in a binary operator
template <class T> VarVec<T> operator + (const T a, const VarVec<T>& v);
template <class T> VarVec<T> operator - (const T a, const VarVec<T>& v);
template <class T> VarVec<T> operator * (const T a, const VarVec<T>& v);

// ASCII stream IO
template <class T> std::ostream& operator<<(std::ostream& os, const VarVec<T>& vec);
template <class T> std::istream& operator>>(std::istream& is, VarVec<T>& vec);

//==============================================================================
// VarVec<T>
//==============================================================================

// Constructors

/** default Ctor
*/
template <class T>
inline VarVec<T>::VarVec() : Array1<T>() {}

/** Ctor that does no initalization.
 */
template <class T>
inline VarVec<T>::VarVec(int _len) : Array1<T>(_len) {}

/** Ctor that initializes elements from an array.
 * @param _len size of the vector
 * @param d pointer to the initalization array
 */
template <class T>
inline VarVec<T>::VarVec(int _len, const T* d) : Array1<T>(_len,d) {}

/** Ctor that initializes all elements from a scalar.
 * @param _len size of the vector
 * @param a the value to assign to all elements
 */
template <class T>
inline VarVec<T>::VarVec(int _len, const T a) : Array1<T>(_len,a) {}

/** Copy Ctor
 * @param a is the array to duplicate
 */
template <class T>
inline VarVec<T>::VarVec(const Array1<T>& a) : Array1<T>(a) {}

// Casting Operation

/** Casting Ctor that initializes from passed vector.
 * @param v is the vector to duplicate
 */
template <class T> template <class U>
inline VarVec<T>::VarVec(const VarVec<U>& v) {
  set<U>(v);
}

/** Casting Ctor that initializes from passed vector with type cast.
 * @param v is the vector to duplicate
 */
template <class T> template <class U, int N>
inline VarVec<T>::VarVec(const Vec<U,N>& v) : Array1<T>() {
  setSize(N);
  set<U,N>(v);
}

// Mutators

/** Set all elements equal to a scalar.
 * @param a the value to assign to all elements
 */
template <class T>
inline void VarVec<T>::set(const T a) {
  Array1<T>::set(a);
}

/** Set all elements from an array.
 * @param d pointer to the initalization array
 */
template <class T>
inline void VarVec<T>::set(const T* d) {
  Array1<T>::set(d);
}

/** Set this vector equal to passed vector
 * @param v the vector to replicate
 */
template <class T>
inline void VarVec<T>::set(const VarVec<T>& v) {
  if(len!=v.len) setSize(v.len);
  Array1<T>::set(v);
}

/** Set a subvector by replacing elements from i onward with the
 * elements from v.
 * Replaces elements till the end of either vector is reached.
 * @param v the vector from which to copy values
 * @param i the (zero based) index of the start of the subvector
 */
template <class T>
inline void VarVec<T>::setSubVec(const int i, const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+v.len > len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::setSubVec(" << i << ", ";
    ss << "VarVec<" << v.len << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int j=i,k=0;j<len && k<v.len;j++,k++) x[j] = v.x[k];
}

/** Set all elements equal to a scalar.
 * @param a the value to assign to all elements
 */
template <class T>
inline VarVec<T>& VarVec<T>::operator = (const T a) {
  set(a);
  return *this;
}

/** Set all elements from an array.
 * @param d pointer to the initalization array
 */
template <class T>
inline VarVec<T>& VarVec<T>::operator = (const T* d) {
  set(d);
  return *this;
}

/** Set this vector equal to passed vector
 * @param v the vector to replicate
 */
template <class T>
inline VarVec<T>& VarVec<T>::operator = (const VarVec<T>& v) {
  set(v);
  return *this;
}
// Casting Mutators

/** Set this vector equal to passed vector with type cast.
 * @param v the vector to replicate
 */
template <class T> template <class U>
inline void VarVec<T>::set(const VarVec<U>& v) {
  if(len!=v.len) setSize(v.len);
  for (int i=0;i<len;i++) x[i] = T(v.x[i]);
}

/** Set from another matrix with type cast.
 * @param v is the vector to duplicate
 */
template <class T> template <class U, int N>
inline void VarVec<T>::set(const Vec<U,N>& v) {
  if(len!=N) setSize(N);
  for (int k=0;k<len;k++) x[k] = T(v.x[k]);
}

/** Set this vector equal to passed vector with type cast.
 * @param v the vector to replicate
 */
template <class T> template <class U>
inline VarVec<T>& VarVec<T>::operator = (const VarVec<U>& v) {
  set<U>(v);
  return *this;
}

// Accessors

/** Get an element of the vector (by value).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T>
inline T VarVec<T>::operator [] (const int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > len) {
    std::stringstream ss;
    ss << "Vec<" << len << ">::(" << i << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
}
#endif
  return x[i];
}

/** Get an element of the vector (by reference).
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i the (zero based) index into the vector
 */
template <class T>
inline T& VarVec<T>::operator [] (const int i) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i > len) {
    std::stringstream ss;
    ss << "&Vec<" << len << ">::(" << i << "): index out of range";
    throw Exception(ss.str());
  }
#endif
  return x[i];
}

/** Get the subvector of length N starting at element i
 * @param i the (zero based) index of the first element of subvector
 * @param sub_len the length of the subvector
 * @return subvector of length sub_len
 */
template <class T>
inline VarVec<T> VarVec<T>::getSubVec(const int i, int sub_len) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+sub_len > len) {
    std::stringstream ss;
    ss << "&Vec<" << len << ">::(" << i << "): index out of range";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> v(sub_len,T(0));
  for (int j=i,k=0;j<len && k<sub_len;j++,k++) v.x[k] = x[j];
  return v;
}

// Norms and Normalize

/** Calculate the euclidian norm (2-norm)
 * @return the square root of the sum of squares of the elements
 */
template <class T>
inline T VarVec<T>::norm() const {
  return T(sqrt(double(normSqr())));
}

/** Calculate the square of the euclidan norm (2-norm).
 * @return the sum of squares of the elements
 */
template <class T>
inline T VarVec<T>::normSqr() const {
  T sum = T(0);
  for (int i=0;i<len;i++) sum += x[i]*x[i];
  return sum;
}

/** Calculate general p-norms
 * @param p
 * @return the pth root of the sum of pth power of the elmements
 */
template <class T>
inline T VarVec<T>::pnorm(float p) const {
  T sum = T(0);
  for (int i=0;i<len;i++) sum += pow(fabs((double)x[i]), (double)p);
  return pow((double)sum, (double)1.0/p);
}

/** Normalize a vector.
 * @return the 2-norm of the vector before it was normalized
 */
template <class T>
inline T VarVec<T>::normalize() {
  T l = norm();
  if (l > T(0)) for (int i=0;i<len;i++) x[i] /= l;
  return l;
}

/** Get a normalized version of the vector.
 * @return the vector normalized
 */
template <class T>
inline VarVec<T> VarVec<T>::normalized() const {
  VarVec<T> v(len,x);
  v.normalize();
  return v;
}

// Maximum/Minimum value and Sum

/** Find the maximum value of the vector
 * @returns the maximum value
 */
template <class T>
inline T VarVec<T>::max() const {
  T m = x[0];
  for (int i=1;i<len;i++) if (x[i]>m) m = x[i];
  return m;
}

/** Find the minimum value of the vector
 * @returns the minimum value
 */
template <class T>
inline T VarVec<T>::min() const {
  T m = x[0];
  for (int i=1;i<len;i++) if (x[i]<m) m = x[i];
  return m;
}

/** Find the sum of the elements in the vector
 * @returns the sum of the elements in the vector
 */
template <class T>
inline T VarVec<T>::sum() const {
  T s = T(0);
  for (int i=0;i<len;i++) s += x[i];
  return s;
}

/** Calculates the cumulative sum of the elements in the vector
 * @returns the cumulative of the elements in the vector
 */
template <class T>
inline VarVec<T> VarVec<T>::cumsum() const {
  VarVec<T> v(len);
  T s = T(0);
  for (int i=0;i<len;i++) {
    s += x[i];
    v.x[i]=s;
  }
  return v;
}

/** Find the product of the elements in the vector
 * @returns the product of the elements in the vector
 */
template <class T>
inline T VarVec<T>::prod() const {
  T p = T(1);
  for (int i=0;i<len;i++) p *= x[i];
  return p;
}

// General elementwise operations

/** Pass each element through the given single-arg math function,
    replacing the current vector with the result.
 */
template <class T>
inline void VarVec<T>::perform(T (*mathFun)(T)) {
  for (int i=0;i<len;i++) {
    x[i] = (*mathFun)(x[i]);
  }
}

/** Pass each element through the given double-arg math function,
    replacing the current vector with the result.
*/
template <class T>
inline void VarVec<T>::perform(T (*mathFun)(T,T), const T arg2) {
  for (int i=0;i<len;i++) {
    x[i] = (*mathFun)(x[i], arg2);
  }
}

/** Pass each element through the given double-arg math function,
    replacing the current vector with the result.
*/
template <class T>
inline void VarVec<T>::perform(T (*mathFun)(T,T), const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::perform(" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) {
    x[i] = (*mathFun)(x[i], v.x[i]);
  }
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T>
inline VarVec<T> VarVec<T>::performed(T (*mathFun)(T)) {
  VarVec<T> v(len);
  for (int i=0;i<len;i++) {
    v.x[i] = (*mathFun)(x[i]);
  }
  return v;
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T>
inline VarVec<T> VarVec<T>::performed(T (*mathFun)(T,T), const T arg2) {
  VarVec<T> v(len);
  for (int i=0;i<len;i++) {
    v.x[i] = (*mathFun)(x[i], arg2);
  }
  return v;
}

/** Pass each element through the given single-arg math function.
 * @returns a vector of the results
 */
template <class T>
inline VarVec<T> VarVec<T>::performed(T (*mathFun)(T,T), const VarVec<T>& vp) {
#if MATMATH_CHECK_BOUNDS
  if (len!=vp.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::performed(" <<
    ss << "VarVec<" << vp.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> v(len);
  for (int i=0;i<len;i++) {
    v.x[i] = (*mathFun)(x[i], vp.x[i]);
  }
  return v;
}

/** Minimize values: same as *this = min(*this, _rhs), but faster
 * @returns a vector of the results
 */
template <class T>
inline VarVec<T> VarVec<T>::minimize(const VarVec<T>& other) {
#if MATMATH_CHECK_BOUNDS
  if (len!=other.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::minimize(" <<
    ss << "VarVec<" << other.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) {
    if (other.x[i] < x[i]) x[i] = other.x[i];
  }
  return *this;
}

/** Maximize values: same as *this = max(*this, _rhs), but faster
 * @returns a vector of the results
 */
template <class T>
inline VarVec<T> VarVec<T>::maximize(const VarVec<T>& other) {
#if MATMATH_CHECK_BOUNDS
  if (len!=other.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::maximize(" <<
    ss << "VarVec<" << other.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) {
    if (other.x[i] > x[i]) x[i] = other.x[i];
  }
  return *this;
}

// Dot, Cross, and Outer Products

/** Calculate the dot (inner) product with another vector.
 * @param v the other vector
 * @return the inner product
 */
template <class T>
inline T VarVec<T>::dot(const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::dot(" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  T sum = T(0);
  for (int i=0;i<len;i++) sum += x[i]*v.x[i];
  return sum;
}

/** Calculate the outer product with another vector.
 * @param v the other vector
 * @return the outer product matrix
 */
template <class T>
inline VarSMat<T> VarVec<T>::outer(const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::outer(" <<
    ss << "VarVec<" << v.len << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarSMat<T> m(len);
  for (int i=0,k=0;i<len;i++) for (int j=0;j<len;j++,k++) m.x[k] = x[i]*v.x[j];
  return m;
}

/** Calculate the outer product with itself.
 * @return the outer product matrix
 */
template <class T>
inline VarSMat<T> VarVec<T>::outer() const {
  VarSMat<T> m(len);
  for (int i=0,k=0;i<len;i++) for (int j=0;j<len;j++,k++) m.x[k] = x[i]*x[j];
  return m;
}

// Random vectors and sorting

//  /** Creates vector with samples from a uniform distribution on [a,b].
//   * @param a start of range
//   * @param b end of range
//   * @return vector of uniform samples
//   */
//  template <class T>
//  inline VarVec<T> VarVec<T>::uniformRand(const T a, const T b) {
//    VarVec<T> v;
//    for (int i=0;i<len;i++) v.x[i] = puma::uniformRand<T>(a,b);
//    return v;
//  }
//
//  /** Create vector with samples from a normal distribution.
//   * @param mean mean of normal distribution
//   * @param stdev standard deviation of normal distribution
//   * @return vector of normal samples
//   */
//  template <class T>
//  inline VarVec<T> VarVec<T>::normalRand(const T mean, const T stdev) {
//    VarVec<T> v;
//    for (int i=0;i<len;i++) v.x[i] = puma::normalRand<T>(mean, stdev);
//    return v;
//  }
//
//  /** Create vector from a multivariate gaussian distribution.
//   * @param mean mean of normal distribution
//   * @param stdev standard deviation of normal distribution
//   * @return vector of a multivariate gaussian distribution
//   */
//  template <class T>
//  inline VarVec<T> VarVec<T>::multivariateGauss(const VarVec<T>& mean, const SMat<T,len>& cov) {
//    VarVec<T> v;
//    SMat<T,len> S(cov);
//    int n=S.choleskyDecomp();
//    assert(n==0);
//    S.transpose();
//    VarVec<T> X = normalRand();
//    v = mean + S*X;
//    return v;
//  }


/** Sort elements.
 * @param ascending sort in increasing order
 */
template <class T>
inline void VarVec<T>::sort(bool ascending) {
  int count;
  do {
    count = 0;
    for (int i=0;i<(len-1);i++) {
      if (ascending) {
        if (x[i]> x[i+1]) {
          rtc_swap(x[i],x[i+1]); count++;
        }
      } else {
        if (x[i] < x[i+1]) {
          rtc_swap(x[i+1],x[i]); count++;
        }
      }
    }
  } while (count> 0);
}

/** Create a copy of the vector with elements sorted.
 * @param ascending sort in increasing order
 * @return sorted vector
 */
template <class T>
inline VarVec<T> VarVec<T>::sorted(bool ascending) {
  VarVec<T> v(len,x);
  v.sort(ascending);
  return v;
}

// Addition and subtraction

/** Vector-Vector addition.
 */
template <class T>
inline VarVec<T>& VarVec<T>::add(const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::add(" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] = x[i] + v.x[i];
  return *this;
}

/** Add a subvector by adding elements from i onward with the
 * elements from v.
 * @param v the vector from which to add values
 * @param i the (zero based) index of the start of the subvector
 */
template <class T>
inline void VarVec<T>::addSubVec(const int i, const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+v.len > len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::addSubVec(" << i << ", ";
    ss << "VarVec<" << v.len << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int j=i,k=0;j<len && k<v.len;j++,k++) x[j] += v.x[k];
}

/** Vector-Vector subtraction.
 */
template <class T>
inline VarVec<T>& VarVec<T>::subtract(const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::subtract(" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] = x[i] - v.x[i];
  return *this;
}

/** Subtract a subvector by subtracting elements from i onward with the
 * elements from v.
 * @param v the vector from which to subtract values
 * @param i the (zero based) index of the start of the subvector
 */
template <class T>
inline void VarVec<T>::subtractSubVec(const int i, const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+v.len > len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::addSubVec(" << i << ", ";
    ss << "VarVec<" << v.len << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int j=i,k=0;j<len && k<v.len;j++,k++) x[j] -= v.x[k];
}

// Addition and subtraction operator

/** Vector-Vector addition operator.
 */
template <class T>
inline VarVec<T> VarVec<T>::operator + (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator + (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i] + v.x[i];
  return vp;
}

/** Vector-Vector increment operator.
 */
template <class T>
inline void VarVec<T>::operator += (const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator += (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] += v.x[i];
}

/** Vector-Vector subtraction operator.
 */
template <class T>
inline VarVec<T> VarVec<T>::operator - (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator - (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i] - v.x[i];
  return vp;
}

/** Vector-Vector decrement operator.
 */
template <class T>
inline void VarVec<T>::operator -= (const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator -= (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] -= v.x[i];
}

/** Vector negation operator.
 */
template <class T>
inline VarVec<T> VarVec<T>::operator - () const {
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = -x[i];
  return vp;
}

/** Vector-Scalar addition.
 */
template <class T>
inline VarVec<T> VarVec<T>::add(const T a) {
  for (int i=0;i<len;i++) x[i] = x[i] - a;
  return *this;
}

/** Vector-Scalar addition operator.
 * @param a the scalar to add to each element of vector
 */
template <class T>
inline VarVec<T> VarVec<T>::operator + (const T a) const {
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i] + a;
  return vp;
}

/** Vector-Scalar increment operator.
 * @param a the scalar by which to increment each element of vector
 */
template <class T>
inline void VarVec<T>::operator += (const T a) {
  for (int i=0;i<len;i++) x[i] += a;
}

/** Vector-Scalar subtraction.
 */
template <class T>
inline VarVec<T> VarVec<T>::subtract(const T a) {
  for (int i=0;i<len;i++) x[i] = x[i] - a;
  return *this;
}

/** Vector-Scalar subtraction operator.
 * @param a the scalar to subtract from each element of vector
 */
template <class T>
inline VarVec<T> VarVec<T>::operator - (const T a) const {
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i] - a;
  return vp;
}

/** Vector-Scalar decrement operator.
 * @param a the scalar by which to decrement each element of vector
 */
template <class T>
inline void VarVec<T>::operator -= (const T a) {
  for (int i=0;i<len;i++) x[i] -= a;
}

/** Scalar-Vector addition operator.
 * @param a the scalar to add to each element of vector
 * @param v the vector
 */
template <class T>
inline VarVec<T> operator + (const T a, const VarVec<T>& v) {
  VarVec<T> vp(v.len);
  for (int i=0;i<v.len;i++) vp.x[i] = a + v.x[i];
  return vp;
}

/** Scalar-Vector subtraction operator.
 * @param a the scalar to subtract from each element of vector
 * @param v the vector
 */
template <class T>
inline VarVec<T> operator - (const T a, const VarVec<T>& v) {
  VarVec<T> vp(v.len);
  for (int i=0;i<v.len;i++) vp.x[i] = a - v.x[i];
  return vp;
}

// Multiplication and division

/** Vector-Scalar multiplication operator.
 * @param a the scalar with which to multiply each element of vector
 */
template <class T>
inline VarVec<T> VarVec<T>::operator * (const T a) const {
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i]*a;
  return vp;
}

/** Vector-Scalar division operator.
 * @param a the scalar by which to divide each element of vector
 */
template <class T>
inline VarVec<T> VarVec<T>::operator / (const T a) const {
  VarVec<T> vp(len);
  for (int i=0;i<len;i++) vp.x[i] = x[i]/a;
  return vp;
}

/** Vector-Scalar multiplication assignment operator.
 * @param a the scalar with which to multiply each element of vector
 */
template <class T>
inline void VarVec<T>::operator *= (const T a) {
  for (int i=0;i<len;i++) x[i] *= a;
}

/** Vector-Scalar division assignment operator.
 * @param a the scalar by which to divide each element of vector
 */
template <class T>
inline void VarVec<T>::operator /= (const T a) {
  for (int i=0;i<len;i++) x[i] /= a;
}

/** Scalar-Vector multiplication operator.
 * @param a the scalar with which to multiply each element of vector
 * @param v the vector
 */
template <class T>
inline VarVec<T> operator * (const T a, const VarVec<T>& v) {
  VarVec<T> vp(v.len);
  for (int i=0;i<v.len;i++) vp.x[i] = a*v.x[i];
  return vp;
}

// Equality and inequality tests.

/** Element-wise test for equality.
 * @return vector of boolean results
 */
template <class T>
inline bool VarVec<T>::operator == (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator == (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++)
    if (x[i] != v.x[i])
      return(false);
  return true;
}

/** Element-wise test for inequality.
 * @return vector of boolean results
 */
template <class T>
inline bool VarVec<T>::operator != (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator != (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++)
    if (x[i] != v.x[i])
      return(true);
  return false;
}

//  /** Element-wise test for equality.
//   * @return vector of boolean results
//   */
//  template <class T>
//  inline VarVec<bool> VarVec<T>::operator == (const VarVec<T>& v) const {
//    VarVec<bool> b(len,false);
//    for (int i=0;i<len;i++) if (x[i] == v.x[i]) b.x[i] = true;
//    return b;
//  }

//  /** Element-wise test for inequality.
//   * @return vector of boolean results
//   */
//  template <class T>
//  inline VarVec<bool> VarVec<T>::operator != (const VarVec<T>& v) const {
//    VarVec<bool> b(len,false);
//    for (int i=0;i<len;i++) if (x[i] != v.x[i]) b.x[i] = true;
//    return b;
//  }

/** Element-wise test for greater or equal.
 * @return vector of boolean results
 */
template <class T>
inline VarVec<bool> VarVec<T>::operator >= (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator >= (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  VarVec<bool> b(len,false);
  for (int i=0;i<len;i++) if (x[i] >= v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less or equal.
 * @return vector of boolean results
 */
template <class T>
inline VarVec<bool> VarVec<T>::operator <= (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator <= (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  VarVec<bool> b(len,false);
  for (int i=0;i<len;i++) if (x[i] <= v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater.
 * @return vector of boolean results
 */
template <class T>
inline VarVec<bool> VarVec<T>::operator > (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator > (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  VarVec<bool> b(len,false);
  for (int i=0;i<len;i++) if (x[i] > v.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less.
 * @return vector of boolean results
 */
template <class T>
inline VarVec<bool> VarVec<T>::operator < (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator < (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  VarVec<bool> b(len,false);
  for (int i=0;i<len;i++) if (x[i] < v.x[i]) b.x[i] = true;
  return b;
}

/** Compare the entire vector to the passed vector.
 * @returns 1 if all element in this vector are greater than corresponding
 * elements in the passed vector, -1 if they are all less, and 0 otherwise
 */
template <class T>
inline int VarVec<T>::compareTo(const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::compareTo (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  int g=0, l=0;
  for (int i=0;i<len;i++) {
    if (x[i] < v.x[i]) l++;
    if (x[i] > v.x[i]) g++;
  }
  if (l==len) return -1;
  else if (g==len) return 1;
  else return 0;
}

/** Compare the entire vector to the passed vector.
 * @returns true of all corresponding elements are within the given tolerance
 */
template <class T>
inline bool VarVec<T>::equalTo(const VarVec<T>& v, const T tol) const {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::equalTo (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  bool t = true;
  for (int i=0;i<len;i++) if (rtc_abs(x[i] - v.x[i]) > tol) t = false;
  return t;
}

// Element-wise multiplication and division

/** Element-wise multiplication.
 */
template <class T>
inline VarVec<T> VarVec<T>::operator * (const VarVec<T>& v) const {
  VarVec<T> vp(v.len);
  for (int i=0;i<len;i++) vp.x[i] = x[i]*v.x[i];
  return vp;
}

/** Element-wise division.
 */
template <class T>
inline VarVec<T> VarVec<T>::operator / (const VarVec<T>& v) const {
  VarVec<T> vp(v.len);
  for (int i=0;i<v.len;i++) vp.x[i] = x[i]/v.x[i];
  return vp;
}

/** Element-wise multiplication assigment.
 */
template <class T>
inline void VarVec<T>::operator *= (const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator *= (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] *= v.x[i];
}

/** Element-wise division assignment.
 */
template <class T>
inline void VarVec<T>::operator /= (const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (len!=v.len) {
    std::stringstream ss;
    ss << "VarVec<" << len << ">::operator /= (" <<
    ss << "VarVec<" << v.len << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0;i<len;i++) x[i] /= v.x[i];
}

// Serialization routines

/** Writes state to a binary stream.
 */
template <class T>
inline bool VarVec<T>::write(std::ostream& os) const {
  os.write((char *)(len),sizeof(int));
  os.write((char *)(x),len*sizeof(T));
  return os.good();
}

/** Restores state from a binary stream.
 */
template <class T>
inline bool VarVec<T>::read(std::istream& is) {
  int new_len;
  is.read((char *)(new_len),sizeof(int));
  if(new_len!=len) setSize(len);
  is.read((char *)(x),len*sizeof(T));
  return is.good();
}

/** Write state to stream as formated ASCII
 */
template <class T>
std::ostream& operator<<(std::ostream& os, const VarVec<T>& vec) {
int minFieldWidth = os.precision()+2;

os << "[";
for (int i=0; i<vec.length(); ++i)
  os << std::setw(minFieldWidth) << vec.x[i] << " ";
os << "]";

return os;
}

/** Restores state from formated ASCII stream
 */
template <class T>
std::istream& operator>>(std::istream& is, VarVec<T>& vec) {
  using namespace std;
  vector<T> data;
  string vecString;
  stringstream vecStringStream;

  getline(is, vecString, ']');
  int sPos = (int)vecString.find('[');
  if (sPos == (int)string::npos)
    throw Exception("format error: expecting formated matrix to start with '['");

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
  if (colCount != vec.len)
    vec.setSize(colCount);

  //copy extracted data
  for (int i=0;i<vec.len;i++){
    vec.x[i] = data[i];
  }

  return is;
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VARVEC_H defined
//==============================================================================
