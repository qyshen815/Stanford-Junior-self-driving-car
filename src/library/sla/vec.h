//-*-c++-*-
#ifndef VEC_H
#define VEC_H

/** 
 * @file vec.h
 * @brief Basic static vector type, templated in type and size
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 - forked off from matmath.h
 */

#include <mathutil.h>
#include <mat.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

#undef min
#undef max

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int M> class Vec; // M-d vector
  template <class T, int M, int N> class Mat; // MxN Matrix
  template <class T, int M> class SMat; // MxM Square Matrix  
  
  //////////////////// Vec ////////////////////
  /** 
   * An M-Dimensional statically allocated vector.
   * Represents an M dimensional vector of type T
   */ 
  template <class T, int M> 
  class Vec {
  public:
    
    // Data
    T x[M]; ///< storage array

    // Constructors
    Vec();
    Vec(const T* d);
    Vec(const T a);
    Vec(const Vec<T,M>& v);

    // Cast Operation
    template <class U> Vec(const Vec<U,M>& v);    
    
    // Mutators
    void set(const T* d, const int len = M);
    void set(const T a, const int len = M);
    void set(const Vec<T,M>& v, const int len = M);
    Vec<T,M>& operator = (const T a);
    Vec<T,M>& operator = (const T* d);
    Vec<T,M>& operator = (const Vec<T,M>& v);
    template <int N> void setSubVec(const int i, const Vec<T,N>& v);
    template <int N> void addToSubVec(const int i, const Vec<T,N>& v);
    
    // Casting Mutators
    template <class U> void set(const Vec<U,M>& v);
    template <class U> Vec<T,M>& operator = (const Vec<U,M>& v);
    
    // Accessors
    const T& operator () (const int i) const;
    T& operator () (const int i);
    const T& end(const int i = 0) const;
    T& end(const int i = 0);
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
    int argmax(int active_m = M) const;

    // Elementwise operations
    void perform(T (*mathFun)(T));
    void perform(T (*mathFun)(T,T), const T arg2);
    void perform(T (*mathFun)(T,T), const Vec<T,M>& v);
    Vec<T,M> performed(T (*mathFun)(T));
    Vec<T,M> performed(T (*mathFun)(T,T), const T arg2);
    Vec<T,M> performed(T (*mathFun)(T,T), const Vec<T,M>& v); 
   
    // Dot and Outer Products
    T dot(const Vec<T,M>& v) const;
    SMat<T,M> outer(const Vec<T,M>& v) const;
    SMat<T,M> outer() const;
    
    // Random vectors and sorting etc
    static Vec<T,M> uniformRand(const T a = T(0), const T b = T(1));
    static Vec<T,M> normalRand(const T mean = T(0), const T stdev = T(1));
    void sort(bool ascending = true);
    Vec<T,M> sorted(bool ascending = true);
    int find(const T element, const int active_m = M) const; 
    bool contains(const T element, const int active_m = M) const; 
    bool containsAll(const Vec<T,M>& v, const int active_m = M) const;
    void shiftBack(const T element);

    
    // Addition and subtraction
    Vec<T,M> operator + (const Vec<T,M>& v) const;
    void operator += (const Vec<T,M>& v);
    Vec<T,M> operator - (const Vec<T,M>& v) const;
    void operator -= (const Vec<T,M>& v);
    Vec<T,M> operator - () const;
    Vec<T,M> operator + (const T a) const;
    void operator += (const T a);
    Vec<T,M> operator - (const T a) const;
    void operator -= (const T a);
    
    // Multiplication and division
    Vec<T,M> operator * (const T a) const;
    Vec<T,M> operator / (const T a) const;
    void operator *= (const T a);
    void operator /= (const T a);

    // Equality and inequality tests
    Vec<bool,M> operator == (const Vec<T,M>& v) const;
    Vec<bool,M> operator != (const Vec<T,M>& v) const;
    Vec<bool,M> operator >= (const Vec<T,M>& v) const;
    Vec<bool,M> operator <= (const Vec<T,M>& v) const;
    Vec<bool,M> operator > (const Vec<T,M>& v) const;
    Vec<bool,M> operator < (const Vec<T,M>& v) const;
    int compareTo(const Vec<T,M>& v) const;
    bool equalTo(const Vec<T,M>& v, const int active_m = M) const;
    bool nearTo(const Vec<T,M>& v, const T tol = T(0), const int active_m = M) const;

    // Element-wise operations
    Vec<T,M> operator * (const Vec<T,M>& v) const;
    Vec<T,M> operator / (const Vec<T,M>& v) const;
    void operator *= (const Vec<T,M>& v);
    void operator /= (const Vec<T,M>& v);

    // Serialization
    void writeTo(std::ostream& os) const;
    void readFrom(std::istream& is);
    void print(std::ostream& os = std::cout, const int active_m = M) const;
    void matlab_print(const std::string name, std::ostream& os = std::cout, const int active_m = M) const;
  };

  // Global operators for cases where Vec<T,M> 
  // is the second argument in a binary operator
  template <class T, int M> 
  Vec<T,M> operator + (const T a, const Vec<T,M>& v);
  template <class T, int M> 
  Vec<T,M> operator - (const T a, const Vec<T,M>& v);
  template <class T, int M> 
  Vec<T,M> operator * (const T a, const Vec<T,M>& v);
  
  // Qsort-style comparison function
  template <class T, int M>
  int compareVecAscending(const void* a, const void* b);
  template <class T, int M>
  int compareVecDescending(const void* a, const void* b);
  template <class T, int M, int K>
  int compareVecElementAscending(const void* a, const void* b);
  template <class T, int M, int K>
  int compareVecElementDescending(const void* a, const void* b);

  // ASCII stream IO
  template <class T, int M> 
  std::ostream& operator<<(std::ostream& os, const Vec<T,M>& vec);
  
  // Declare a few common typdefs
  typedef Vec<bool,5> Vec5b;
  typedef Vec<char,5> Vec5c;
  typedef Vec<unsigned char,5> Vec5uc;
  typedef Vec<int,5> Vec5i;
  typedef Vec<float,5> Vec5f;
  typedef Vec<double,5> Vec5d;

  typedef Vec<bool,6> Vec6b;
  typedef Vec<char,6> Vec6c;
  typedef Vec<unsigned char,6> Vec6uc;
  typedef Vec<int,6> Vec6i;
  typedef Vec<float,6> Vec6f;
  typedef Vec<double,6> Vec6d;

  typedef Vec<bool,8> Vec8b;
  typedef Vec<char,8> Vec8c;
  typedef Vec<unsigned char,8> Vec8uc;
  typedef Vec<int,8> Vec8i;
  typedef Vec<float,8> Vec8f;
  typedef Vec<double,8> Vec8d;

  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Vec ////////////////////
  
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
   * @param a the value to assign to all elements
   */
  template <class T, int M>
  inline Vec<T,M>::Vec(const Vec<T,M>& v) {
    set(v);
  }
  
  // Casting Operation

  /** Casting Ctor that initializes from passed vector.
   * @param a the value to assign to all elements
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
  inline void Vec<T,M>::set(const T a, const int len) {
    for (int i=0;i<len;++i) x[i] = a;
  }
    
  /** Set all elements from an array.
   * @param d pointer to the initalization array
   */
  template <class T, int M> 
  inline void Vec<T,M>::set(const T* d, const int len) {
    for (int i=0;i<len;i++) x[i] = d[i];
  }
    
  /** Set this vector equal to passed vector
   * @param v the vector to replicate
   */
  template <class T, int M> 
  inline void Vec<T,M>::set(const Vec<T,M>& v, const int len) {
    for (int i=0;i<len;i++) x[i] = v.x[i];
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

  /** Add to a subvector by replacing elements from i onward with the
   * elements from v.
   * Replaces elements till the end of either vector is reached.
   * @param v the vector from which to copy values
   * @param i the (zero based) index of the start of the subvector
   */
  template <class T, int M> template <int N> 
  inline void Vec<T,M>::addToSubVec(const int i, const Vec<T,N>& v) {
    for (int j=i,k=0;j<M && k<N;j++,k++) x[j] += v.x[k]; 
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
  inline const T& Vec<T,M>::operator () (const int i) const {
#if MATMATH_CHECK_BOUNDS
    if (i < 0 || i >= M) { 
      std::cerr << "Vec<" << M << ">::(" << i << "): index out of range\n";
      std::cerr << std::flush;
      assert(0); 
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
    if (i < 0 || i >= M) { 
      std::cerr << "&Vec<" << M << ">::(" << i << "): index out of range\n"; 
      std::cerr << std::flush;
      assert(0); 
      exit(1); 
    }
#endif
    return x[i];
  }
  
  /** Get an element of the vector (by reference), starting from the end of the vector.
   * Indices need to be negative, positive indices lead to out-of-bounds accessing. 
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i the (zero based) index into the vector
   */
  template <class T, int M> 
  inline const T& Vec<T,M>::end(const int i) const {
    return operator()(M-1+i); 
  }

  /** Get an element of the vector (by reference), starting from the end of the vector.
   * Indices need to be negative, positive indices lead to out-of-bounds accessing. 
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i the (zero based) index into the vector
   */
  template <class T, int M> 
  inline T& Vec<T,M>::end(const int i) {
    return operator()(M-1+i); 
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
  
  /** Find the product of the elements in the vector
   * @returns the product of the elements in the vector
   */
  template <class T, int M> 
  inline T Vec<T,M>::prod() const {
    T p = T(1);
    for (int i=0;i<M;i++) p *= x[i];
    return p;
  }
  
  /** Find the index of the maximum value of the vector
   * @returns the index of the maximum value
   */
  template <class T, int M> 
  inline int Vec<T,M>::argmax(int active_m) const {
    int im = 0;
    for (int i=1;i<active_m;i++) if (x[i]>x[im]) im = i;
    return im;
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
    for (int i=0;i<M;i++) v.x[i] = sla::uniformRand<T>(a,b);
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
    for (int i=0;i<M;i++) v.x[i] = sla::normalRand<T>(mean, stdev);
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
	    swap(x[i],x[i+1]); count++;
	  }
	} else {
	  if (x[i] < x[i+1]) {
	    swap(x[i+1],x[i]); count++;
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

  /** Finds the index of an element in the vector
  * @param element is the element to find
  * @return an int: the index of the element. -1 if it could not be found. 
  */
  template <class T, int M> 
  inline int Vec<T,M>::find(const T element, const int active_m) const {
    for (int i=0;i<active_m;i++) if (x[i] == element) return i;
    return -1; 
  }

  /** Checks if the given element is in this vector
  * @param element is the element to check
  * @return a bool: true if the element is in the vector, false if not
  */
  template <class T, int M> 
  inline bool Vec<T,M>::contains(const T element, const int active_m) const {
    return find(element, active_m) >= 0;
  }

  /** Checks if this vector contains all the elements of the given vector, up to element active_m
  * @param v is the other vector to which we are comparing
  * @return a bool: true if contains all of other elements
  */
  template <class T, int M> 
  inline bool Vec<T,M>::containsAll(const Vec<T,M>& v, const int active_m) const {
    for (int i=0;i<active_m;i++) if (!contains(v.x[i],active_m)) return false;
    return true;
  }

  /** Shifts the contents of this vector so that all elements end up on the index before them, dropping
  * the value in x[0] and inserting the supplied element at x[M-1]. 
  * @param element to put in the last slot
  * @return nothing
  */
  template <class T, int M> 
  inline void Vec<T,M>::shiftBack(const T element) {
    for (int i=1;i<M;i++) x[i-1] = x[i]; 
    x[M-1] = element; 
  }



  // Addition and subtraction
  
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
   */
  template <class T, int M> 
  inline Vec<T,M> operator + (const T a, const Vec<T,M>& v) {
    Vec<T,M> vp;
    for (int i=0;i<M;i++) vp.x[i] = a + v.x[i];
    return vp;  
  }
  
  /** Scalar-Vector subtraction operator.
   * @param a the scalar to subtract from each element of vector
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
  inline Vec<bool,M> Vec<T,M>::operator == (const Vec<T,M>& v) const {
    Vec<bool,M> b(false);
    for (int i=0;i<M;i++) if (x[i] == v.x[i]) b.x[i] = true;
    return b;
  }
  
  /** Element-wise test for inequality.
   * @return vector of boolean results
   */
  template <class T, int M> 
  inline Vec<bool,M> Vec<T,M>::operator != (const Vec<T,M>& v) const {
    Vec<bool,M> b(false);
    for (int i=0;i<M;i++) if (x[i] != v.x[i]) b.x[i] = true;
    return b;
  }
  
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
   * @returns true if all corresponding elements are equal
   */
  template <class T, int M> 
  inline bool Vec<T,M>::equalTo(const Vec<T,M>& v, const int active_m) const {
    for (int i=0;i<active_m;i++) if (x[i] != v.x[i]) return false;
    return true;
  }

  /** Compare the entire vector to the passed vector.
   * @returns true if all corresponding elements are within the specified tolerance
   */
  template <class T, int M> 
  inline bool Vec<T,M>::nearTo(const Vec<T,M>& v, const T tol, const int active_m) const {
    for (int i=0;i<active_m;i++) if (fabs(double(x[i] - v.x[i])) > tol) return false;
    return true;
  }

  /** General compare functions for use with QSort, etc.
   */
  template <class T, int M>
  inline int compareVecAscending(const void* a, const void* b) {
    T an = (*(Vec<T,M>*)(a)).norm();
    T bn = (*(Vec<T,M>*)(b)).norm();
    if (an == bn) return 0;
    else return 2*int(an > bn)-1;
  } 
  template <class T, int M>
  inline int compareVecDescending(const void* a, const void* b) {
    T an = (*(Vec<T,M>*)(a)).norm();
    T bn = (*(Vec<T,M>*)(b)).norm();
    if (an == bn) return 0;
    else return 2*int(an < bn)-1;
  }
  template <class T, int M, int K>
  inline int compareVecElementAscending(const void* a, const void* b) {
    T an = (*(Vec<T,M>*)(a))(K);
    T bn = (*(Vec<T,M>*)(b))(K);
    if (an == bn) return 0;
    else return 2*int(an > bn)-1;
  } 
  template <class T, int M, int K>
  inline int compareVecElementDescending(const void* a, const void* b) {
    T an = (*(Vec<T,M>*)(a))(K);
    T bn = (*(Vec<T,M>*)(b))(K);
    if (an == bn) return 0;
    else return 2*int(an < bn)-1;
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
  inline void Vec<T,M>::writeTo(std::ostream& os) const {
    os.write((char *)(x),M*sizeof(T));
  }

  /** Restores state from a binary stream.
   */
  template <class T, int M> 
  inline void Vec<T,M>::readFrom(std::istream& is) {
    is.read((char *)(x),M*sizeof(T));
  }
  
  /** Write state to stream as formated ASCII 
   */
  template <class T, int M> 
    void Vec<T,M>::print(std::ostream& os, const int active_m) const 
  {
    if (active_m<8) {
      Mat<T,1,M> tmpMat;
      tmpMat.setRow(0,*this);
      tmpMat.print(os,1,active_m);
    } else {
      Mat<T,M,1> tmpMat;
      tmpMat.setCol(0,*this);
      tmpMat.print(os,active_m,1);
    }
  }

  /** Pretty-print in format easily imported into Matlab 
   */
  template <class T, int M> 
    void Vec<T,M>::matlab_print(const std::string name, std::ostream& os, const int active_m) const 
  {
    os << std::endl << name << " = ... " << std::endl;
    print(os, active_m); 
    if (active_m<8) os << "\'";
    os << ";" << std::endl;     
  }

  /** Write state to stream as formated ASCII 
  */
  template <class T, int M> 
    std::ostream& operator<<(std::ostream& os, const Vec<T,M>& vec) 
  {
    vec.print(os);
    return os;
  }
} // end namespace sla

#endif
