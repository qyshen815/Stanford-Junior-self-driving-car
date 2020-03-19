//-*-c++-*-
#ifndef MAT_H
#define MAT_H

/** 
 * @file mat.h
 * @brief Basic static matric type, templated in type and size
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 - forked off from matmath.h
 */

#include <string.h>
#include <mathutil.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <stdexcept>

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

  //////////////////// Mat ////////////////////
  /**
   * An MxN matrix.
   */
  template <class T, int M, int N> 
  class Mat {
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
    template <int P, int Q> void setSubMat(const int i, const int j, 
					   const Mat<T,P,Q>& m);  
    template <int P, int Q> void addToSubMat(const int i, const int j, 
					     const Mat<T,P,Q>& m);  

    // Casting Mutators
    template <class U> void set(const Mat<U,M,N>& m);
    template <class U> Mat<T,M,N>& operator = (const Mat<U,M,N>& m);        
    
    // Accessors
    T operator () (const int i, const int j) const;
    T& operator () (const int i, const int j);  
    Vec<T,N> getRow(int i) const;
    Vec<T,M> getCol(int j) const;
    template <int P, int Q> Mat<T,P,Q> getSubMat(const int i, const int j);
    
    // Convenience
    bool contains(const T element, const int active_m = M, const int active_n = N) const; 

    // Addition and subtraction: <matrix> +/- <matrix>
    Mat<T,M,N> operator + (const Mat<T,M,N>& m) const;
    void operator += (const Mat<T,M,N>& m);
    Mat<T,M,N> operator - (const Mat<T,M,N>& m) const;
    void operator -= (const Mat<T,M,N>& m);
    Mat<T,M,N> operator - () const;

    // Multiplication and division: <matrix> *// <scalar>
    Mat<T,M,N> operator * (const T a) const;
    void operator *= (const T a);
    Mat<T,M,N> operator / (const T a) const;
    void operator /= (const T a);

    // Multiplication: <matrix> * <vector>
    Vec<T,M> operator * (const Vec<T,N>& v) const;
    template <int P> Mat<T,M,P> operator * (const Mat<T,N,P>& m) const;
    void operator *= (const Mat<T,N,N>& m);
    Mat<T,N,N> inner() const;
    template <int P> Mat<T,N,P> inner(const Mat<T,M,P>& m) const;
    
    // Equality and inequality tests
    int compareTo(const Mat<T,M,N>& m) const;
    bool equalTo(const Mat<T,M,N>& m) const;
    bool nearTo(const Mat<T,M,N>& m, const T tol = T(0)) const;
    
    // Other matrix operations
    Mat<T,N,M> transposed() const;
    
    // Random matrix
    static Mat<T,M,N> uniformRand(const T a = T(0), const T b = T(1));
    static Mat<T,M,N> normalRand(const T mean = T(0), const T stdev = T(1));
    
    // Serialization
    void writeTo(std::ostream& os) const;
    void readFrom(std::istream& is);
    void print(std::ostream& os = std::cout, const int active_m = M, const int active_n = N) const;
    void matlab_print(const std::string name, std::ostream& os = std::cout, const int active_m = M, const int active_n = N) const;
  };
  
  // Global operators to for cases where Mat<T,M,N> 
  // is the second argument in a binary operator  
  template <class T, int M, int N> 
  Mat<T,M,N> operator * (const T a, const Mat<T,M,N> &m);
  template <class T, int M, int N> 
  Vec<T,N> operator * (const Vec<T,M>& v, const Mat<T,M,N> &m);
  
  // ASCII stream IO
  template <class T, int M, int N>  
  std::ostream& operator<<(std::ostream& os, const Mat<T,M,N>& mat);  

  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////

  //////////////////// Mat ////////////////////
  
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
   * @param a the value to assign to all elements
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
    for (int i=0;i<M*N;++i) x[i] = a;
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
    if (i < 0 || i >= M) { 
      std::cerr << "Mat<" << M << "," << N << ">::setRow(" << i 
		<< ", v): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    for (int j=0,k=i;j<N;j++,k+=M) x[k] = v.x[j];
  }
  
  /** Set a column.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of col to set
   * @param v source vector
   */
  template <class T, int M, int N> 
  inline void Mat<T,M,N>::setCol(const int j, const Vec<T,M>& v) {
#if MATMATH_CHECK_BOUNDS
    if (j < 0 || j >= N) { 
      std::cerr << "Mat<" << M << "," << N << ">::setCol(" << j 
		<< ", v) out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    for (int i=0,k=j*M;i<M;i++,k++) x[k] = v.x[i];
  }
  
  /** Set a sub-matrix.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of top row of sub-matrix
   * @param j zero based index of left col of sub-matrix
   * @param m matrix to be placed with UL corner at (i,j)
   */
  template <class T, int M, int N> template <int P, int Q> 
  inline void Mat<T,M,N>::setSubMat(const int i, const int j, 
				    const Mat<T,P,Q>& m) 
  {
#if MATMATH_CHECK_BOUNDS
    if (i < 0 || i+P > M || j < 0 || j+Q > N) { 
      std::cerr << "Mat<" << M << "," << N << ">::setSubMat(" << i 
        << ", " << j << ", m): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    for (int id=i,is=0;id<M && is<P;id++,is++) 
      for (int jd=j,js=0;jd<N && js<Q;jd++,js++) 
        x[id+jd*M] = m.x[is+js*P];
  }
    
  /** Adds to a sub-matrix.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of top row of sub-matrix
   * @param j zero based index of left col of sub-matrix
   * @param m matrix to be placed with UL corner at (i,j)
   */
  template <class T, int M, int N> template <int P, int Q> 
  inline void Mat<T,M,N>::addToSubMat(const int i, const int j, 
				      const Mat<T,P,Q>& m) 
  {
#if MATMATH_CHECK_BOUNDS
    if (i < 0 || i+P > M || j < 0 || j+Q > N) { 
      std::cerr << "Mat<" << M << "," << N << ">::setSubMat(" << i 
        << ", " << j << ", m): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    for (int id=i,is=0;id<M && is<P;id++,is++) 
      for (int jd=j,js=0;jd<N && js<Q;jd++,js++) 
        x[id+jd*M] += m.x[is+js*P];
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
    if (i<0 || i>=M || j<0 || j>=N) { 
      std::cerr << "Mat<" << M << "," << N << ">::operator (" << i 
		<< ", " << j << "): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    return x[i+j*M];
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
    if (i<0 || i>=M || j<0 || j>=N) { 
      std::cerr << "&Mat<" << M << "," << N << ">::operator (" << i 
		<< ", " << j << "): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    return x[i+j*M];
  }
  
  /** Get a row.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of row to get
   */
  template <class T, int M, int N> 
  inline Vec<T,N> Mat<T,M,N>::getRow(int i) const {
#if MATMATH_CHECK_BOUNDS
    if (i < 0 || i >= M) { 
      std::cerr << "Mat<" << M << "," << N << ">::getRow(" << i 
		<< "): index out of range\n";
      std::cerr << std::flush;
      exit(1);      
    }
#endif
    Vec<T,N> v;
    for (int j=0,k=i;j<N;j++,k+=M) v.x[j] = x[k];
    return v;
  }
  
  /** Get a column.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of row to get
   */
  template <class T, int M, int N> 
  inline Vec<T,M> Mat<T,M,N>::getCol(int j) const {
#if MATMATH_CHECK_BOUNDS
    if (j < 0 || j >= N) { 
      std::cerr << "Mat<" << M << "," << N << ">::getCol(" << j
		<< "): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif
    Vec<T,M> v;
    for (int i=0,k=j*M;i<M;i++,k++) v.x[i] = x[k];
    return v;
  }

  /** Get a sub-matrix.
   * Does bounds checking if MATMATH_CHECK_BOUNDS is set
   * @param i zero based index of top row of sub-matrix
   * @param j zero based index of left col of sub-matrix
   */
  template <class T, int M, int N> template <int P, int Q> 
  inline Mat<T,P,Q> Mat<T,M,N>::getSubMat(const int i, const int j) 
  {
#if MATMATH_CHECK_BOUNDS
    if (i < 0 || i+P > M || j < 0 || j+Q > N) { 
      std::cerr << "Mat<" << M << "," << N << ">::getSubMat(" << i 
        << ", " << j << "): index out of range\n";
      std::cerr << std::flush;
      exit(1); 
    }
#endif  
    Mat<T,P,Q> m(T(0));
    for (int id=i,is=0;id<M && is<P;id++,is++) 
      for (int jd=j,js=0;jd<N && js<Q;jd++,js++) 
        m.x[is+js*P] = x[id+jd*M];
    return m;
  }

  
  /** Checks if the given element is in this vector
  * @param element is the element to check
  * @return a bool: true if the element is in the vector, false if not
  */
  template <class T, int M, int N> 
  inline bool Mat<T,M,N>::contains(const T element, const int active_m, const int active_n) const {
    for (int j=0;j<active_m;j++) for (int i=0;i<active_n;i++) if (x[i+j*M] == element) return true;
    return false; 
  }


  // Addition and subtraction: <matrix> +/- <matrix>
  
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
    for (int j=0,k=0;j<N;j++) for (int i=0;i<M;i++,k++) vp.x[i] += x[k]*v.x[j];
    return vp;
  }
  
  /** Vector-Matrix multiplication operator.
   */
  template <class T, int M, int N> 
  Vec<T,N> operator * (const Vec<T,M>& v, const Mat<T,M,N> &m) {
    Vec<T,N> vp(T(0));
    for (int j=0,k=0;j<N;j++) for (int i=0;i<M;i++,k++) vp.x[j]+=m.x[k]*v.x[i];
    return vp;  
  }

  // Multiplication: <matrix> * <matrix>

  /** Matrix-Matrix multiplication operator.
   */  
  template <class T, int M, int N> template <int P>
  inline Mat<T,M,P> Mat<T,M,N>::operator * (const Mat<T,N,P>& m) const {
    Mat<T,M,P> mp(T(0));
    for (int j=0;j<P;j++) 
      for (int i=0;i<M;i++) 
        for (int k=0;k<N;k++) 
          mp.x[i+j*M] += x[i+k*M]*m.x[k+j*N];
    return mp;
  }
  
  
  /** Matrix-Matrix multiplication assignement operator.
   */
  template <class T, int M, int N> 
  inline void Mat<T,M,N>::operator *= (const Mat<T,N,N>& m) {
    (*this) = (*this)*m;
  }

  /** Inner product of this matrix with itself, i.e., A'*A
   */
  template <class T, int M, int N> 
    inline Mat<T,N,N> Mat<T,M,N>::inner() const {
      Mat<T,N,N> mp(T(0));
      for (int i=0;i<N;i++) 
        for (int j=0;j<N;j++)
          for (int k=0;k<M;k++)
            mp.x[i+j*N] += x[k+i*M]*x[k+j*M];
      return mp;
    }
  
  /** Inner product of this matrix with given matrix, i.e., A'*M
   */
  template <class T, int M, int N> template <int P>
    inline Mat<T,N,P> Mat<T,M,N>::inner(const Mat<T,M,P>& m) const {
      Mat<T,N,P> mp(T(0));
      for (int i=0;i<N;i++) 
        for (int j=0;j<P;j++)
          for (int k=0;k<M;k++)
            mp.x[i+j*N] += x[k+i*M]*m.x[k+j*M];
      return mp;
    }  

    // Equality and inequality tests

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
   * @returns true of all corresponding elements are equal
   */
  template <class T, int M, int N> 
  inline bool Mat<T,M,N>::equalTo(const Mat<T,M,N>& m) const {
    for (int i=0;i<M*N;i++) if (x[i] != m.x[i]) return false;
    return true;
  }

  /** Compare the entire matrix to the passed matrix.
   * @returns true of all corresponding elements are within the given tolerance
   */
  template <class T, int M, int N> 
  inline bool Mat<T,M,N>::nearTo(const Mat<T,M,N>& m, const T tol) const {
    for (int i=0;i<M*N;i++) if (fabs(x[i] - m.x[i]) > tol) return false;
    return true;
  }

  // Transpose
  
  /** Return the transpose of the matrix without modifying matrix.
   */
  template <class T, int M, int N> 
  inline Mat<T,N,M> Mat<T,M,N>::transposed() const {
    Mat<T,N,M> mp;
    for (int i=0;i<M;i++) for (int j=0;j<N;j++) mp.x[j+i*N] = x[i+j*M];
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
    for (int i=0;i<M*N;i++) m.x[i] = sla::uniformRand<T>(a,b);
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
    for (int i=0;i<M*N;i++) m.x[i] = sla::normalRand<T>(mean,stdev);
    return m;
  }

  // Serialization routines
  
  /** Write state to a binary stream.
   */
  template <class T, int M, int N> 
  inline void Mat<T,M,N>::writeTo(std::ostream& os) const {
    os.write((char *)(x),M*N*sizeof(T));
  }

  /** Restore state from a binary stream.
   */
  template <class T, int M, int N> 
  inline void Mat<T,M,N>::readFrom(std::istream& is) {
    is.read((char *)(x),M*N*sizeof(T));
  }
    
  /** Write state to stream as formated ASCII 
   */
  template <class T, int M, int N> 
  void Mat<T,M,N>::print(std::ostream& os, const int active_m, const int active_n) const {
    os.precision(8);
    int minFieldWidth = os.precision()+2;  

    os << "[" ;      
    for (int i=0;i<active_m;i++) {
      for (int j=0;j<active_n;j++) 
        os << std::setw(minFieldWidth) << (*this)(i,j) << (j<active_n-1 ? " " : "");
      if (i<active_m-1) os << ";" << std::endl; 
    }
    os << "]";
  }

  /** Pretty-print in format easily imported into Matlab
   */  
  template <class T, int M, int N> 
  void Mat<T,M,N>::matlab_print(const std::string name, std::ostream& os, const int active_m, const int active_n) const 
  {
    os << std::endl << name << " = ... " << std::endl;
    print(os, active_m, active_n); 
    os << ";" << std::endl;     
  }


  /** Write state to stream as formated ASCII 
   */
  template <class T, int M, int N> 
  std::ostream& operator<<(std::ostream& os, const Mat<T,M,N>& mat) {
    mat.print(os);    
    return os;
  }
} // end namespace sla

#endif  
