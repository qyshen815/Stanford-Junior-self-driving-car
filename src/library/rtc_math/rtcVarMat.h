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
 * file .......: rtcVarMat.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VARMAT_H
#define RTC_VARMAT_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"
#include "rtcVarVec.h"
#include "rtcVarSMat.h"
#include "rtcArray2.h"

//== NAMESPACES ================================================================
namespace rtc {

/// Forward declarations
template <class T> class VarVec;  // M-d vector
template <class T> class VarMat;  // MxN Matrix
template <class T> class VarSMat; // MxM Matrix

/**
 * Mathematical matrix container class.
 *
 * The VarMat class allows the representation of \e n x \e m matrices.
 * The rows will be indexed between 0 and n-1, and the columns between 0
 * and m-1.
 *
 * The matrix class is a container class implemented as template.
 *
 * If you need to create a matrix of floats with 20 rows and 15 columns,
 * all elements initialized with an initial value of 4.27 just create it:
 *
 * \code
 * rtc::VarMat<float> myMat(20,15,4.27f) // creates matrix with 300 elements
 *                                       // all initialized with 4.27f
 * \endcode
 *
 * To access the matrix elements use the access operators.  There are many
 * possibilities.  With at(const int row, const int col) is possible to
 * access an element directly.
 *
 */
template <class T>
class VarMat : public Array2<T> {
public:
  // Constructors
  VarMat();
  VarMat(int rows, int cols);
  VarMat(int rows, int cols, const T* d);
  VarMat(int rows, int cols, const T a);
  VarMat(const Array<T,2>& a);

  // Cast Operation
  template <class U> VarMat(const VarMat<U>& m);
  template <class U, int M, int N> VarMat(const Mat<U,M,N>& m);

  // Mutators
  void setRow(int i, const VarVec<T>& v);
  void setCol(int j, const VarVec<T>& v);
  void setSubMat(int r, int c, const VarMat<T>& m);

  // Casting Mutators
  template <class U> void set(const VarMat<U>& m);
  template <class U, int M, int N> void set(const Mat<U,M,N>& m);
  template <class U> VarMat<T>& operator = (const VarMat<U>& m);
  template <class U, int M, int N> VarMat<T>& operator = (const Mat<U,M,N>& m);

  // Accessors
  int length() const;
  VarVec<T> getRow(int r) const;
  VarVec<T> getCol(int c) const;
  VarMat<T> getSubMat(int i, int j, int sub_rows, int sub_cols);

  // Addition and subtraction: <matrix> +/- <matrix>
  VarMat<T>& add(const VarMat<T>& m);
  void addSubMat(int r, int c, const VarMat<T>& m);
  VarMat<T>& subtract(const VarMat<T>& m);
  void subtractSubMat(int r, int c, const VarMat<T>& m);
  VarMat<T> operator + (const VarMat<T>& m) const;
  void operator += (const VarMat<T>& m);
  VarMat<T> operator - (const VarMat<T>& m) const;
  void operator -= (const VarMat<T>& m);
  VarMat<T> operator - () const;

  // Addition and subtraction: <matrix> +/- <scalar>
  VarMat<T>& add(const T a);
  VarMat<T>& subtract(const T a);
  VarMat<T> operator + (const T a) const;
  void operator += (const T a);
  VarMat<T> operator - (const T a) const;
  void operator -= (const T a);

  // Multiplication and division: <matrix> *// <scalar>
  VarMat<T> operator * (const T a) const;
  void operator *= (const T a);
  VarMat<T> operator / (const T a) const;
  void operator /= (const T a);

  // Multiplication: <matrix> * <vector>
  VarVec<T> operator * (const VarVec<T>& v) const;
  VarMat<T> operator * (const VarMat<T>& m) const;
  void operator *= (const VarMat<T>& m);

  // Equality and inequality tests
  VarMat<bool> operator == (const VarMat<T>& m) const;
  VarMat<bool> operator != (const VarMat<T>& m) const;
  VarMat<bool> operator >= (const VarMat<T>& m) const;
  VarMat<bool> operator <= (const VarMat<T>& m) const;
  VarMat<bool> operator > (const VarMat<T>& m) const;
  VarMat<bool> operator < (const VarMat<T>& m) const;
  int compareTo(const VarMat<T>& m) const;
  bool equalTo(const VarMat<T>& m, const T tol = T(0)) const;

  // Other matrix operations
  VarMat<T> transposed() const;

  // Random matrix
  VarMat<T> uniformRand(const T a = T(0), const T b = T(1));
  VarMat<T> normalRand(const T mean = T(0), const T stdev = T(1));
//    static VarMat<T> multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov);

  // General elementwise operations
  void perform(T (*mathFun)(T));

  // Reductions: Max/Min, Sum/Product
  //T max() const;
  //T min() const;
  VarVec<T> sum() const;
  //T prod() const;

  // Bilinear Interpolation
  T interpolate(const float i, const float j) const;

  // statistical operations
  VarVec<T> meanOfRows() const;
  VarSMat<T> covarianceMatrixOfRows() const;

  // inherit member data and functions of parent
  using Array<T,2>::x;
  using Array<T,2>::len;
  using Array<T,2>::set;
  using Array<T,2>::mul;
  using Array2<T>::setSize;
  using Array2<T>::rows;
  using Array2<T>::columns;
};

// Declare a few common typdefs
typedef VarMat<bool> VarMatb;
typedef VarMat<char> VarMatc;
typedef VarMat<unsigned char> VarMatuc;
typedef VarMat<int> VarMati;
typedef VarMat<float> VarMatf;
typedef VarMat<double> VarMatd;

// Global operators to for cases where VarMat<T>
// is the second argument in a binary operator
template <class T> VarMat<T> operator * (const T a, const VarMat<T> &m);
//  template <class T> Vec<T,N> operator * (const Vec<T,M>& v, const VarMat<T> &m);

// ASCII stream IO
template <class T> std::ostream& operator<<(std::ostream& os, const VarMat<T>& m);
template <class T> std::istream& operator>>(std::istream& is, VarMat<T>& m);

//==============================================================================
// VarMat<T>
//==============================================================================

// Constructors

/**
 * default Ctor
 */
template <class T>
inline VarMat<T>::VarMat() : Array2<T>() {}

/** Ctor that initializes an empty matrix.
 * @param rows the number of rows
 * @param cols the number of columns
 */
template <class T>
inline VarMat<T>::VarMat(int rows, int cols) : Array2<T>(rows,cols) {
}

/** Ctor that initializes from an array.
 * @param rows the number of rows
 * @param cols the number of columns
 * @param d the (row major) data array of length rows*cols
 */
template <class T>
inline VarMat<T>::VarMat(int rows, int cols, const T* d) : Array2<T>(rows,cols,d) {
}

/** Ctor that initializes ALL ELEMENTS to a scalar value.
 * @param rows the number of rows
 * @param cols the number of columns
 * @param a the value to which all elements will be set
 */
template <class T>
inline VarMat<T>::VarMat(int rows, int cols, const T a) : Array2<T>(rows,cols,a) {
}

/** Ctor that initializes from passed matrix.
 * @param a the matrix to be copied.
 */
template <class T>
inline VarMat<T>::VarMat(const Array<T,2>& a) : Array2<T>(a) {
}

// Casting Operation

/** Casting Ctor that initializes from passed matrix with type cast.
 * @param m the matrix to be copied.
 */
template <class T> template <class U>
inline VarMat<T>::VarMat(const VarMat<U>& m) : Array2<T>() {
  setSize(m.rows(),m.columns());
  set<U>(m);
}

// Mutators

/** Casting Ctor that initializes from passed matrix with type cast.
 * @param m the matrix to be copied.
 */
template <class T> template <class U, int M, int N>
inline VarMat<T>::VarMat(const Mat<U,M,N>& m) : Array2<T>() {
  setSize(M,N);
  set<U,M,N>(m);
}

// Mutators

/** Set a row.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of row to set
 * @param v source vector
 */
template <class T>
inline void VarMat<T>::setRow(int i, const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (columns()!=v.size() || i < 0 || i > rows() ) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::setRow(" << i << ", ";
    ss << "VarVec<" << v.size() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int j=0,k=i*columns();j<columns();j++,k++) x[k] = v.x[j];
}

/** Set a column.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of col to set
 * @param v source vector
 */
template <class T>
inline void VarMat<T>::setCol(int j, const VarVec<T>& v) {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=v.size() || j < 0 || j > rows() ) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::setCol(" << j << ", ";
    ss << "VarVec<" << v.size() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int i=0,k=j;i<rows();i++,k+=columns()) x[k] = v.x[i];
}

/** Set a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 * @param m matrix to be placed with UL corner at (i,j)
 */
template <class T>
inline void VarMat<T>::setSubMat(int i, int j, const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+m.rows() > rows() || j < 0 || j+m.columns() > columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::setSubMat(" << i << ", " << j << ", ";
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int id=i,is=0;id<rows() && is<m.rows();id++,is++)
    for (int jd=j,js=0;jd<columns() && js<m.columns();jd++,js++)
      x[id*columns()+jd] = m.x[is*m.columns()+js];
}

// Casting Mutators

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T> template <class U>
inline void VarMat<T>::set(const VarMat<U>& m) {
  if(rows()!=m.rows()||columns()!=m.columns()) setSize(m.rows(),m.columns());
  for (int k=0;k<len;k++) x[k] = T(m.x[k]);
}

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T> template <class U, int M, int N>
inline void VarMat<T>::set(const Mat<U,M,N>& m) {
  if(rows()!=M||columns()!=N) setSize(M,N);
  for (int k=0;k<len;k++) x[k] = T(m.x[k]);
}

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T> template <class U>
inline VarMat<T>& VarMat<T>::operator = (const VarMat<U>& m) {
  set<U>(m);
}

/** Set from another matrix with type cast.
 * @param m the matrix to replicate.
 */
template <class T> template <class U, int M, int N>
inline VarMat<T>& VarMat<T>::operator = (const Mat<U,M,N>& m) {
  set<U,M,N>(m);
}

// Accessors

/** Returns the number of matrix elements
 * @return the number of matrix elements
 */
template <class T>
inline int VarMat<T>::length() const {
  return len;
}

/** Get a row.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of row to get
 */
template <class T>
inline VarVec<T> VarMat<T>::getRow(int i) const {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i >= rows()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::getRow(" << i;
    ss << "): index out of range\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> v(columns());
  for (int j=0,k=i*columns();j<columns();j++,k++) v.x[j] = x[k];
  return v;
}

/** Get a column.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param j zero based index of row to get
 * @return a column vector
 */
template <class T>
inline VarVec<T> VarMat<T>::getCol(int j) const {
#if MATMATH_CHECK_BOUNDS
  if (j < 0 || j >= columns()) {
    std::stringstream ss;
    ss << "Mat<" << rows() << "," << columns() << ">::getCol(" << j;
    ss << "): index out of range\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> v(rows());
  for (int i=0,k=j;i<rows();i++,k+=columns()) v.x[i] = x[k];
  return v;
}

/** Get a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 * @param sub_rows number of rows in sub-matrix
 * @param sub_cols number of columns in sub-matrix
 * @return a sub-matrix
 */
template <class T>
inline VarMat<T> VarMat<T>::getSubMat(int i, int j, int sub_rows, int sub_cols) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+sub_rows > rows() || j < 0 || j+sub_cols > columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::getSubMat(" << i;
    ss << ", " << j << "): index out of range\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<T> m(sub_rows,sub_cols,T(0));
  for (int id=i,is=0;id<rows() && is<sub_rows;id++,is++)
    for (int jd=j,js=0;jd<columns() && js<sub_cols;jd++,js++)
      m.x[is*sub_cols+js] = x[id*columns()+jd];
  return m;
}

// Addition and subtraction: <matrix> +/- <matrix>

/** Addition.
 */
template <class T>
inline VarMat<T>& VarMat<T>::add(const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::add (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int k=0;k<len;k++) x[k] = x[k] + m.x[k];
  return *this;
}

/** Add a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 * @param m matrix to be placed with UL corner at (i,j)
 */
template <class T>
inline void VarMat<T>::addSubMat(int i, int j, const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+m.rows() > rows() || j < 0 || j+m.columns() > columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::setSubMat(" << i << ", " << j << ", ";
    ss << "VarVec<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int id=i,is=0;id<rows() && is<m.rows();id++,is++)
    for (int jd=j,js=0;jd<columns() && js<m.columns();jd++,js++)
      x[id*columns()+jd] += m.x[is*m.columns()+js];
}

/** Subtraction.
 */
template <class T>
inline VarMat<T>& VarMat<T>::subtract(const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::subtract (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int k=0;k<len;k++) x[k] = x[k] - m.x[k];
  return *this;
}

/** Subtract a sub-matrix.
 * Does bounds checking if MATMATH_CHECK_BOUNDS is set
 * @param i zero based index of top row of sub-matrix
 * @param j zero based index of left col of sub-matrix
 * @param m matrix to be placed with UL corner at (i,j)
 */
template <class T>
inline void VarMat<T>::subtractSubMat(int i, int j, const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (i < 0 || i+m.rows() > rows() || j < 0 || j+m.columns() > columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::setSubMat(" << i << ", " << j << ", ";
    ss << "VarVec<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int id=i,is=0;id<rows() && is<m.rows();id++,is++)
    for (int jd=j,js=0;jd<columns() && js<m.columns();jd++,js++)
      x[id*columns()+jd] -= m.x[is*m.columns()+js];
}

// Addition and subtraction operators: <matrix> +/- <matrix>

/** Addition operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator + (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator + (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<T> mp(rows(),columns());
  for (int k=0;k<len;k++) mp.x[k] = x[k] + m.x[k];
  return mp;
}

/** Addition assignment operator.
 */
template <class T>
inline void VarMat<T>::operator += (const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator += (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int k=0;k<len;k++) x[k] += m.x[k];
}

/** Subtraction operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator - (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator - (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<T> mp(rows(),columns());
  for (int k=0;k<len;k++) mp.x[k] = x[k] - m.x[k];
  return mp;
}

/** Subtraction assignment operator.
 */
template <class T>
inline void VarMat<T>::operator -= (const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator -= (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  for (int k=0;k<len;k++) x[k] -= m.x[k];
}

/** Negation operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator - () const {
  VarMat<T> mp(rows(),columns());
  for (int k=0;k<len;k++) mp.x[k] = -x[k];
  return mp;
}

// Addition and subtraction: <matrix> +/- <scalar>

/** Addition.
 */
template <class T>
inline VarMat<T>& VarMat<T>::add(const T a) {
  for (int k=0;k<len;k++) x[k] = x[k] + a;
  return *this;
}

/** Subtraction.
 */
template <class T>
inline VarMat<T>& VarMat<T>::subtract(const T a) {
  for (int k=0;k<len;k++) x[k] = x[k] - a;
  return *this;
}

// Addition and subtraction operators: <matrix> +/- <scalar

 /** Addition operator.
  */
 template <class T>
 inline VarMat<T> VarMat<T>::operator + (const T a) const {
   VarMat<T> mp(rows(),columns());
   for (int k=0;k<len;k++) mp.x[k] = x[k] + a;
   return mp;
 }

 /** Addition assignment operator.
  */
 template <class T>
 inline void VarMat<T>::operator += (const T a) {
   for (int k=0;k<len;k++) x[k] += a;
 }

 /** Subtraction operator.
  */
 template <class T>
 inline VarMat<T> VarMat<T>::operator - (const T a) const {
   VarMat<T> mp(rows(),columns());
   for (int k=0;k<len;k++) mp.x[k] = x[k] - a;
   return mp;
 }

 /** Subtraction assignment operator.
  */
 template <class T>
 inline void VarMat<T>::operator -= (const T a) {
   for (int k=0;k<len;k++) x[k] -= a;
 }

// Multiplication and division: <matrix> *// <scalar>

/** Matrix-Scalar multiplication operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator * (const T a) const {
  VarMat<T> m(rows(),columns());
  for (int k=0;k<len;k++) m.x[k] = x[k]*a;
  return m;
}

/** Matrix-Scalar multiplication assignement operator.
 */
template <class T>
inline void VarMat<T>::operator *= (const T a) {
  for (int k=0;k<len;k++) x[k] *= a;
}

/** Matrix-Scalar division operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator / (const T a) const {
  VarMat<T> m(rows(),columns());
  for (int k=0;k<len;k++) m.x[k] = x[k]/a;
  return m;
}

/** Matrix-Scalar division assignement operator.
 */
template <class T>
inline void VarMat<T>::operator /= (const T a) {
  for (int k=0;k<len;k++) x[k] /= a;
}

/** Scalar-Matrix multiplication operator.
 */
template <class T>
VarMat<T> operator * (const T a, const VarMat<T> &m) {
  VarMat<T> mp(m.rows(),m.columns());
  for (int k=0;k<m.length();k++) mp.x[k] = m.x[k]*a;
  return mp;
}

// Multiplication: <matrix> * <vector>

/** Matrix-Vector multiplication operator.
 */
template <class T>
inline VarVec<T> VarMat<T>::operator * (const VarVec<T>& v) const {
#if MATMATH_CHECK_BOUNDS
  if (columns()!=v.length()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator * (" <<
    ss << "VarVec<" << v.length() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> vp(rows(),T(0));
  for (int i=0,k=0;i<rows();i++)
    for (int j=0;j<columns();j++,k++)
      vp.x[i] += x[k]*v.x[j];
  return vp;
}

/** Vector-Matrix multiplication operator.
 */
template <class T>
VarVec<T> operator * (const VarVec<T>& v, const VarMat<T> &m) {
#if MATMATH_CHECK_BOUNDS
  if (m.rows()!=v.length()) {
    std::stringstream ss;
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">::operator * (";
    ss << "VarVec<" << v.length() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarVec<T> vp(m.columns(),T(0));
  for (int i=0,k=0;i<m.rows();i++)
    for (int j=0;j<m.columns();j++,k++)
      vp.x[j] += m.x[k]*v.x[i];
  return vp;
}

// Multiplication: <matrix> * <matrix>

/** Matrix-Matrix multiplication operator.
 */
template <class T>
inline VarMat<T> VarMat<T>::operator * (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (columns()!=m.rows()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator * (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<T> mp(rows(),m.columns(),T(0));
  for (int i=0;i<rows();i++)
    for (int j=0;j<m.columns();j++)
      for (int k=0;k<columns();k++)
        mp.x[i*m.columns()+j] += x[i*columns()+k]*m.x[k*m.columns()+j];
  return mp;
}

/** Matrix-Matrix multiplication assignement operator.
 */
template <class T>
inline void VarMat<T>::operator *= (const VarMat<T>& m) {
#if MATMATH_CHECK_BOUNDS
  if (columns()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator *= (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  (*this) = (*this)*m;
}

// Equality and inequality tests.

/** Element-wise test for equality.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator == (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator == (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] == m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for inequality.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator != (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator != (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] != m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater or equal.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator >= (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator >= (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] >= m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less or equal.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator <= (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator <= (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << "): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] <= m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for greater.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator > (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator > (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] > m.x[i]) b.x[i] = true;
  return b;
}

/** Element-wise test for less.
 * @return matrix of boolean results
 */
template <class T>
inline VarMat<bool> VarMat<T>::operator < (const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::operator < (" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  VarMat<bool> b(rows(),columns(),false);
  for (int i=0;i<len;i++) if (x[i] < m.x[i]) b.x[i] = true;
  return b;
}

/** Compare the entire matrix to the passed matrix.
 * @returns 1 if all element in this matrix are greater than corresponding
 * elements in the passed matrix, -1 if they are all less, and 0 otherwise
 */
template <class T>
inline int VarMat<T>::compareTo(const VarMat<T>& m) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::compareTo(" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  int g=0, l=0;
  for (int i=0;i<len;i++) {
    if (x[i] < m.x[i]) l++;
    if (x[i] > m.x[i]) g++;
  }
  if (l==(len)) return -1;
  else if (g==(len)) return 1;
  else return 0;
}

/** Compare the entire matrix to the passed matrix.
 * @returns true of all corresponding elements are within the given tolerance
 */
template <class T>
inline bool VarMat<T>::equalTo(const VarMat<T>& m, const T tol) const {
#if MATMATH_CHECK_BOUNDS
  if (rows()!=m.rows() || columns()!=m.columns()) {
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::equalTo(" <<
    ss << "VarMat<" << m.rows() << "," << m.columns() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  bool t = true;
  for (int i=0;i<len;i++) if (fabs(x[i] - m.x[i]) > tol) t = false;
  return t;
}

// Transpose

/** Return the transpose of the matrix without modifying matrix.
 */
template <class T>
inline VarMat<T> VarMat<T>::transposed() const {
  VarMat<T> mp(columns(),rows());
  for (int i=0;i<rows();i++)
    for (int j=0;j<columns();j++)
      mp.x[j*rows()+i] = x[i*columns()+j];
  return mp;
}

// Random matrices

/** Create matrix of samples from a uniform distribution on [a,b].
 * @param a start of range
 * @param b end of range
 * @return matrix of uniform samples
 */
template <class T>
inline VarMat<T> VarMat<T>::uniformRand(const T a, const T b) {
  VarMat<T> m(rows(),columns());
  for (int i=0;i<len;i++) m.x[i] = rtc_uniform_rand<T>(a,b);
  return m;
}

/** Create matrix of samples from a normal distribution.
 * @param mean mean of normal distribution
 * @param stdev standard deviation of normal distribution
 * @return matrix of normal samples
 */
template <class T>
inline VarMat<T> VarMat<T>::normalRand(const T mean, const T stdev) {
  VarMat<T> m(rows(),columns());
  for (int i=0;i<len;i++) m.x[i] = rtc_normal_rand<T>(mean,stdev);
  return m;
}

//  /** Create matrix of samples from a multivariate gaussian distribution.
//   * @param mean mean of normal distribution
//   * @param stdev standard deviation of normal distribution
//   * @return matrix of normal samples
//   */
//  template <class T>
//  inline VarMat<T> VarMat<T>::multivariateGauss(const Vec<T,M>& mean, const SMat<T,M>& cov) {
//    VarMat<T> m;
//    SMat<T,M> S(cov);
//    int n=S.choleskyDecomp();
//    assert(n==0);
//    S.transpose();
//    VarMat<T> X = normalRand();
//    for(int j=0;j<N;++j) m.setCol(j,mean);
//    m = m + S*X;
//    return m;
//  }
//
// Serialization routines

/** Write state to stream as formated ASCII
 */
template <class T>
std::ostream& operator<<(std::ostream& os, const VarMat<T>& mat) {

  int minFieldWidth = os.precision()+2;

  //case with 1 row
  if (mat.rows() == 1){
    os << "[" ;
    for (int i=0; i<mat.columns(); ++i)
      os << std::setw(minFieldWidth) << mat(0,i) << " ";

    os << "]" << std::endl;
  }
  //case with 2+ rows
  else{
    //write first row
    os << "[" ;
    for (int j=0; j<mat.columns()-1; ++j){
      os << std::setw(minFieldWidth) << mat(0,j) << " ";
    }
    os << std::setw(minFieldWidth) << mat(0,mat.columns()-1) << ";" << std::endl;
    //write middle rows
    for (int i=1;i<mat.rows()-1;++i){
      for (int j=0;j<mat.columns();++j){
        os << " " << std::setw(minFieldWidth) << mat(i,j);
        }
      os << ";" << std::endl;
      }
    //write last row
    for (int j=0; j<mat.columns(); ++j){
      os << " " << std::setw(minFieldWidth) << mat(mat.rows()-1,j);
    }
    os << "]" << std::endl;
  }

  return os;
}

/** Restores state data from formated ASCII stream
 */
template <class T>
std::istream& operator>>(std::istream& is, VarMat<T>& mat){
  using namespace std;
  vector<T> data;
  string sizeString;
  string matString;
  stringstream matStringStream;
  string rowString;
  stringstream rowStringStream;
  int sPos;

  getline(is, matString, ']');
  sPos = matString.find('[');
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
  if (rowCount != mat.rows() || colCount != mat.columns())
    mat.setSize(rowCount,colCount);

  //copy extracted data
  for (int i=0;i<mat.len;i++){
    mat.x[i] = data[i];
  }

  return is;
}

// General elementwise operations

/** Pass each element through the given single-arg math function,
replacing the current vector with the result.
*/
template <class T>
inline void VarMat<T>::perform(T (*mathFun)(T)) {
  for (int i=0;i<len;i++) x[i] = (*mathFun)(x[i]);
}

/** Find the sum of the elements in each column in the matrix
 * @returns the sum of the elements in each column in the matrix
 */
template <class T>
inline VarVec<T> VarMat<T>::sum() const {
  VarVec<T> s(columns(),T(0));
  for (int j=0;j<columns();j++)
    for (int i=0;i<rows();i++) s.x[j] += x[i*columns()+j];
  return s;
}

/**Use bilinear interpolation to approximate values
 * between the elements of matrices.
 * @returns the bilinear interpolation at i,j
 */
template <class T>
inline T VarMat<T>::interpolate(const float i, const float j) const {
  int r = rows();
  int c = columns();
  int truncR = rtc_clamp(static_cast<int>(i),0,r-1);
  int truncR1 = rtc_clamp(truncR+1,0,r-1);
  const float fractR = i - static_cast<float>(truncR);
  int truncC = rtc_clamp(static_cast<int>(j),0,c-1);
  int truncC1 = rtc_clamp(truncC+1,0,c-1);
  const float fractC = j - static_cast<float>(truncC);

  // do the interpolation
  const T syx = x[truncR*c+truncC];
  const T syx1 = x[truncR*c+truncC1];
  const T sy1x = x[truncR1*c+truncC];
  const T sy1x1 = x[truncR1*c+truncC1];
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
template <class T>
inline VarVec<T> VarMat<T>::meanOfRows() const {
  VarVec<T> m(columns(),T(0));
  for (int j=0;j<columns();j++)
    for (int i=0;i<rows();i++)
      m.x[j] += x[i*columns()+j];
  m/=static_cast<T>(rows());
  return m;
}

/*
  * This is a new, faster version, written by Gu Xin:
  */
template <class T>
inline VarSMat<T> VarMat<T>::covarianceMatrixOfRows() const {
  VarSMat<T> dest(columns(),T(0));

  //Implementation for the sum of Matrix[X];
  if (rows()<2) { //just one row
    std::stringstream ss;
    ss << "VarMat<" << rows() << "," << columns() << ">::covarianceMatrixOfRows(): " <<
    ss << "matrix has to have more than one row\n";
    throw Exception(ss.str());
  }

  // mean vector
  VarVec<T> mu(columns(),T(0));

  /*
   * the loop gets out the result of the Matrix[X];
   * Matrix[X]=sum(Matrix[X(i)]);-- i from 0 to n-1--
   */
  for(int i=0;i<rows();i++) {
    VarVec<T> tmpV = getRow(i);
    mu.add(tmpV);
    VarSMat<T> tmp = tmpV.outer(tmpV);
    dest.add(tmp); //add the Matrix[X(i)] to Matrix[sumMatrix];
  }
  //End of the Implementation for the sum of Matrix[X];

  //Implementation for the Matrix[meanOf];
  mu/=static_cast<T>(rows());

  VarSMat<T> tmp = mu.outer(mu);
  tmp *= static_cast<T>(rows());

  //get the result of the difference from Matrix[X] and Matrix[meanOf];
  dest.subtract(tmp);
  dest/=static_cast<T>(rows()-1);
  return dest;
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VARMAT_H defined
//==============================================================================

