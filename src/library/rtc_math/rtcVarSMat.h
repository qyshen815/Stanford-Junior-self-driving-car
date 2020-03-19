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
 * file .......: rtcVarSMat.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_VARSMAT_H
#define RTC_VARSMAT_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVarVec.h"
#include "rtcVarMat.h"
#include "rtcArray2.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T> class VarVec; // r-d vector
template <class T> class VarMat; // MxN Matrix
template <class T> class VarSMat; // MxM Square Matrix

/**
 * A square matrix.
 * A specialization of a general matrix that provides operations only
 * possible on square matrices like guassian elimination and Cholesky
 * decomposition.
 */
template <class T>
class VarSMat: public VarMat<T> {
public:
  // Data
  using VarMat<T>::x;
  using VarMat<T>::set;
  using VarMat<T>::rows;

  // Constructors
  VarSMat();
  VarSMat(int size);
  VarSMat(int size, const T* d);
  VarSMat(int size, const T diagVal);
  VarSMat(int size, const VarVec<T>& diagVec);
  VarSMat(const Array2<T>& m);

  // Casting Operation
  template <class U> VarSMat(const VarMat<U>& m);

  // Mutators
  void setSize(int size);
  void setIdentity();
  void setDiag(const T a);
  void setDiag(const VarVec<T>& diagVec);

  // Accessors
  VarVec<T> getDiag();
  int size() const;

  // Basic square matrix functions
  T trace() const;
  void transpose();
  VarSMat<T> minorMat(const int ip, const int jp) const;
  T det() const;
  VarSMat<T> inverted() const;
  int invert();

  // Decompositions
  void luDecomp(VarVec<int>& indx, T* d = NULL);
  void luSolve(const VarVec<int>& indx, VarVec<T>& b);
  void choleskyDecomp();
  void choleskyDecomp(VarSMat<T>& r);
  void choleskySolve(VarVec<T>& b);
}; // end class VarSMat<T>

// Declare a few common typdefs
typedef VarSMat<bool> VarSMatb;
typedef VarSMat<char> VarSMatc;
typedef VarSMat<unsigned char> VarSMatuc;
typedef VarSMat<int> VarSMati;
typedef VarSMat<float> VarSMatf;
typedef VarSMat<double> VarSMatd;

//==============================================================================
// VarSMat<T>
//==============================================================================

// Constructors

/** default Ctor
*/
template <class T>
inline VarSMat<T>::VarSMat() : VarMat<T>() {}

/** Ctor that doesn't initialize anything.
 */
template <class T>
inline VarSMat<T>::VarSMat(int size) : VarMat<T>(size,size) {}

/** Ctor that initializes from an array.
 * @param size is the number of rows/columns
 * @param d the (row major) data array of length r*r
 */
template <class T>
inline VarSMat<T>::VarSMat(int size, const T* d) : VarMat<T>(size,size,d) {}

/** Ctor that makes a multiple of the identity matrix.
 * @param size is the number of rows/columns
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline VarSMat<T>::VarSMat(int size, const T diagVal) : VarMat<T>(size,size) {
  set(T(0));
  setDiag(diagVal);
}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param size is the number of rows/columns
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline VarSMat<T>::VarSMat(int size, const VarVec<T>& diagVec) : VarMat<T>(size,size) {
  set(T(0));
  setDiag(diagVec);
}

/** Ctor that initializes from a Mat<T,r,r>
 */
template <class T>
inline VarSMat<T>::VarSMat(const Array2<T>& m) : VarMat<T>(m) {}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,r,r>
 */
template <class T> template <class U>
inline VarSMat<T>::VarSMat(const VarMat<U>& m) : VarMat<T>(m) {}

// Mutators

/** Sets matrix to specific size
 * @param size is the number of rows/columns
 */
template <class T>
inline void VarSMat<T>::setSize(int size) {
  VarMat<T>::setSize(size,size);
}

/** Sets matrix to an identiy matrix, where the diagonal elements are equal to
 * 1 and every other element equals 0.
 */
template <class T>
inline void VarSMat<T>::setIdentity() {
  set(T(0));
  setDiag(T(1));
}

/** Sets matrix to a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T>
inline void VarSMat<T>::setDiag(const T diagVal) {
  for (int i=0,k=0;i<size();i++,k+=(size()+1))
    x[k] = diagVal;
}

/** Sets the diagonal entries of matrix from a vector.
 * without changing other entries.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T>
inline void VarSMat<T>::setDiag(const VarVec<T>& diagVec) {
#if MATMATH_CHECK_BOUNDS
  if (size()!=diagVec.length()) {
    std::stringstream ss;
    ss << "VarSMat<" << size() << "," << size() << ">::setDiag (";
    ss << "VarVec<" << diagVec.length() << ">): not a valid operation\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  for (int i=0,k=0;i<size();i++,k+=(size()+1))
    x[k] = diagVec.x[i];
}

// Accessors

/** Get the diagonal entries of the matrix.
 * @return vector of the diagonal entries.
 */
template <class T>
inline VarVec<T> VarSMat<T>::getDiag() {
  VarVec<T> v(size());
  for (int i=0,k=0;i<size();i++,k+=(size()+1)) v.x[i] = x[k];
  return v;
}

/**
 * @return the size of the matrix
 */
template <class T>
inline int VarSMat<T>::size() const {
  return rows();
}

// Some basic square matrix functions

/** Return the trace of the matrix
 */
template <class T>
inline T VarSMat<T>::trace() const {
  T sum = T(0);
  for (int i=0,k=0;i<size();i++,k+=(size()+1)) sum += x[k];
  return sum;
}

/** Transposes this matrix in place, overwriting old data
 */
template <class T>
inline void VarSMat<T>::transpose() {
  for (int i=0;i<size();i++) for (int j=i+1;j<size();j++) rtc_swap(x[i*size()+j],x[j*size()+i]);
}

/** Return the minor of this matrix about element (ip,jp)
 */
template <class T>
inline VarSMat<T> VarSMat<T>::minorMat(const int ip, const int jp) const {
#if MATMATH_CHECK_BOUNDS
  if (ip<0 || ip>size() || jp<0 || jp>size()) {
    std::stringstream ss;
    ss << "VarSMat<" << size() << ">::minorMat(" << ip;
    ss << ", " << jp << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  VarSMat<T> m(size()-1);
  for (int i=0,k=0;i<size();++i) if (i != ip) {
    for (int j=0,l=0;j<size();++j) if (j != jp) {
      m.x[k*(size()-1)+l] = x[i*size()+j];
      ++l;
    }
    ++k;
  }
  return m;
}

/** Return the determinant of this matrix.
 * This uses the LU decomposition.  Matrices of size 3 and under have
 * specialized implementations.
 */
template <class T>
inline T VarSMat<T>::det() const {
  VarSMat<T> tmp(*this);
  VarVec<int> indx(size()); T d;
  tmp.luDecomp(indx,&d);
  return d*tmp.getDiag().prod();
}

/** Return the inverse of this matrix
 * This uses the LU decomposition
 */
template <class T>
inline VarSMat<T> VarSMat<T>::inverted() const {
  VarSMat<T> tmp(*this);
  VarVec<int> indx(size()); T d;
  try {
    tmp.luDecomp(indx,&d);
  }
  catch (Exception e) {
    throw Exception( "VarSMat::invert(): error, can't take inverse of singular matrix.");
  }
  VarSMat<T> inv(size());
  for (int i=0;i<size();i++) {
    VarVec<T> col(size(),T(0));
    col(i) = T(1);
    tmp.luSolve(indx,col);
    inv.setCol(i,col);
  }
  return inv;
}

/** Return the inverse of this matrix
 * This uses the LU decomposition
 */
template <class T>
inline int VarSMat<T>::invert() {
  VarSMat<T> tmp(*this);
  VarVec<int> indx(size()); T d;
  try {
    tmp.luDecomp(indx,&d);
  }
  catch (Exception e) {
    throw Exception( "VarSMat::inverted(): error, can't take inverse of singular matrix.");
  }
  for (int i=0;i<size();i++) {
    VarVec<T> col(size(),T(0));
    col(i) = T(1);
    tmp.luSolve(indx,col);
    this->setCol(i,col);
  }
  return 0;
}

// Decomposition and Solve Routines (compliments of Numerical Recipes in C)

/** Perform the LU Decomposition in place
 * @return indx output vector that records the row permutation effected by
 * the partial pivoting
 * @return d (optional) is +/- 1 depending on if there were an even (+) or
 * odd (-) number of row-interchanges (used to compute determinant).
 * @returns 1 on failure (matrix singular), 0 on success
 */
template <class T>
inline void VarSMat<T>::luDecomp(VarVec<int>& indx, T* d)
{
  int r = size();
  if(r!=indx.length()) indx.setSize(r);
  int i,imax=0,j,k;
  T big,dum,sum,temp;
  VarVec<T> vv(r);

  if (d)
    *d=1.0;
  for (i=0; i<r; i++) {
    big=0.0;
    for (j=0; j<r; j++)
      if ((temp=fabs(x[i*r+j])) > big)
        big=temp;
    if (big == 0.0) {
      std::stringstream ss;
      ss << "VarSMat<" << r << "," << r << ">::luDecomp(VarVec<" << r << ">& indx, T* d)";
      ss << ": matrix is singular.";
      throw Exception(ss.str());
    }
    vv(i)=T(1)/big;
  }
  for (j=0; j<r; j++) {
    for (i=0; i<j; i++) {
      sum=x[i*r+j];
      for (k=0; k<i; k++)
        sum -= x[i*r+k]*x[k*r+j];
      x[i*r+j]=sum;
    }
    big=0.0;
    for (i=j; i<r; i++) {
      sum=x[i*r+j];
      for (k=0; k<j; k++)
        sum -= x[i*r+k]*x[k*r+j];
      x[i*r+j]=sum;
      if ((dum=vv(i)*fabs(sum)) >= big) {
        big=dum;
        imax=i;
      }
    }
    if (j != imax) {
      for (k=0; k<r; k++) {
        dum=x[imax*r+k];
        x[imax*r+k]=x[j*r+k];
        x[j*r+k]=dum;
      }
      if (d)
        *d = -(*d);
      vv(imax)=vv(j);
    }
    indx.x[j]=imax;
    if (x[j*r+j] == 0.0)
      x[j*r+j] = T(1e-20);
    if (j != r-1) {
      dum=T(1)/(x[j*r+j]);
      for (i=j+1; i<r; i++)
        x[i*r+j] *= dum;
    }
  }
}

/** Perform a LU Decomposition backsolve with given RHS.
 * Precondtion: luDecomp(indx,d) has already been called
 * @param indx the row-swap vector returned from luDecomp()
 * @param b the RHS vector in the equation A*x = b
 * @return b is also the solution, overwriting the passed RHS
 */
template <class T>
inline void VarSMat<T>::luSolve(const VarVec<int>& indx, VarVec<T>& b)
{
#if MATMATH_CHECK_BOUNDS
  if (size()!=indx.length() || size()!=b.length()) {
    std::stringstream ss;
    ss << "VarSMat<" << size() << "," << size() << ">::luSolve (";
    ss << "VarVec<" << indx.length() << ">, VarVec<" << b.length() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  int i,ii=-1,ip,j;
  T sum;
  for (i=0;i<size();i++) {
    ip=indx.x[i]; sum=b.x[ip]; b.x[ip]=b.x[i];
    if (ii>=0) for (j=ii;j<i;j++) sum -= x[i*size()+j]*b.x[j];
    else if (sum) ii=i;
    b.x[i]=sum;
  }
  for (i=(size()-1);i>=0;i--) {
    sum=b.x[i];
    for (j=i+1;j<size();j++) sum -= x[i*size()+j]*b.x[j];
    b.x[i]=sum/x[i*size()+i];
  }
}

/** Perform the Cholesky Decomposition in place.
 * Only the upper triangle (including the diagonal) is used as source data.
 * The result is stored in the lower triangle (including the diagonal).
 * @returns 1 on failure (matrix not positive definite), 0 on success
 */
template <class T>
inline void VarSMat<T>::choleskyDecomp()
{
  int r = size();
  T sum;
  T p[r];
  for (int i=0; i<r; i++)
    for (int j=i, k; j<r; j++) {
      for (sum=x[i*r+j], k=i-1; k>=0; k--)
        sum -= x[i*r+k]*x[j*r+k];
      if (i == j) {
        if (sum <= T(0)) {
          std::stringstream ss;
          ss << "VarSMat<" << r << "," << r << ">::choleskyDecomp()";
          ss << ": matrix is not positive definite.";
          throw Exception(ss.str());
        }
        p[i] = sqrt(double(sum));
      } else
        x[j*r+i] = sum/p[i];
    }
  for (int i=0; i<r; i++)
    x[i*r+i] = p[i];
}

/** Perform the Cholesky Decomposition and stores the result in r.
 * @returns 1 on failure (matrix not positive definite), 0 on success
 */
template <class T>
inline void VarSMat<T>::choleskyDecomp(VarSMat<T>& _r)
{
  _r.set(*this);
  _r.choleskyDecomp();
  for (int i=0;i<size()-1;i++)
    for (int j=i+1;j<size();j++)
      _r.x[i*size()+j]=T(0);
  _r.transpose();
}

/** Perform a Cholesky Decomposition backsolve with given RHS.
 * Precondtion: choleskyDecomp() has already been called
 * @param b the RHS vector in the equation A*x = b
 * @return b is also the solution, overwriting the passed RHS
 */
template <class T>
inline void VarSMat<T>::choleskySolve(VarVec<T>& b) {
#if MATMATH_CHECK_BOUNDS
  if (size()!=b.length()) {
    std::stringstream ss;
    ss << "VarSMat<" << size() << "," << size() << ">::choleskySolve (";
    ss << "VarVec<" << b.length() << ">): not a valid operation\n";
    throw Exception(ss.str());
  }
#endif
  T sum;
  for (int i=0,k;i<size();i++) {
    for (sum=b.x[i],k=i-1;k>=0;k--) sum -= x[i*size()+k]*b.x[k];
    b.x[i] = sum/x[i*size()+i];
  }
  for (int i=size()-1,k;i>=0;i--) {
    for (sum=b.x[i],k=i+1;k<size();k++) sum -= x[k*size()+i]*b.x[k];
    b.x[i] = sum/x[i*size()+i];
  }
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_VARSMAT_H defined
//==============================================================================
