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
 * file .......: rtcSMat.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_SMAT_H
#define RTC_SMAT_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"
#include "rtcMat.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T, int M, int N> class Mat; // MxN Matrix
template <class T, int M> class SMat; // MxM Square Matrix

/**
 * A square matrix.
 * A specialization of a general matrix that provides operations only
 * possible on square matrices like guassian elimination and Cholesky
 * decomposition.
 */
template <class T, int M>
class SMat: public Mat<T,M,M> {
public:
  // Data
  using Mat<T,M,M>::x;
  using Mat<T,M,M>::set;

  // Constructors
  SMat();
  SMat(const T* d);
  SMat(const T diagVal);
  SMat(const Vec<T,M>& diagVec);
  SMat(const Mat<T,M,M>& m);

  // Casting Operation
  template <class U> SMat(const Mat<U,M,M>& m);

  // Mutators
  void setIdentity();
  void setDiag(const T a);
  void setDiag(const Vec<T,M>& diagVec);
  SMat<T,M>& leftMultiply(const SMat<T,M>& m);

  // Accessors
  Vec<T,M> getDiag();

  // Basic square matrix functions
  T trace() const;
  void transpose();
  SMat<T,M-1> minorMat(const int ip, const int jp) const;
  T det() const;
  SMat<T,M> inverted() const;
  int invert();

  // Decompositions
  int luDecomp(Vec<int,M>& indx, T* d = NULL);
  void luSolve(const Vec<int,M>& indx, Vec<T,M>& b);
  int choleskyDecomp();
  int choleskyDecomp(SMat<T,M>& r);
  void choleskySolve(Vec<T,M>& b);
};

//==============================================================================
// SMat<T,M>
//==============================================================================

// Constructors

/** Ctor that doesn't initialize anything.
 */
template <class T, int M>
inline SMat<T,M>::SMat() {}

/** Ctor that initializes from an array.
 * @param d the (row major) data array of length M*M
 */
template <class T, int M>
inline SMat<T,M>::SMat(const T* d) : Mat<T,M,M>(d) {}

/** Ctor that makes a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T, int M>
inline SMat<T,M>::SMat(const T diagVal) {
  set(T(0));
  setDiag(diagVal);
}

/** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T, int M>
inline SMat<T,M>::SMat(const Vec<T,M>& diagVec) {
  set(T(0));
  setDiag(diagVec);
}

/** Ctor that initializes from a Mat<T,M,M>
 */
template <class T, int M>
inline SMat<T,M>::SMat(const Mat<T,M,M>& m) : Mat<T,M,M>(m) {}

// Casting Operation

/** Casting Ctor that initializes from a Mat<U,M,M>
 */
template <class T, int M> template <class U>
inline SMat<T,M>::SMat(const Mat<U,M,M>& m) : Mat<T,M,M>(m) {}

// Mutators

/** Sets matrix to an identiy matrix, where the diagonal elements are equal to
 * 1 and every other element equals 0.
 */
template <class T, int M>
inline void SMat<T,M>::setIdentity() {
  set(T(0));
  setDiag(T(1));
}

/** Sets matrix to a multiple of the identity matrix.
 * @param diagVal the value to which all diagonal entries will be set
 */
template <class T, int M>
inline void SMat<T,M>::setDiag(const T diagVal) {
  for (int i=0,k=0;i<M;i++,k+=(M+1))
    x[k] = diagVal;
}

/** Sets the diagonal entries of matrix from a vector.
 * without changing other entries.
 * @param diagVec the vector of values that should appear on the diagonal
 */
template <class T, int M>
inline void SMat<T,M>::setDiag(const Vec<T,M>& diagVec) {
  for (int i=0,k=0;i<M;i++,k+=(M+1))
    x[k] = diagVec.x[i];
}

/** Matrix-Matrix multiplication from the left.
 */
template <class T, int M>
inline SMat<T,M>& SMat<T,M>::leftMultiply(const SMat<T,M>& m) {
  (*this) = m*(*this);
  return *this;
}

// Accessors

/** Get the diagonal entries of the matrix.
 * @return vector of the diagonal entries.
 */
template <class T, int M>
inline Vec<T,M> SMat<T,M>::getDiag() {
  Vec<T,M> v;
  for (int i=0,k=0;i<M;i++,k+=(M+1)) v.x[i] = x[k];
  return v;
}

// Some basic square matrix functions

/** Return the trace of the matrix
 */
template <class T, int M>
inline T SMat<T,M>::trace() const {
  T sum = T(0);
  for (int i=0,k=0;i<M;i++,k+=(M+1)) sum += x[k];
  return sum;
}

/** Transposes this matrix in place, overwriting old data
 */
template <class T, int M>
inline void SMat<T,M>::transpose() {
  for (int i=0;i<M;i++) for (int j=i+1;j<M;j++) rtc_swap(x[i*M+j],x[j*M+i]);
}

/** Return the minor of this matrix about element (ip,jp)
 */
template <class T, int M>
inline SMat<T,M-1> SMat<T,M>::minorMat(const int ip, const int jp) const {
#if MATMATH_CHECK_BOUNDS
  if (ip<0 || ip>M || jp<0 || jp>M) {
    std::stringstream ss;
    ss << "SMat<" << M << ">::minorMat(" << ip;
    ss << ", " << jp << "): index out of range\n";
    ss << std::flush;
    throw Exception(ss.str());
  }
#endif
  SMat<T,M-1> m;
  for (int i=0,k=0;i<M;++i) if (i != ip) {
    for (int j=0,l=0;j<M;++j) if (j != jp) {
m.x[k*(M-1)+l] = x[i*M+j];
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
template <class T, int M>
inline T SMat<T,M>::det() const {
  SMat<T,M> tmp(*this);
  Vec<int,M> indx; T d;
  tmp.luDecomp(indx,&d);
  return d*tmp.getDiag().prod();
}

/** Return the inverse of this matrix
 * This uses the LU decomposition
 */
template <class T, int M>
inline SMat<T,M> SMat<T,M>::inverted() const {
  SMat<T,M> tmp(*this);
  Vec<int,M> indx; T d;
  if (tmp.luDecomp(indx,&d)) {
    throw Exception("SMat::inverted(): error, can't take the inverse of a singular matrix.");
  }
  SMat<T,M> inv;
  for (int i=0;i<M;i++) {
    Vec<T,M> col(T(0));
    col(i) = T(1);
    tmp.luSolve(indx,col);
    inv.setCol(i,col);
  }
  return inv;
}

/** Return the inverse of this matrix
 * This uses the LU decomposition
 */
template <class T, int M>
inline int SMat<T,M>::invert() {
  SMat<T,M> tmp(*this);
  Vec<int,M> indx; T d;
  if (tmp.luDecomp(indx,&d))
    throw Exception("SMat::invert(): error, can't take the inverse of a singular matrix.");

  for (int i=0;i<M;i++) {
    Vec<T,M> col(T(0));
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
template <class T, int M>
inline int SMat<T,M>::luDecomp(Vec<int,M>& indx, T* d)
{
  int i,imax=0,j,k;
  T big,dum,sum,temp;
  Vec<T,M> vv;

  if (d) *d=1.0;
  for (i=0;i<M;i++) {
    big=0.0;
    for (j=0;j<M;j++) if ((temp=fabs(x[i*M+j])) > big) big=temp;
    if (big == 0.0) return 1;
    vv(i)=T(1)/big;
  }
  for (j=0;j<M;j++) {
    for (i=0;i<j;i++) {
      sum=x[i*M+j];
      for (k=0;k<i;k++) sum -= x[i*M+k]*x[k*M+j];
      x[i*M+j]=sum;
    }
    big=0.0;
    for (i=j;i<M;i++) {
      sum=x[i*M+j];
      for (k=0;k<j;k++) sum -= x[i*M+k]*x[k*M+j];
      x[i*M+j]=sum;
      if ((dum=vv(i)*fabs(sum)) >= big) {
        big=dum;
        imax=i;
      }
    }
    if (j != imax) {
      for (k=0;k<M;k++) {
        dum=x[imax*M+k];
        x[imax*M+k]=x[j*M+k];
        x[j*M+k]=dum;
      }
      if (d) *d = -(*d);
      vv(imax)=vv(j);
    }
    indx.x[j]=imax;
    if (x[j*M+j] == 0.0) x[j*M+j] = T(1e-20);
    if (j != M-1) {
      dum=T(1)/(x[j*M+j]);
      for (i=j+1;i<M;i++) x[i*M+j] *= dum;
    }
  }
  return 0;
}

/** Perform a LU Decomposition backsolve with given RHS.
 * Precondtion: luDecomp(indx,d) has already been called
 * @param indx the row-swap vector returned from luDecomp()
 * @param b the RHS vector in the equation A*x = b
 * @return b is also the solution, overwriting the passed RHS
 */
template <class T, int M>
inline void SMat<T,M>::luSolve(const Vec<int,M>& indx, Vec<T,M>& b)
{
  int i,ii=-1,ip,j;
  float sum;
  for (i=0;i<M;i++) {
    ip=indx.x[i]; sum=b.x[ip]; b.x[ip]=b.x[i];
    if (ii>=0) for (j=ii;j<i;j++) sum -= x[i*M+j]*b.x[j];
    else if (sum) ii=i;
    b.x[i]=sum;
  }
  for (i=(M-1);i>=0;i--) {
    sum=b.x[i];
    for (j=i+1;j<M;j++) sum -= x[i*M+j]*b.x[j];
    b.x[i]=sum/x[i*M+i];
  }
}

/** Perform the Cholesky Decomposition in place.
 * Only the upper triangle (including the diagonal) is used as source data.
 * The result is stored in the lower triangle (including the diagonal).
 * @returns 1 on failure (matrix not positive definite), 0 on success
 */
template <class T, int M>
inline int SMat<T,M>::choleskyDecomp()
{
  T sum; T p[M];
  for (int i=0;i<M;i++) for (int j=i,k;j<M;j++) {
    for (sum=x[i*M+j],k=i-1;k>=0;k--) sum -= x[i*M+k]*x[j*M+k];
    if (i == j) {
      if (sum <= T(0)) return 1;
      p[i] = sqrt(double(sum));
    } else x[j*M+i] = sum/p[i];
  }
  for (int i=0;i<M;i++) x[i*M+i] = p[i];
  return 0;
}

/** Perform the Cholesky Decomposition and stores the result in r.
 * @returns 1 on failure (matrix not positive definite), 0 on success
 */
template <class T, int M>
inline int SMat<T,M>::choleskyDecomp(SMat<T,M>& r)
{
  r.set(*this);
  r.choleskyDecomp();
  for (int i=0;i<M-1;i++)
    for (int j=i+1;j<M;j++)
      r.x[i*M+j]=T(0);
  r.transpose();
  return 0;
}

/** Perform a Cholesky Decomposition backsolve with given RHS.
 * Precondtion: choleskyDecomp() has already been called
 * @param b the RHS vector in the equation A*x = b
 * @return b is also the solution, overwriting the passed RHS
 */
template <class T, int M>
inline void SMat<T,M>::choleskySolve(Vec<T,M>& b)
{
  T sum;
  for (int i=0,k;i<M;i++) {
    for (sum=b.x[i],k=i-1;k>=0;k--) sum -= x[i*M+k]*b.x[k];
    b.x[i] = sum/x[i*M+i];
  }
  for (int i=M-1,k;i>=0;i--) {
    for (sum=b.x[i],k=i+1;k<M;k++) sum -= x[k*M+i]*b.x[k];
    b.x[i] = sum/x[i*M+i];
  }
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_SMAT_H defined
//==============================================================================
