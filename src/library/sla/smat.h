//-*-c++-*-
#ifndef SMAT_H
#define SMAT_H

/** 
 * @file mat.h
 * @brief Basic static square matric type, templated in type and size
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 forked off from matmath.h
 *
 * @todo this seems to be working pretty well, but it needs to be looked at
 * again to check for bugs one final time.  All functionality is complete, 
 * however, except possibly some eigenvalue routines.
 */

#include <mathutil.h>
#include <mat.h>

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

  //////////////////// SMat ////////////////////
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
    void setDiag(const T a);
    void setDiag(const Vec<T,M>& diagVec);

    // Accessors
    Vec<T,M> getDiag();
    
    // Basic square matrix functions
    T trace() const;
    void transpose();
    T det() const;
    SMat<T,M> inverted() const;
    int invert();
    void symmetrize();
    SMat<T,M> symmetrized() const;
    
    // Decompositions
    int luDecomp(Vec<int,M>& indx, T* d = NULL);
    void luSolve(const Vec<int,M>& indx, Vec<T,M>& b);
    int choleskyDecomp(const int active_m = M);
    int choleskyUpdate(const int j);
    template <int N> void choleskySolve(Mat<T,M,N>& B, Mat<T,M,N>& Result, const int active_m = M, const int active_n = N);
    void choleskySolve(Vec<T,M>& b, Vec<T,M>& result, const int active_m = M);
    void choleskySolve(T* b_x, T* result_x, const int active_m = M);
  }; // end class SMat<T,M>

  // Non-member functions that involve this type
  template <class T, int M> 
  void operator *= (const Vec<T,M>& v, const SMat<T,M>& m);
  
  // Declare a few common typdefs
  typedef SMat<bool,5> SMat5b;
  typedef SMat<char,5> SMat5c;
  typedef SMat<unsigned char,5> SMat5uc;
  typedef SMat<int,5> SMat5i;
  typedef SMat<float,5> SMat5f;
  typedef SMat<double,5> SMat5d;
  
  typedef SMat<bool,6> SMat6b;
  typedef SMat<char,6> SMat6c;
  typedef SMat<unsigned char,6> SMat6uc;
  typedef SMat<int,6> SMat6i;
  typedef SMat<float,6> SMat6f;
  typedef SMat<double,6> SMat6d;


  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////

  //////////////////// SMat ////////////////////

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
  
  /** Vector-Matrix multiplication asignment operator.
   */
  template <class T, int M> 
  void operator *= (const Vec<T,M>& v, const SMat<T,M>& m) {
    Vec<T,M> vp(T(0));
    for (int j=0,k=0;j<M;j++) for (int i=0;i<M;i++,k++) vp.x[j]+=m.x[k]*v.x[i];
    v.set(vp);
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
    for (int j=0;j<M;j++) for (int i=j+1;i<M;i++) swap(x[i+j*M],x[j+i*M]);
  }

  /** Symmetrizes this matrix in place by setting A <- 0.5*(A + A^T)
   */
  template <class T, int M> 
  inline void SMat<T,M>::symmetrize() {
    for (int j=0;j<M;j++) for (int i=j+1;i<M;i++) {
      x[i+j*M] = x[j+i*M] = T(0.5)*(x[i+j*M] + x[j+i*M]);
    }
  }
  
  /** Returs symmetrized matrix 0.5*(A + A^T)
   */
  template <class T, int M> 
  inline SMat<T,M> SMat<T,M>::symmetrized() const {
    SMat<T,M> m;
    m.symmetrize(); 
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
    tmp.invert();
    return tmp;
  }

  /** Return the inverse of this matrix
   * This uses the LU decomposition
   */
  template <class T, int M> 
  inline int SMat<T,M>::invert() {
    SMat<T,M> tmp(*this);
    Vec<int,M> indx; T d;
    if (tmp.luDecomp(indx,&d)) {
      std::cerr << "SMat<" << M << ">::invert(): warning, "
		<< "can't take inverse of singular matrix." << std::endl << std::flush;
      return 1;
    }
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
    int i,imax,j,k; 
    T big,dum,sum,temp; 
    Vec<T,M> vv;

    if (d) *d=1.0; 
    for (i=0;i<M;i++) { 
      big=0.0; 
      for (j=0;j<M;j++) if ((temp=fabs(x[i+j*M])) > big) big=temp; 
      if (big == 0.0) return 1;
      vv(i)=T(1)/big; 
    }

    for (j=0;j<M;j++) { 
      for (i=0;i<j;i++) { 
        sum=x[i+j*M]; 
        for (k=0;k<i;k++) sum -= x[i+k*M]*x[k+j*M]; 
        x[i+j*M]=sum; 
      } 
      big=0.0; 

      for (i=j;i<M;i++) { 
        sum=x[i+j*M]; 
        for (k=0;k<j;k++) sum -= x[i+k*M]*x[k+j*M]; 
        x[i+j*M]=sum; 
        if ((dum=vv(i)*fabs(sum)) >= big) { 
          big=dum; 
          imax=i; 
        } 
      } 

      if (j != imax) { 
        for (k=0;k<M;k++) { 
          dum=x[imax+k*M]; 
          x[imax+k*M]=x[j+k*M]; 
          x[j+k*M]=dum; 
        }
        if (d) *d = -(*d); 
        vv(imax)=vv(j); 
      } 

      indx.x[j]=imax;
      if (x[j+j*M] == 0.0) x[j+j*M] = T(1e-20);
      if (j != M-1) { 
        dum=1.0/(x[j+j*M]); 
        for (i=j+1;i<M;i++) x[i+j*M] *= dum; 
      } 
    } 
    return 0;
  }

  /** Perform a LU Decomposition backsolve with given RHS.
   * Precondition: luDecomp(indx,d) has already been called
   * @param indx the row-swap vector returned from luDecomp()
   * @param b the RHS vector in the equation A*x = b
   * @return b is also the solution, overwriting the passed RHS
   */
  template <class T, int M> 
  inline void SMat<T,M>::luSolve(const Vec<int,M>& indx, Vec<T,M>& b) {
    int i,ii=-1,ip,j; 
    float sum;
    for (i=0;i<M;i++) { 
      ip=indx.x[i]; sum=b.x[ip]; b.x[ip]=b.x[i]; 
      if (ii>=0) for (j=ii;j<i;j++) sum -= x[i+j*M]*b.x[j]; 
      else if (sum) ii=i; 
      b.x[i]=sum; 
    } 
    for (i=(M-1);i>=0;i--) { 
      sum=b.x[i]; 
      for (j=i+1;j<M;j++) sum -= x[i+j*M]*b.x[j]; 
      b.x[i]=sum/x[i+i*M]; 
    } 
  }

  // Performs a Cholesky-Banachiewicz-style decomposition assuming
  // active_m active rows & columns in the matrix. This allows
  // expansion of the active set in an iterative fashion. 
  // Returns 1 (=failure) if the matrix is not positive definite,
  // 0 (=success) if the matrix is positive definite
  template <class T, int M> 
    inline int SMat<T,M>::choleskyDecomp(const int active_m) 
  {
    for (int ij=0;ij<active_m;ij++) {
      if(choleskyUpdate(ij)) return 1; 
    }
    return 0; 
  }

  // Banachiewicz style update
  // New input assumed to be added in the COLUMN i, 
  // Cholesky-factors will be computed in the ROW i. 
  // Returns 1 if not positive definite, 0 if positive definite
  template <class T, int M> 
    inline int SMat<T,M>::choleskyUpdate(const int ij) 
  {
    // First the elements in the row
    for (int j=0;j<ij;j++) {
      //DO ROW INPUT & OUTPUT x[ij+j*M] = x[j+ij*M]; // input in column, output in row
      for (int k=0;k<j;k++) x[ij+j*M] -= x[ij+k*M]*x[j+k*M];
      x[ij+j*M] /= x[j+j*M]; 
    }

    // Then the diagonal element, note that i == j here and since we're doing
    // this in-place l_ii == a_ii when starting out
    for (int k=0;k<ij;k++) x[ij+ij*M] -= x[ij+k*M]*x[ij+k*M];
    if (x[ij+ij*M] <= T(0)) return 1; // Not positive definite
    x[ij+ij*M] = sqrt(x[ij+ij*M]); 

    return 0; 
  }


  /** Perform a Cholesky Decomposition backsolve with given RHS.
  * Precondtion: choleskyDecomp() has already been called
  * @param b the RHS vector in the equation A*x = b
  * @return b is also the solution, overwriting the passed RHS
  */
  template <class T, int M> 
    inline void SMat<T,M>::choleskySolve(Vec<T,M>& b, Vec<T,M>& result, const int active_m) 
  {
    choleskySolve(b.x, result.x, active_m); 
  }

  template <class T, int M> template <int N>
    inline void SMat<T,M>::choleskySolve(Mat<T,M,N>& B, Mat<T,M,N>& Result, const int active_m, const int active_n)
  {
    for (int j=0;j<active_n;j++) {
      choleskySolve(&B.x[j*M], &Result.x[j*M], active_m); 
    }
  }


  /** Perform a Cholesky Decomposition backsolve with given RHS.
  * Precondtion: choleskyDecomp() has already been called
  * @param b the RHS vector in the equation A*x = b
  * @return b is also the solution, overwriting the passed RHS
  */
  template <class T, int M> 
    inline void SMat<T,M>::choleskySolve(T* b_x, T* result_x, const int active_m) 
  {
    T sum; int k;
    for (int i=0;i<active_m;i++) { 
      sum = b_x[i];
      for (k=i-1;k>=0;k--) sum -= x[i+k*M]*result_x[k];
      result_x[i] = T(sum/x[i+i*M]);
    }
    for (int i=active_m-1;i>=0;i--) { 
      sum = result_x[i];
      for (k=i+1;k<active_m;k++) sum -= x[k+i*M]*result_x[k];
      result_x[i] = T(sum/x[i+i*M]);
    }
  }
  /*template <class T, int M> 
    inline void SMat<T,M>::choleskySolve(Vec<T,M>& b) 
  {
    T sum; int k;
    for (int i=0;i<M;i++) { 
      for (sum=b.x[i],k=i-1;k>=0;k--) sum -= x[i+k*M]*b.x[k];
      b.x[i] = sum/x[i+i*M];
    }
    for (int i=M-1;i>=0;i--) { 
      for (sum=b.x[i],k=i+1;k<M;k++) sum -= x[k+i*M]*b.x[k];
      b.x[i] = sum/x[i+i*M];
    }
  }*/
} // end namespace sla

#endif
