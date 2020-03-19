//-*-c++-*-
#ifndef GBMAT_H
#define GBMAT_H

/** 
 * @file dmat.h
 * @brief Basic dynamic general banded matrix of type T, 
 *        using COLUMN-MAJOR ordering
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 21 March 2006 - Started 
 */


#include <array2.h>
#include <gemat.h>
#include <cpplapack.h>

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes 
 * plus some random utility functions and other containers
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int K> class Array; // K-dimensional array
  template <class T> class Array2; // 2-dimensional array
  template <class T> class GBMat; // general banded matrix type
  template <class T> class GEMat; // general matrix class

  //////////////////// GBMat ////////////////////
  /**
   * A General Banded Matrix class of type T
   */	
  template <class T> 
  class GBMat: public Array2<T> {
  public:
    // Constructors/Destructor
    GBMat();
    GBMat(const int m, const int n, const int kl, const int ku);

    // Mutators
    T& operator () (const int i1, const int i2);
    void setSize(const int m, const int n, const int kl, const int ku);
    
    // Accessors
    T operator () (const int i1, const int i2) const;
    int numOffDiags(const int lo_up) const { return msc(lo_up); }
    Vec2i size() const { return Vec2i(msc(2), dim(1)); }
    int size(const int k) const { return (k==0)?msc(2):dim(1); }

    // Helper functions
    int indexOf(const int i1, const int i2) const;
    
    // Printing routines
    void print(char* label = NULL);
    
    // LAPACK/CBLAS interface routines
    int mv(GEMat<T>& x_, GEMat<T>& y_, 
	   const T alpha = T(1), const T beta = T(0));
    
    // LAPACK/CBLAS wrappers
    void gbmv(int M, int N, int KL, int KU, T alpha, T *A, int lda, T *X, 
	      int incX, T beta, T *Y, int incY);
    
    // inherit member data and functions of parent
    using Array2<T>::x;
    using Array2<T>::reset;
    using Array2<T>::operator ();
    
  protected:
    // inherit member data and functions of parent
    using Array2<T>::dim;
    using Array2<T>::mul;
    using Array2<T>::len;
    Vec3i msc; // number of sub- and super-diagonals, plus number of rows
  };

  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// GBMat ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline GBMat<T>::GBMat() : Array2<T>(), msc(0) {}
  
  /** Ctor that starts with given dimensions
   * @param nn_ is the size
   */
  template <class T> 
  inline GBMat<T>::GBMat(const int m, const int n, 
			 const int kl, const int ku): Array2<T>() {
    setSize(m,n,kl,ku);
  }
  
  // Mutators
  
  /** Set the size of the array
   * @param n1, n2 the sizes of the array in each dimension
   */
  template <class T> 
  inline void GBMat<T>::setSize(const int m, const int n, 
				const int kl, const int ku) {
    msc(0) = kl; msc(1) = ku; msc(2) = m;
    Array2<T>::setSize(kl+ku+1,n);
  }      
  
  /** Returns mutable reference to array element
   * @param i1,i2 are the indices
   * @return a reference to the array element
   */
  template <class T> 
  inline T& GBMat<T>::operator () (const int i1, const int i2) {
    return x[indexOf(i1,i2)];
  }
  
  // Accessors

  /** Returns inmutable reference to array element
   * @param i1,i2 are the indices
   * @return the value of the array element
   */
  template <class T> 
  inline T GBMat<T>::operator () (const int i1, const int i2) const {
    return x[indexOf(i1,i2)];
  }

  // Helper functions (used internally only)
  
  /** Returns linear index of given array indices
   * @param are the indices to be converted
   * @return the linear index in the data array x
   */
  template <class T> 
  inline int GBMat<T>::indexOf(const int i1, const int i2) const {
#if AR_CHECK_BOUNDS
    if (i1 > min(size(0)-1,i2+numOffDiags(0)) ||
	i1 < max(0,i2-numOffDiags(1))) {
      std::cerr << "GBMat: Bounds error. (" << i1 << ", " << i2 
		<< ") invalid." << std::endl << std::flush;
      exit(1);	
    }
#endif
    return Array2<T>::indexOf(numOffDiags(1)+i1-i2,i2);
  }
  
  // Printing routines
  
  /** Write state to stream as formated ASCII 
   */
  template <class T> 
  inline void GBMat<T>::print(char* label) {
    int minFieldWidth = std::cout.precision()+5;
    T val;
    if (label) std::cout << label << " = [" << std::endl;
    else std::cout << "A = [" << std::endl;
    for (int i=0;i<size(0);i++) {
      for (int j=0;j<size(1);j++) {
	if (i <= min(size(0)-1,j+numOffDiags(0)) &&
	    i >= max(0,j-numOffDiags(1))) val = operator()(i,j);
	else val = T(0.0);
	std::cout << std::setw(minFieldWidth) << val << " ";
      }
      if (i<size(0)-1) std::cout << ";" << std::endl;
      else std::cout << "];" << std::endl;
    }
  }
  
  // LAPACK wrapper routines

  /** Internal wrapper for double - see mv() for details
   */
  template <> 
  inline void GBMat<double>::
  gbmv(int M, int N,int KL, int KU, double alpha, double *A, int lda, 
       double *X, int incX, double beta, double *Y, int incY) {
    cblas_dgbmv(CblasColMajor,CblasNoTrans,M,N,KL,KU,alpha,A,
		lda,X,incX,beta,Y,incY);
  }
  
  /** Internal wrapper for float - see mv() for details
   */
  template <> 
  inline void GBMat<float>::
  gbmv(int M, int N, int KL, int KU, float alpha, float *A, int lda, float *X,
       int incX, float beta, float *Y, int incY) {
    cblas_sgbmv(CblasColMajor,CblasNoTrans,M,N,KL,KU,alpha,A,
		lda,X,incX,beta,Y,incY);
  }
  
  /** Perform Solution by Cholesky factorization
   * @param b is the RHS and the solution vector
   */
  template <class T> 
  inline int GBMat<T>::mv(GEMat<T>& x_, GEMat<T>& y_, 
			  const T alpha, const T beta) {
    // define inputs
    int M = size(0);
    int N = size(1);
    int KL = numOffDiags(0);
    int KU = numOffDiags(1);
    int lda = size(0);
    int incX = 1;
    int incY = 1;
    
    // call cblas routine
    gbmv(M,N,KL,KU,alpha,x,lda,x_.x,incX,beta,y_.x,incY);
    
    // always return 0, if success
    return 0;
  }
} // end namespace sla

#endif
