//-*-c++-*-
#ifndef SBMAT_H
#define SBMAT_H

/** 
 * @file sbmat.h
 * @brief symmetric banded matrix class using column-wise ordering
 *        
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
  template <class T> class SBMat; // symmetric banded matrix type
  template <class T> class GEMat; // general matrix class

  //////////////////// SBMat ////////////////////
  /**
   * A Symmetric Banded Matrix class of type T
   */	
  template <class T> 
  class SBMat: public Array2<T> {
  public:
    // Constructors/Destructor
    SBMat();
    SBMat(const int n, const int kd);

    // Mutators
    T& operator () (const int i1, const int i2);
    void setSize(const int n, const int kd);
    void set(const SBMat<T>& m);
    
    // Accessors
    T operator () (const int i1, const int i2) const;
    int numOffDiags() const { return dim(0)-1; }
    int size() const { return dim(1); }

    // Helper functions
    int indexOf(int i1, int i2) const;
    
    // Printing routines
    void print(char* label = NULL, std::ostream& os = std::cout);

    // LAPACK interface routines
    int sv(GEMat<T>& b);
    int ev(GEMat<T>& d, GEMat<T>& v);
    int ev(GEMat<T>& d);
    
    // LAPACK wrappers
    void pbsv(char* uplo, integer* n, integer* kd, integer* 
	      nrhs, T* ab, integer* ldab, T* b, integer* ldb, 
	      integer* info);
    void sbev(char* jobz, char* uplo, integer* n, integer* kd, 
	      T* ab, integer* ldab, T* w, T* z__, 
	      integer* ldz, T* work, integer* info);
    
    // inherit member data and functions of parent
    using Array2<T>::x;
    using Array2<T>::reset;
    using Array2<T>::operator ();
    using Array2<T>::size;
    using Array2<T>::set;
    
  protected:
    // inherit member data and functions of parent
    using Array2<T>::dim;
  };

  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// SBMat ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline SBMat<T>::SBMat() : Array2<T>() {}
  
  /** Ctor that starts with given dimensions
   * @param nn_ is the size
   */
  template <class T> 
  inline SBMat<T>::SBMat(const int n, const int kd) : Array2<T>(kd+1,n) {}
  
  // Mutators
  
  /** Set the size of the array
   * @param n1, n2 the sizes of the array in each dimension
   */
  template <class T> 
  inline void SBMat<T>::setSize(const int n, const int kd) {
    Array2<T>::setSize(kd+1,n);
  }      
  
  /** Returns mutable reference to array element
   * @param i1,i2 are the indices
   * @return a reference to the array element
   */
  template <class T> 
  inline T& SBMat<T>::operator () (const int i1, const int i2) {
    return x[indexOf(i1,i2)];
  }
  
  /** Set this matrix to the passed matrix, value-wise
   * @param m is the array to duplicate
   */
  template <class T> 
  inline void SBMat<T>::set(const SBMat<T>& m) {
    setSize(m.size(),m.numOffDiags());
    Array2<T>::set(m.x);
  }

  // Accessors

  /** Returns inmutable reference to array element
   * @param i1,i2 are the indices
   * @return the value of the array element
   */
  template <class T> 
  inline T SBMat<T>::operator () (const int i1, const int i2) const {
    return x[indexOf(i1,i2)];
  }

  // Helper functions (used internally only)
  
  /** Returns linear index of given array indices
   * @param are the indices to be converted
   * @return the linear index in the data array x
   */
  template <class T> 
  inline int SBMat<T>::indexOf(int i1, int i2) const {
#if AR_CHECK_BOUNDS
    if (i1 < i2) swap(i1,i2);
    if (i1 > sla::min(size()-1,i2+numOffDiags())) {
      std::cerr << "Size = " << size() << std::endl
		<< "NumOffDiags = " << numOffDiags() << std::endl;
      std::cerr << "SBMat: Bounds error. (" << i1 << ", " << i2 
		<< ") invalid." << std::endl << std::flush;
      exit(1);
    }
#endif
    return Array2<T>::indexOf(i1-i2,i2);
  }

  // Printing routines
  
  /** Write state to stream as formated ASCII 
   */
  template <class T> 
  inline void SBMat<T>::print(char* label, std::ostream& os) {
    int minFieldWidth = os.precision()+2;
    T val;
    if (label) os << label << " = [" << std::endl;
    else os << "A = [" << std::endl;
    for (int i=0;i<size();i++) {
      for (int j=0;j<size();j++) {
	if (fabs(i-j) < 1+numOffDiags()) val = operator()(i,j);
	else val = T(0.0);
	os << std::setw(minFieldWidth) << val << " ";
      }
      if (i<size()-1) os << ";" << std::endl;
      else os << "];" << std::endl;
    }
  }

  // LAPACK wrapper routines

  /** Internal wrapper for double - see ev() for details
   */
  template <> inline 
  void SBMat<double>::sbev(char* jobz, char* uplo, integer* n, integer* kd, 
			   double* ab, integer* ldab, double* w, double* z__, 
			   integer* ldz, double* work, integer* info) {
    dsbev_(jobz,uplo,n,kd,ab,ldab,w,z__,ldz,work,info);
  }
  
  /** Internal wrapper for float - see ev() for details
   */
  template <> inline 
  void SBMat<float>::sbev(char* jobz, char* uplo, integer* n, integer* kd, 
			  float* ab, integer* ldab, float* w, float* z__, 
			  integer* ldz, float* work, integer* info) {
    ssbev_(jobz,uplo,n,kd,ab,ldab,w,z__,ldz,work,info);
  }


  /** Internal wrapper for double - see sv() for details
   */
  template <> 
  inline void SBMat<double>::pbsv(char* uplo, integer* n, integer* kd, 
				  integer* nrhs, double* ab, integer* ldab, 
				  double* b, integer* ldb, integer* info) {
    dpbsv_(uplo,n,kd,nrhs,ab,ldab,b,ldb,info);
  }
  
  /** Internal wrapper for float - see sv() for details
   */
  template <> 
  inline void SBMat<float>::pbsv(char* uplo, integer* n, integer* kd, 
				 integer* nrhs, float* ab, integer* ldab, 
				 float* b, integer* ldb, integer* info) {
    spbsv_(uplo,n,kd,nrhs,ab,ldab,b,ldb,info);
  }
  
  /** Perform Eigenvalue-eigenvector decomposition
   * @return d returns the eigenvalues
   * @return v returns the eigenvectors (columnwise)
   */
  template <class T> 
  inline int SBMat<T>::ev(GEMat<T>& d, GEMat<T>& v) {
    if (AR_PO>2) std::cout << "Running Eigen-decomposition... " 
			   << std::endl << std::flush;
    char jobz = 'V';
    char uplo = 'L';
    integer n = integer(size());
    integer kd = integer(numOffDiags());
    integer ldab = kd+1;
    if (d.size(0) < n) d.setSize(n,1);
    if (v.size(0) < n || v.size(1) < n) d.setSize(n,n);
    integer ldz = integer(v.size(0));
    GEMat<T> work(sla::max(1,3*int(n)-2),1);
    integer info;
    
    // run cholesky factor/solve
    sbev(&jobz,&uplo,&n,&kd,x,&ldab,d.x,v.x,&ldz,work.x,&info);
    if (AR_PO>2) std::cout << "Done." << std::endl << std::flush;
    return int(info);  
  }

  /** Perform Eigenvalue-eigenvector decomposition
   * @return d returns the eigenvalues
   * @return v returns the eigenvectors (columnwise)
   */
  template <class T> 
  inline int SBMat<T>::ev(GEMat<T>& d) {
    if (AR_PO>2) std::cout << "Running Eigen-decomposition... " 
			   << std::endl << std::flush;
    char jobz = 'N';
    char uplo = 'L';
    integer n = integer(size());
    integer kd = integer(numOffDiags());
    integer ldab = kd+1;
    if (d.size(0) < n) d.setSize(n,1);
    T v;
    integer ldz = 1;
    GEMat<T> work(sla::max(1,3*int(n)-2),1);
    integer info;
    
    // run cholesky factor/solve
    sbev(&jobz,&uplo,&n,&kd,x,&ldab,d.x,&v,&ldz,work.x,&info);
    if (AR_PO>2) std::cout << "Done." << std::endl << std::flush;
    return int(info);  
  }

  /** Perform Solution by Cholesky factorization
   * @param b is the RHS and the solution vector
   */
  template <class T> 
  inline int SBMat<T>::sv(GEMat<T>& b) {
    if (AR_PO>2) std::cout << "Running Cholesky/solve... " 
			   << std::endl << std::flush;
    char uplo = 'L';
    integer n = integer(size());
    integer kd = integer(numOffDiags());
    integer nrhs = integer(b.size(1));
    integer ldab = kd+1;
    integer ldb = integer(b.size(0));
    integer info;
    
    // run cholesky factor/solve
    pbsv(&uplo,&n,&kd,&nrhs,x,&ldab,b.x,&ldb,&info);
    if (AR_PO>2) std::cout << "Done." << std::endl << std::flush;
    return int(info);  
  }
} // end namespace sla

#endif
