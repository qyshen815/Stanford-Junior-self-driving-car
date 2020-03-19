//-*-c++-*-
#ifndef GEMAT_H
#define GEMAT_H

/** 
 * @file dmat.h
 * @brief Basic dynamic general matrix of type T, using COLUMN-MAJOR ordering
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 21 March 2006 - Started 
 */


#include <array2.h>
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
  template <class T> class GEMat; // general matrix class

  //////////////////// GEMat ////////////////////
  /**
   * A General matrix class of type T
   */	
  template <class T> 
  class GEMat: public Array2<T> {
  public:
    // Constructors/Destructor
    GEMat();
    GEMat(const int n1, const int n2);

    // Printing routines
    void print(char* label = NULL, std::ostream& os = std::cout);

    // Utility routines
    T mn();

    // LAPACK/CBLAS interface routines
    int mm(GEMat<T>& b, GEMat<T>& c, 
	   bool transA = false, bool TransB = false,
	   T alpha = T(1), T beta = T(0));
    int svd(GEMat<T>& s, GEMat<T>& u, GEMat<T>& vt,
	       char jobu = 'S', char jobvt = 'S');
    int lsd(GEMat<T>& b, GEMat<T>& s, int& rank, T rcond = T(1e-8));
    int sv(GEMat<T>& b);

    // LAPACK/CBLAS wrappers
    void gesv(integer *n, integer *nrhs, T *a, integer 
	      *lda, integer *ipiv, T *b, integer *ldb, integer *info);
    void gemm(const bool TransA, const bool TransB, const int M, const int N,
	      const int K, const T alpha, const T *A, const int lda, 
	      const T *B, const int ldb, const T beta, T *C, const int ldc);
    void gesvd(char* jobu, char* jobvt, integer* m, integer* n, T* a, 
	       integer* lda, T* s, T* u, integer* ldu, T* vt, integer* ldvt, 
	       T *work, integer *lwork, integer *info);
    void gelsd(integer* m, integer* n, integer* nrhs, T* a, integer* lda, 
	       T* b, integer* ldb, T* s, T* rcond, integer* rank, T* work, 
	       integer* lwork, integer* iwork, integer* info);
    
    // inherit member data and functions of parent
    using Array2<T>::x;
    using Array2<T>::reset;
    using Array2<T>::operator ();
    using Array2<T>::size;
    using Array2<T>::set;
    using Array2<T>::setSize;
    
  protected:
    // inherit member data and functions of parent
    using Array2<T>::dim;
  };

    
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// GEMat ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline GEMat<T>::GEMat() : Array2<T>() {}
  
  /** Ctor that starts with given dimensions
   * @param nn_ is the size
   */
  template <class T> 
  inline GEMat<T>::GEMat(const int n1, const int n2) : Array2<T>(n1,n2) {}

  // Printing routines
  
  /** Write state to stream as formated ASCII 
   */
  template <class T> 
  inline void GEMat<T>::print(char* label, std::ostream& os) {
    int minFieldWidth = os.precision()+5;  
    if (label) os << label << " = [" << std::endl;
    else os << "A = [" << std::endl;
    for (int i=0;i<size(0);i++) {
      for (int j=0;j<size(1);j++) {
	os << std::setw(minFieldWidth) << operator()(i,j) << " ";
      }
      if (i<size(0)-1) os << ";" << std::endl;
      else os << "];" << std::endl;
    }
  }

  // Utility routines
  
  template <class T> 
  inline T GEMat<T>::mn() {
    T mn_val = x[0];
    for (int i=1;i<dim.prod();i++) if (x[i] < mn_val) mn_val = x[i];
    return mn_val;
  }

  // LAPACK wrapper routines

  /** Internal wrapper for double - see svd() for details
   */
  template <> 
  inline void GEMat<double>::gesvd(char* jobu, char* jobvt, integer* m, 
				  integer* n, double* a, integer* lda, 
				  double* s, double* u, integer* ldu, 
				  double* vt, integer* ldvt, double *work, 
				  integer *lwork, integer *info) {
    dgesvd_(jobu,jobvt,m,n,a,lda,s,u,ldu,vt,ldvt,work,lwork,info);
  }

  /** Internal wrapper for float - see svd() for details
   */
  template <> 
  inline void GEMat<float>::gesvd(char* jobu, char* jobvt, integer* m, 
				 integer* n, float* a, integer* lda, 
				 float* s, float* u, integer* ldu, float* vt, 
				 integer* ldvt, float *work, 
				 integer *lwork, integer *info) {
    sgesvd_(jobu,jobvt,m,n,a,lda,s,u,ldu,vt,ldvt,work,lwork,info);
  }
  
  /** Internal wrapper for double - see lsd() for details
   */
  template <> 
  inline void GEMat<double>::gelsd(integer* m, integer* n, integer* nrhs, 
				  double* a, integer* lda, double* b, 
				  integer* ldb, double* s, double* rcond, 
				  integer* rank, double* work, integer* lwork, 
				  integer* iwork, integer* info) {
    dgelsd_(m,n,nrhs,a,lda,b,ldb,s,rcond,rank,work,lwork,iwork,info);
  }
  
  /** Internal wrapper for float - see lsd() for details
   */
  template <> 
  inline void GEMat<float>::gelsd(integer* m, integer* n, integer* nrhs, 
				 float* a, integer* lda, float* b, 
				 integer* ldb, float* s, float* rcond, 
				 integer* rank, float* work, integer* lwork, 
				 integer* iwork, integer* info) {
    sgelsd_(m,n,nrhs,a,lda,b,ldb,s,rcond,rank,work,lwork,iwork,info);
  }


  /** Internal wrapper for double - see mm() for details
   */
  template <> 
  inline void GEMat<double>::gemm(const bool TransA, const bool TransB, 
				  const int M, const int N, const int K, 
				  const double alpha, const double *A, 
				  const int lda, const double *B, 
				  const int ldb, const double beta, 
				  double *C, const int ldc) {
    if (!TransA && !TransB)
      cblas_dgemm(CblasColMajor,CblasNoTrans,CblasNoTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (!TransA && TransB) 
      cblas_dgemm(CblasColMajor,CblasNoTrans,CblasTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (TransA && !TransB) 
      cblas_dgemm(CblasColMajor,CblasTrans,CblasNoTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (TransA && TransB) 
      cblas_dgemm(CblasColMajor,CblasTrans,CblasTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
  }

  /** Internal wrapper for float - see mm() for details
   */
  template <> 
  inline void GEMat<float>::gemm(const bool TransA, const bool TransB, 
				 const int M, const int N, const int K, 
				 const float alpha, const float *A, 
				 const int lda, const float *B, const int ldb, 
				 const float beta, float *C, const int ldc) {
    if (!TransA && !TransB)
      cblas_sgemm(CblasColMajor,CblasNoTrans,CblasNoTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (!TransA && TransB) 
      cblas_sgemm(CblasColMajor,CblasNoTrans,CblasTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (TransA && !TransB) 
      cblas_sgemm(CblasColMajor,CblasTrans,CblasNoTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
    if (TransA && TransB) 
      cblas_sgemm(CblasColMajor,CblasTrans,CblasTrans,M,N,K,
		  alpha,A,lda,B,ldb,beta,C,ldc);
  }
  
  /** Internal wrapper for double - see sv() for details
   */
  template <> 
  inline void GEMat<double>::gesv(integer *n, integer *nrhs, double *a, 
				  integer *lda, integer *ipiv, double *b, 
				  integer *ldb, integer *info) {
    dgesv_(n,nrhs,a,lda,ipiv,b,ldb,info);
  }
  
  /** Internal wrapper for float - see sv() for details
   */
  template <> 
  inline void GEMat<float>::gesv(integer *n, integer *nrhs, float *a, 
				 integer *lda, integer *ipiv, float *b, 
				 integer *ldb, integer *info) {
    sgesv_(n,nrhs,a,lda,ipiv,b,ldb,info);    
  }

  /** Solves general system of linear equations AX = B.
   *  @param B is the righthand side and the solution
   */
  template <class T> 
  inline int GEMat<T>::sv(GEMat<T>& b) {
    // define inputs
    integer N = integer(size(0));
    integer NRHS = integer(b.size(1));
    integer LDA = integer(size(0));
    Array2<integer> IPIV(int(N),1);
    integer LDB = integer(b.size(0));
    integer INFO;

    // call LAPACK wrapper
    gesv(&N,&NRHS,x,&LDA,IPIV.x,b.x,&LDB,&INFO);

    return int(INFO);
  }

  /** Performs a matrix product.  If this matrix is A, then this performs
   *  C = alpha*A*B + beta*C with an optional transpose on both A and B.
   * @param c is the return matrix
   * @param b is the matrix against which we multiply
   */
  template <class T> 
  inline int GEMat<T>::mm(GEMat<T>& b, GEMat<T>& c, 
			  const bool transA, const bool transB,
			  const T alpha, const T beta) {
    // define inputs
    int M = transA?size(1):size(0);
    int N = transB?b.size(0):b.size(1);
    int K = transA?size(0):size(1);
    int lda = size(0);
    int ldb = b.size(0);
    int ldc = M;
    c.setSize(M,N);
    
    // call cblas wrapper
    gemm(transA,transB,M,N,K,alpha,x,lda,b.x,ldb,beta,c.x,ldc);
    
    return 0;
  }
  
  /** Perform singular value decomposition on this matrix a = u*diag(s)*vt
   * @param s is the vector of singular values
   * @param u is the matrix of left singular vectors
   * @param vt is the matrix of right singular vectors
   * @param jobu is 'A', 'S', 'O', or 'N', to define output in u
   * @param jobvt is 'A', 'S', 'O', or 'N', to define output in vt
   */
  template <class T> 
  inline int GEMat<T>::svd(GEMat<T>& s, GEMat<T>& u, GEMat<T>& vt,
			  char jobu, char jobvt) {
    if (AR_PO>2) std::cout << "Running SVD... " << std::endl << std::flush;
    
    int fp_type = -1;
    if (sizeof(T) == sizeof(float)) fp_type = 0;
    if (sizeof(T) == sizeof(double)) fp_type = 1;
    if (fp_type == -1) {
      std::cerr << "Error, don't recognize type." << std::endl << std::flush;
      exit(1);
    }

    integer ma = integer(size(0));
    integer na = integer(size(1));
    integer ms = integer(sla::min<int>(int(ma),int(na)));
    s.setSize(ms,1);
    
    integer mu, nu;
    switch (jobu) {
    case 'A': mu = ma; nu = ma; break;
    case 'S': mu = ma; nu = integer(sla::min<int>(int(ma),int(na))); break;
    case 'O': case 'N': mu = 1; nu = 1; break;
    default: 
      cerr << "Invalid flag jobu = " << jobu << endl;
      exit(1);
    }
    if (mu > 1 && nu > 1) u.setSize(int(mu),int(nu));
    
    integer mvt, nvt;
    switch (jobvt) {
    case 'A': nvt = na; mvt = na; break;
    case 'S': nvt = na; mvt = integer(sla::min<int>(int(ma),int(na))); break;
    case 'O': case 'N': nvt = 1; mvt = 1; break;
    default: 
      cerr << "Invalid flag jobvt = " << jobu << endl;
      exit(1);
    }
    if (mvt > 1 && nvt > 1) vt.setSize(int(mvt),int(nvt));
    
    integer info;
    integer lwork;
    GEMat<T> work(1,1);
    
    // query optimal work size
    lwork = -1;
    gesvd(&jobu, &jobvt, &ma, &na, x, &ma, s.x, u.x, &mu, 
	  vt.x, &mvt, work.x, &lwork, &info);
    if (info != 0) {
      std::cerr << "GEMat<T>::svd: Can't compute optimal work space size."
		<< std::endl << std::flush;
      exit(1);
    }
    lwork = integer(work(0));
    work.setSize(lwork,1);
    
    // run actual SVD
    gesvd(&jobu, &jobvt, &ma, &na, x, &ma, s.x, u.x, &mu, 
	  vt.x, &mvt, work.x, &lwork, &info);

    if (AR_PO>2) std::cout << "Done." << std::endl << std::flush;
    return int(info);
  }
  
  /** Perform linear least squares by SVD: min |b - a*x|
   * @param b is the RHS and the solution vector
   * @param s is the vector of singular values
   * @param rank is the rank of the matrix a
   * @param rcond is the threshold for considering a singular value zero
   */
  template <class T> 
  inline int GEMat<T>::lsd(GEMat<T>& b, GEMat<T>& s, int& rank, T rcond) {
    if (AR_PO>2) std::cout << "Running LLS by SVD... " 
			   << std::endl << std::flush;

    integer ma = integer(size(0));
    integer na = integer(size(1));
    integer ms = integer(std::min(int(ma),int(na)));
    s.setSize(ms,1);
        
    integer ldb = integer(b.size(0));
    integer nb = integer(b.size(1));

    if (ldb < ma) {
      std::cerr << "problem, ldb < ma." << std::endl << std::flush;
      exit(1);
    }
    
    integer info;
    integer lwork;
    integer rnk;
    GEMat<T> work(1,1);
    GEMat<integer> iwork(1,1);
    
    // query optimal work size
    lwork = -1;
    gelsd(&ma, &na, &nb, x, &ma, b.x, &ldb, s.x, &rcond, &rnk, 
	    work.x, &lwork, iwork.x, &info);
    if (info != 0) {
      std::cerr << "GEMat<T>::lsd: Can't compute optimal work space size."
		<< std::endl << std::flush;
      exit(1);
    }
    lwork = integer(work(0));
    work.setSize(lwork,1);
    iwork.setSize(lwork,1);
    
    // run actual SVD
    gelsd(&ma, &na, &nb, x, &ma, b.x, &ldb, s.x, &rcond, &rnk, 
	    work.x, &lwork, iwork.x, &info);

    rank = int(rnk);
    if (AR_PO>2) std::cout << "Done." << std::endl << std::flush;
    return int(info);
  }
} // end namespace sla

#endif
