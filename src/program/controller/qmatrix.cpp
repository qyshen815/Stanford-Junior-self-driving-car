#include "qmatrix.h"
#include <execinfo.h>
#include <sstream>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <ctype.h>
#include <unistd.h>

extern "C" {
  int dgesvd_(char *jobu, char *jobvt, int *m, int *n, double *a, int *lda,
	      double *s, double *u, int *ldu, double *vt, int *ldvt,
	      double *work, int *lwork, int *info);

  int dgemm_(const char *transa, const char *transb, int *m, int *n, int *k,
	     double *alpha, double *a, int *lda, double *b, int *ldb,
	     double *beta, double *c, int *ldc);

  double ddot_(int *n, double *dx, int *incx, double *dy, int *incy);

  double dnrm2_(int *n, double *x, int *incx);

  int dgesv_(const int *n, const int *nrhs, double *A, const int *lda,
	     int *ipiv, double *B, const int *ldb, int *info);
  int dger_(const int *m, const int *n, const double *alpha, const double *x,
	    const int *incx, const double *y, const int *incy, double *A,
	    const int *lda);
}


// constructor for empty matrix
qmatrix::qmatrix()
{
  m = 1;
  n = 1;
  d = new double [m*n];
  d[0] = 0;
}
  
// constructor for matrix
qmatrix::qmatrix(int m, int n)
{
  this->m = m;
  this->n = n;
  this->d = new double [m*n];
}

// constructor for vector
qmatrix::qmatrix(int m)
{
  this->m = m;
  n = 1;
  d = new double [m];
}

// constructor with default value
qmatrix::qmatrix(int m, int n, double x)
{
  this->m = m;
  this->n = n;
  d = new double[m*n];
  (*this) = x;
}

// constructor with default value
qmatrix::qmatrix(int m, double x)
{
  this->m = m;
  n = 1;
  d = new double [m];
  (*this) = x;
}

// constructor with default values
qmatrix::qmatrix(int m, int n, const char *vals)
{
  this->m = m;
  this->n = n;
  d = new double[m*n];
  this->read(vals);
}

qmatrix::qmatrix(int m, const char* vals)
{
  this->m = m;
  n = 1;
  d = new double[m*n];
  this->read(vals);
}

qmatrix::qmatrix(int m, int n, const double *vals)
{
  this->m = m;
  this->n = n;
  d = new double[m*n];
  *this = vals;
}


// copy constructor
qmatrix::qmatrix(const qmatrix& b)
{
  m = b.m;
  n = b.n;
  d = new double [m*n];
  memcpy(d, b.d, m*n*sizeof(double));
}

// constructor from file
qmatrix::qmatrix(const char* filename)
{
  m = n = 1;
  d = new double [1];
  d[0] = 1;
  load_from_file(filename);
}

// destructor
qmatrix::~qmatrix()
{
  delete [] d;
  d = 0;
}


// () operators
double& qmatrix::operator()(int i, int j)
{
  if (i < 0 || i > m-1 || j < 0 || j > n-1)
    throw qmat_exception("operator(): index out of bounds");
  return d[j*m + i];
}
  
double qmatrix::operator()(int i, int j) const
{
  if (i < 0 || i > m-1 || j < 0 || j > n-1)
    throw qmat_exception("operator(): index out of bounds");
  return d[j*m + i];
}

double& qmatrix::operator()(int i) 
{
  return (*this)(i,0);
}
  
double qmatrix::operator()(int i) const
{
  return (*this)(i,0);
}

  // = operators
qmatrix& qmatrix::operator=(const qmatrix& b)
{
  if (this == &b) return *this;

  if (d != 0) delete [] d;
  m = b.m;
  n = b.n;
  d = new double[m * n];
  memcpy(d, b.d, m*n*sizeof(double));
  return *this;
}

qmatrix& qmatrix::operator=(double x)
{
  for (int i = 0; i < m*n; i++) d[i] = x;
  return *this;
}

qmatrix& qmatrix::operator=(const double *x)
{
  for (int i = 0; i < m*n; i++) d[i] = x[i];
  return *this;
}
  
  // += operators
qmatrix& qmatrix::operator+=(const qmatrix& b)
{
  if (m != b.m || n != b.n)
    throw qmat_exception("operator+=: matrices are different sizes");
  for (int i = 0; i < n*m; i++) d[i] += b.d[i];
  return *this;
}

qmatrix& qmatrix::operator+=(double x)
{
  for (int i = 0; i < n*m; i++) d[i] += x;
  return *this;
}

// -= operators
qmatrix& qmatrix::operator-=(const qmatrix& b)
{
  if (m != b.m || n != b.n)
    throw qmat_exception("operator-=: matrices are different sizes");
  for (int i = 0; i < n*m; i++) d[i] -= b.d[i];
  return *this;
}

qmatrix& qmatrix::operator-=(double x)
{
  for (int i = 0; i < n*m; i++) d[i] -= x;
  return *this;
}

// *= operators (element-wise multiplication)
qmatrix& qmatrix::operator*=(const qmatrix& b)
{
  if (m != b.m || n != b.n)
    throw qmat_exception("operator*=: matrices are different sizes");
  for (int i = 0; i < n*m; i++) d[i] *= b.d[i];
  return *this;
}

qmatrix& qmatrix::operator*=(double x)
{
  for (int i = 0; i < n*m; i++) d[i] *= x;
  return *this;
}

// /= operators (element-wise division)
qmatrix& qmatrix::operator/=(const qmatrix& b)
{
  if (m != b.m || n != b.n)
    throw qmat_exception("operator/=: matrices are different sizes");
  for (int i = 0; i < n*m; i++) d[i] /= b.d[i];
  return *this;
}

qmatrix& qmatrix::operator/=(double x)
{
  for (int i = 0; i < n*m; i++) d[i] /= x;
  return *this;
}

// + operator
qmatrix qmatrix::operator+(const qmatrix& b) const
{
  return qmatrix(*this) += b;
}
qmatrix qmatrix::operator+(double x) const
{
  return qmatrix(*this) += x;
}

// - operator
qmatrix qmatrix::operator-(const qmatrix& b) const
{
  return qmatrix(*this) -= b;
}

qmatrix qmatrix::operator-(double x) const
{
  return qmatrix(*this) -= x;
}

// * operator
qmatrix qmatrix::operator*(const qmatrix &b) const
{
  if (n != b.m)
    throw qmat_exception("operator*: inner dimension mismatch");
  
  qmatrix a(m, b.n);
  double one = 1.0, zero = 0.0;
  int n = this->n, m = this->m, bm = b.m, bn = b.n;
  char cn = 'n';
  dgemm_(&cn, &cn, &m, &bn, &n, &one, d, &m, b.d, &bm, &zero, a.d, &m);
  
  return a;
}

qmatrix qmatrix::operator*(double x) const
{
  return qmatrix(*this) *= x;
}


// submatrices
qmatrix qmatrix::submatrix(int rl, int ru, int cl, int cu) const
{
  if (ru < rl || cu < cl)
    throw qmat_exception("submatrix: invalid indices");
  
  qmatrix sub(ru-rl+1,cu-cl+1);
  for (int i = 0; i <= ru-rl; i++) {
    for (int j = 0; j <= cu-cl; j++) {
      sub.d[j*sub.m+i] = d[(j+cl)*m + i+rl];
    }
  }
  return sub;
}

// get rows
qmatrix qmatrix::rows(int rl, int ru) const
{
  return submatrix(rl, ru, 0, size2()-1);
}

  // get columns
qmatrix qmatrix::cols(int cl, int cu) const
{
  return submatrix(0, size1()-1, cl, cu);
}

// get a single row
qmatrix qmatrix::row(int r) const
{
  return rows(r,r);
}


// get a single column
qmatrix qmatrix::col(int c) const
{
  return cols(c,c);
}


// get submatrix with index array
qmatrix qmatrix::submatrix(int *ri, int ri_count, int *ci, int ci_count) const
{
  qmatrix a(ri_count, ci_count);

  for (int i = 0; i < ci_count; i++) {
    for (int j = 0; j < ri_count; j++) {
      a.d[i*ri_count + j] = d[ci[i]*m + ri[j]];
    }
  }
  return a;
}


// get rows with index array  
qmatrix qmatrix::rows(int *ri, int ri_count) const
{
  qmatrix a(ri_count, n);

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < ri_count; j++) {
      a.d[i*ri_count + j] = d[i*m + ri[j]];
    }
  }
  return a;
}

// get cols with index array
qmatrix qmatrix::cols(int *ci, int ci_count) const
{
  qmatrix a(m, ci_count);

  for (int i = 0; i < ci_count; i++) {
    for (int j = 0; j < m; j++) {
      a.d[i*m + j] = d[ci[i]*m + j];
    }
  }
  return a;
}


// set a submatrix
void qmatrix::set_submatrix(int rl, int ru, int cl, int cu, const qmatrix &b)
{
  if (ru < rl || cu < cl)
    throw qmat_exception("set_submatrix: invalid indices");
  if (b.size1() != ru-rl+1 || b.size2() != cu-cl+1)
    throw qmat_exception("set_submatrix: submatrix is incorrect size");
  
  for (int i = 0; i <= ru-rl; i++) {
    for (int j = 0; j <= cu-cl; j++) {
      d[i+rl + m*(j+cl)] = b.d[j*b.m+i];
    }
  }
}

// set rows
void qmatrix::set_rows(int rl, int ru, const qmatrix &b)
{
  set_submatrix(rl, ru, 0, size2()-1, b);
}

// set columns
void qmatrix::set_cols(int cl, int cu, const qmatrix &b)
{
  set_submatrix(0, size1()-1, cl, cu, b);
}

// set row
void qmatrix::set_row(int r, const qmatrix &b)
{
  set_rows(r, r, b);
}

// set column
void qmatrix::set_col(int c, const qmatrix &b)
{
  set_cols(c, c, b);
}

// set submatrix defined by array of indices
void qmatrix::set_submatrix(int *ri, int ri_count, int *ci, int ci_count,
			       const qmatrix &b)
{
  for (int i = 0; i < ci_count; i++) {
    for (int j = 0; j < ri_count; j++) {
      d[ci[i]*m + ri[j]] = b.d[i*ri_count + j];
    }
  }
}

// set rows using array of indices
void qmatrix::set_rows(int *ri, int ri_count, const qmatrix &b)
{
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < ri_count; j++) {
      d[i*m + ri[j]] = b.d[i*ri_count + j];
    }
  }

}

// set cols using array of indices
void qmatrix::set_cols(int *ci, int ci_count, const qmatrix &b)
{
  for (int i = 0; i < ci_count; i++) {
    for (int j = 0; j < m; j++) {
      d[ci[i]*m + j] = b.d[i*m + j];
    }
  }
}


// swap rows
void qmatrix::swap_rows(int r1, int r2)
{
  qmatrix temp(1,n);
  temp = this->row(r1);
  this->set_row(r1, this->row(r2));
  this->set_row(r2, temp);
}

// swap columns
void qmatrix::swap_cols(int c1, int c2)
{
  qmatrix temp(m);
  temp = this->col(c1);
  this->set_col(c1, this->col(c2));
  this->set_col(c2, temp);
}


// resize matrix
void qmatrix::resize(int m, int n)
{
  double *oldd = d;
  d = new double [m*n];
  
  for (int i = 0; i < MIN(m, this->m); i++) {
    for (int j = 0; j < MIN(n, this->n); j++) {
      d[j*m+i] = oldd[j*this->m + i];
    }
  }
  this->m = m;
  this->n = n;
  delete [] oldd;
}

// resize, setting all added elements to zero
void qmatrix::resize_zero(int m, int n)
{
  int oldm = this->m, oldn = this->n;
  resize(m,n);
  for(int i = 0; i < oldn; i++) {
    for (int j = oldm; j < m; j++) {
      d[i*m+j] = 0.0;
    }
  }
  for (int i = oldn; i < n; i++) {
    for (int j = 0; j < m; j++) {
      d[i*m+j] = 0.0;
    }
  }
}


// reshape the matrix
void qmatrix::reshape(int m, int n)
{
  if (m*n != this->m*this->n)
    throw qmat_exception("reshape: dimensions are not compatible");
  this->m = m;
  this->n = n;
}

// make a long vector out of the matrix
qmatrix qmatrix::vec()
{
  qmatrix v(*this);
  v.reshape(m*n,1);
  return v;
}


// repeat the matrix m by n times
qmatrix qmatrix::repmat(int m, int n) const
{
  qmatrix a(m*this->m, n*this->n);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      a.set_submatrix(i*this->m, (i+1)*this->m-1, j*this->n, (j+1)*this->n-1,
                     *this);
    }
  }
  return a;
}
  

// multiply by b.t()
qmatrix qmatrix::mul_t(const qmatrix &b) const
{
  qmatrix a(m, b.m);
  double one = 1.0, zero = 0.0;
  int n = this->n, m = this->m, bm = b.m;
  dgemm_("n", "t", &m, &bm, &n, &one, d, &m, b.d, &bm, &zero, a.d, &m);

  return a;
}

// multiply .t() * b
qmatrix qmatrix::t_mul(const qmatrix &b) const
{
  qmatrix a(n, b.n);
  double one = 1.0, zero = 0.0;
  int n = this->n, m = this->m, bm = b.m, bn = b.n;
  dgemm_("t", "t", &n, &bn, &m, &one, d, &m, b.d, &bm, &zero, a.d, &m);
  return a;
}


// multiply .t() * b.t()
qmatrix qmatrix::t_mul_t(const qmatrix &b) const
{
  qmatrix a(n, b.m);
  double one = 1.0, zero = 0.0;
  int n = this->n, m = this->m, bm = b.m;
  dgemm_("t", "t", &n, &bm, &m, &one, d, &m, b.d, &bm, &zero, a.d, &m);
  return a;
}


// update += alpha * x * y.t()
void qmatrix::rank1_update(double alpha, const qmatrix &x, const qmatrix &y)
{
  int inc = 1;
  dger_(&m, &n, &alpha, x.d, &inc, y.d, &inc, d, &m);
}


// solve x = A.i() * b
qmatrix qmatrix::solve(const qmatrix &b) const
{
  if (m != n) throw qmat_exception("solve: matrix must be square");
  if (m != b.m) throw qmat_exception("solve: b must be same size as matrix");

  qmatrix x(b), A(*this);
  int ipiv[n], info;

  dgesv_(&n, &b.n, A.d, &A.m, ipiv, x.d, &x.m, &info);
  if (info < 0) throw qmat_exception("solve: argument had illegal value");
  if (info > 0) throw qmat_exception("solve: matrix is singular, use .i()");

  return x;
}

// solve x = A.i() * b for positive definite A
qmatrix qmatrix::solve_pd(const qmatrix &b) const
{

}

// dot product
double qmatrix::dot(const qmatrix &b) const
{
  if (m*n != b.m*b.n)
    throw qmat_exception("dot: matrices must be same size");
  int one = 1, m = this->m * this->n;
  return ddot_(&m, d, &one, b.d, &one);
}
  

// norm (Euclidean or Frobenius)
double qmatrix::norm2() const
{
  int s=m*n, one=1;
  return dnrm2_(&s, d, &one);
}

  
// cross product
qmatrix qmatrix::cross(const qmatrix &b) const
{
  if (m != 3 || b.m != 3 || n != 1 || b.n != 1)
    throw qmat_exception("cross: matrices must be vectors of length 3");
  qmatrix a(3);
  a(0) = (*this)(1) * b(2) - (*this)(2) * b(1);
  a(1) = (*this)(2) * b(0) - (*this)(0) * b(2);
  a(2) = (*this)(0) * b(1) - (*this)(1) * b(0);
  return a;
}

// transpose
qmatrix qmatrix::t() const
{
  qmatrix a(n,m);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      a.d[i*n+j] = d[j*m+i];
    }
  }
  return a;
}


// absolute value of entries
qmatrix qmatrix::abs() const
{
  qmatrix a(m,n);

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      a.d[i*m+j] = fabs(d[i*m+j]);
    }
  }
  return a;
}

// return a matrix of entry signs
qmatrix qmatrix::sign() const
{
  qmatrix a(m,n);

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      a.d[i*m+j] = (d[i*m+j] < 0 ? -1.0 : (d[i*m+j] > 0 ? 1.0 : 0.0));
    }
  }
  return a;
}


// return a matrix of element-wize square roots
qmatrix qmatrix::el_sqrt() const
{
  qmatrix a(m,n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      a.d[i*m+j] = sqrt(d[i*m+j]);
    }
  }
  return a;
}

  

// return number of non-zero elements
int qmatrix::nnz() const
{
  int num_nz = 0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      if (fabs(d[i*m+j]) > 10e-9) {
	num_nz++;
      }
    }
  }
  return num_nz;
}
      



// svd
void qmatrix::svd(qmatrix &U, qmatrix &S, qmatrix &VT) const
{
  
  int r = MIN(m, n);
  if (U.m != m || U.n != r || S.m != r || S.n != r || VT.m != r || VT.n != n)
    throw qmat_exception("svd: matrices are incorrect sizes");
  
  int info, lwork = 10*r + 3*m;
  qmatrix s(r);
  qmatrix work(lwork);
  char cS = 'S';
  qmatrix temp;
  temp = *this;
  
  dgesvd_(&cS, &cS, (int*)&m, (int*)&n, temp.d, (int*)&m, s.d, U.d, (int*)&m,
	  VT.d, &r, work.d, &lwork, &info);
  
  if (info < 0)
    throw qmat_exception("svd: incorrect input");
  if (info > 0)
    throw qmat_exception("svd: did not converge");

  S = 0.0;
  for (int i = 0; i < r; i++) S(i,i) = s(i);
}
  

// inversion (really psuedo-inverse, using svd for simplicity)
qmatrix qmatrix::i() const
{
  int r = MIN(m, n);
  qmatrix inv(m,n);
  qmatrix U(m,r), S(r,r), VT(r,n);
  svd(U,S,VT);
  for (int j = 0; j < r; j++) {
    if (S(j,j) > INVERSE_EPSILON) {
      S(j,j) = 1.0 / S(j,j);
    } else {
      S(j,j) = 0.0;
    }
  }
  inv = VT.t() * S * U.t();
  return inv;
}


// maximum element
double qmatrix::max(int *i, int *j) const
{
  double x = (*this)(0,0);
  if (i != NULL) *i = 0;
  if (j != NULL) *j = 0;
  
  for (int k = 0; k < m; k++) {
    for (int l = 0; l < n; l++) {
      if ((*this)(k,l) > x) {
	x = (*this)(k,l);
	if (i != NULL) *i = k;
	if (j != NULL) *j = l;
      }
    }
  }
  return x;
}

// minimum element
double qmatrix::min(int *i, int *j) const
{
  double x = (*this)(0,0);
  if (i != NULL) *i = 0;
  if (j != NULL) *j = 0;
  
  for (int k = 0; k < m; k++) {
    for (int l = 0; l < n; l++) {
      if ((*this)(k,l) < x) {
	x = (*this)(k,l);
	if (i != NULL) *i = k;
	if (j != NULL) *j = l;
      }
    }
  }
  return x;
}

// mean of all elements
double qmatrix::mean() const
{
  double x = 0.0;
  for (int i = 0; i < m ; i++) {
    for (int j = 0; j < n; j++) {
      x += (*this)(i,j);
    }
  }
  return x / (m*n);
}

// standard deviation of all elements
double qmatrix::std() const
{
  double x = 0.0;
  double mean = this->mean();
  for (int i = 0; i < m ; i++) {
    for (int j = 0; j < n; j++) {
      x += ((*this)(i,j) - mean) * ((*this)(i,j) - mean);
    }
  }
  return sqrt(x / (m*n));
}

// sum of all elements
double qmatrix::sum() const
{
  double x = 0.0;
  for (int i = 0; i < m ; i++) {
    for (int j = 0; j < n; j++) {
      x += (*this)(i,j);
    }
  }
  return x;
}

  

// write to a file (c style)
void qmatrix::write(FILE* f)
{
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(f, "%lf ", (*this)(i,j));
    }
    if (i != m-1) fprintf(f, "\n");
  }
}

// write to a file (c++ style)
void qmatrix::write(ostream &out)
{
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      out << (*this)(i,j) << " ";
    }
    if (i != m-1) out << endl;
  }
}

// write to a string (we don't do bounds checking)
void qmatrix::write(char* str)
{
  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      idx += sprintf(&str[idx], "%lf ", (*this)(i,j));
    }
    if (i != m-1) idx += sprintf(&str[idx], "\n");
  }
}


// read from a file (c style)
void qmatrix::read(FILE *f)
{
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fscanf(f, "%lf ", &d[j*m + i]);
    }
  }
}

// read from a file (c++ style)
void qmatrix::read(istream &in)
{
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      in >> d[j*m + i];
    }
  }
}

// read from a string (no bounds checking)
void qmatrix::read(const char *str)
{
  istringstream in(string(str), istringstream::in);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      in >> d[j*m + i];
    }
  }
}


// dump entire matrix (with size info) to a file
void qmatrix::dump_to_file(const char* filename)
{
  ofstream fout(filename);
  fout << *this << endl;
}


// read entire matrix (with size info) from file, discarding current data  
void qmatrix::load_from_file(const char* filename)
{
  char c;
  double x;
  ifstream fin(filename);
  if (!fin.good()) return;

  // figure out how many columns there are in the file
  n = 0;
  fin.get(c);
  while (c != '\n' && c != '\r') {
    if (!isspace(c)) {
      n++;
      while (!isspace(c)) fin.get(c);
    } else {
      while (isspace(c) && c != '\n' && c != '\r') fin.get(c);
    }
  }

  // figure out how many rows there are
  m = 0;
  while (fin.good()) {
    m++;
    for (int i = 0; i < n; i++) {
      fin >> x;
    }
  }

  // read the file
  if (d != 0) delete [] d;
  d = new double [m*n];
  fin.clear();
  fin.seekg(0, ios::beg);
  fin >> *this;
}




// operators for left-side scalar addition, subtraction, multiplication
qmatrix operator+(double x, const qmatrix &b)
{
  qmatrix a(b.size1(), b.size2());
  for (int i = 0; i < b.size1(); i++) {
    for (int j = 0; j < b.size2(); j++) {
      a(i,j) = x + b(i,j);
    }
  }
  return a;
}

qmatrix operator-(double x, const qmatrix &b)
{
  qmatrix a(b.size1(), b.size2());
  for (int i = 0; i < b.size1(); i++) {
    for (int j = 0; j < b.size2(); j++) {
      a(i,j) = x - b(i,j);
    }
  }
  return a;
}

qmatrix operator*(double x, const qmatrix &b)
{
  qmatrix a(b.size1(), b.size2());
  for (int j = 0; j < b.size2(); j++) {
    for (int i = 0; i < b.size1(); i++) {
      a(i,j) = x * b(i,j);
    }
  }
  return a;
}


// ostream overloading
ostream& operator<<(ostream& out, const qmatrix &b)
{
  for (int i = 0; i < b.size1(); i++) {
    for (int j = 0; j < b.size2(); j++) {
      out << b(i,j) << " ";
    }
    if (i != b.size1()-1) out << endl;
  }
  return out;
}


// istream overloading
istream& operator>>(istream &in, qmatrix &b)
{
  for (int i = 0; i < b.size1(); i++) {
    for (int j = 0; j < b.size2(); j++) {
      in >> b(i,j);
    }
  }
  return in;
}


		    
// generate a matrix with random numbers in [0, 1]
qmatrix qmat_rand(int m, int n)
{
  qmatrix a(m,n);
  for (int i = 0; i < m; i++) {
    for (int j  = 0; j < n; j++) {
      a(i,j) = (double)rand() / RAND_MAX;
    }
  }
  return a;
}



// generate a matrix with random normal numbers
qmatrix qmat_randn(int m, int n)
{
  qmatrix a(m,n);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      double x, y, r;
      // logic taken from GSL code
      do {
	x = 2.0*((double)rand() / RAND_MAX) - 1.0;
	y = 2.0*((double)rand() / RAND_MAX) - 1.0;
	r = x * x + y * y;
      } while (r > 1.0 || r == 0.0);
      a(i,j) = y * sqrt (-2.0 * log(r) / r);
    }
  }
  return a;
}


// generate a matrix of all zeros
qmatrix qmat_zeros(int m, int n)
{
  qmatrix a(m,n);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      a(i,j) = 0.0;
    }
  }
  return a;
}

// generate a matrix of all ones
qmatrix qmat_ones(int m, int n)
{
  qmatrix a(m,n);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      a(i,j) = 1.0;
    }
  }
  return a;
}


// return a range
qmatrix qmat_range(double min, double inc, double max)
{
  qmatrix a((int)ceil((max - min)/inc) + 1);
  int idx = 0;
  
  for (double x = min; x < max; x += inc) {
    a(idx++) = x;
  }
  a(idx) = max;
  return a;
}

// generate a 1x1 matrix with element 1
qmatrix qmat_one()
{
  qmatrix a(1,1);
  a(0,0) = 1.0;
  return a;
}

// generate a 1x1 matrix with element 0
qmatrix qmat_zero()
{
  qmatrix a(1,1);
  a(0,0) = 0.0;
  return a;
}


// generate the identity matrix
qmatrix qmat_identity(int n)
{
  qmatrix a;
  a = qmat_zeros(n,n);
  for (int i = 0; i < n; i++) {
    a(i,i) = 1.0;
  }
  return a;
}

// generate a matrix from a diagonal
qmatrix qmat_diag(const qmatrix& diag)
{
  qmatrix a;
  a = qmat_zeros(diag.size1(), diag.size1());
  for (int i = 0; i < diag.size1(); i++) {
    a(i,i) = diag(i);
  }
  return a;
}


// constructor for exception class
extern char** environ;
qmat_exception::qmat_exception(const char *msg)
{
  void *trace[32];
  int traceSize = 0;
  
  traceSize = backtrace(trace, 32);
  str = "\nEXCEPTION\n";
  str += msg;
  str += "\n";

  // get name of executable
  int argc = 0;
  char **ptr = environ-2;
  while ((int)*ptr != argc) {
    argc++;
    ptr--;
  }
  ptr++;

  // get a complete backtrace using addr2line
  int filedes[2];
  char hexString[50];
  for (int i = 0; i < traceSize; i++) {
    pipe(filedes);
    sprintf(hexString, "0x%x", (unsigned int)trace[i]);
    if (fork() == 0) {
      dup2(filedes[1], fileno(stdout));
      close(filedes[1]);
      execl("/usr/bin/addr2line","addr2line", "-e", *ptr, "-f",
            "-C", hexString, (char*)0);
      exit(0);
    }
    close(filedes[1]);
    char c;
    while (read(filedes[0], &c, 1) != 0) str += c;
    close(filedes[0]);
  }
  str += "\n";
}



/*
int main()
{
  int argc = 0;
  char **ptr = environ-2;
  while ((int)*ptr != argc) {
    argc++;
    ptr--;
  }
  ptr++;

  int filedes[2];
  char hexString[50];
  pipe(filedes);

  if (fork() == 0) {
    dup2(filedes[1], fileno(stdout));
    close(filedes[1]);
    execl("/usr/bin/addr2line", "addr2line", "-e", *ptr, "-f", "-C",
           "0x0", (char*)0);
    exit(0);
  }
  close(filedes[1]);
  int c, i = 0;
  while (read(filedes[0], &c, 1) != 0) {
    fputc(c, stdout);
  }

}
*/

