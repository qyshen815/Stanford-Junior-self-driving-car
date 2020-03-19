#ifndef __QMATRIX_H__
#define __QMATRIX_H__

#define INVERSE_EPSILON 1.0e-10

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>


#ifndef MIN
#define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#define MAX(a,b) (a > b ? a : b)
#endif

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif

#ifndef CLIP
#define CLIP(x,a,b) MAX((a), MIN((x), (b)))
#endif

#ifndef SIGN
#define SIGN(x) ((x) >= 0.0 ? 1.0 : -1.0)
#endif


using namespace std;




// exception handler
class qmat_exception
{
 public:
  qmat_exception(const char *msg);
  ~qmat_exception() {;}
  const char* what() { return str.c_str(); }

 private:
  string str;
};



// matrix class
class qmatrix
{
 public:

  // constructors / destructors
  qmatrix();
  qmatrix(int m, int n);
  qmatrix(int m);
  qmatrix(int m, int n, double x);
  qmatrix(int m, double x);
  qmatrix(int m, int n, const char *vals);
  qmatrix(int m, const char* vals);
  qmatrix(int m, int n, const double *vals);
  qmatrix(const qmatrix& b);
  qmatrix(const char* filename);
  ~qmatrix();


  // operators
  double& operator()(int i, int j);
  double operator()(int i, int j) const;
  double& operator()(int i);
  double operator()(int i) const;
  qmatrix& operator=(const qmatrix& b);
  qmatrix& operator=(double x);
  qmatrix& operator=(const double *x);
  
  qmatrix& operator+=(const qmatrix& b);
  qmatrix& operator+=(double x);
  qmatrix& operator-=(const qmatrix& b);
  qmatrix& operator-=(double x);
  qmatrix& operator*=(const qmatrix& b);
  qmatrix& operator*=(double x);
  qmatrix& operator/=(const qmatrix& b);
  qmatrix& operator/=(double x);

  qmatrix operator+(const qmatrix& b) const;
  qmatrix operator+(double x) const;
  qmatrix operator-(const qmatrix& b) const;
  qmatrix operator-(double x) const;
  qmatrix operator*(const qmatrix &b) const;
  qmatrix operator*(double x) const;


  // submatrix observers / accessors
  qmatrix submatrix(int rl, int ru, int cl, int cu) const;
  qmatrix rows(int rl, int ru) const;
  qmatrix cols(int cl, int cu) const;
  qmatrix row(int r) const;
  qmatrix col(int c) const;
  qmatrix submatrix(int *ri, int ri_count, int *ci, int ci_count) const;
  qmatrix rows(int *ri, int ri_count) const;
  qmatrix cols(int *ci, int ci_count) const;

  void set_submatrix(int rl, int ru, int cl, int cu, const qmatrix &b);
  void set_rows(int rl, int ru, const qmatrix &b);
  void set_cols(int cl, int cu, const qmatrix &b);
  void set_row(int r, const qmatrix &b);
  void set_col(int c, const qmatrix &b);
  void set_submatrix(int *ri, int ri_count, int *ci, int ci_count,
		     const qmatrix &b);
  void set_rows(int *ri, int ri_count, const qmatrix &b);
  void set_cols(int *ci, int ci_count, const qmatrix &b);


  void swap_rows(int r1, int r2);
  void swap_cols(int c1, int c2);
  void resize(int m, int n);
  void resize_zero(int m, int n);
  void reshape(int m, int n);
  qmatrix vec();
  qmatrix repmat(int m, int n) const;

  // multiplication
  qmatrix mul_t(const qmatrix &b) const;
  qmatrix t_mul(const qmatrix &b) const;
  qmatrix t_mul_t(const qmatrix &b) const;
  void rank1_update(double alpha, const qmatrix &x, const qmatrix &y);


  // solve linear equations
  qmatrix solve(const qmatrix &b) const;
  qmatrix solve_pd(const qmatrix &b) const;

  // operations
  double dot(const qmatrix &b) const;
  double norm2() const;
  qmatrix cross(const qmatrix &b) const;
  qmatrix t() const;
  qmatrix i() const;
  void svd(qmatrix &U, qmatrix &S, qmatrix &VT) const;

  qmatrix abs() const;
  qmatrix sign() const;
  qmatrix el_sqrt() const;
  int nnz() const;

  double max(int *i = NULL, int *j = NULL) const;
  double min(int *i = NULL, int *j = NULL) const;
  double mean() const;
  double std() const;
  double sum() const;
  

  int size1() const { return m; }
  int size2() const { return n; }
  double* ptr() const { return d; }
  double* ptr(int i, int j) const { return &d[j*m+i]; }
  
  void write(FILE* f);
  void write(ostream &out);
  void write(char* str);
  void read(FILE *f);
  void read(istream &in);
  void read(const char *str);

  void dump_to_file(const char* filename);
  void load_from_file(const char* filename);
  
 private:
  int m, n;
  double *d;
};
  

// operators for left-side scalar addition, subtraction, multiplication
qmatrix operator+(double x, const qmatrix &b);
qmatrix operator-(double x, const qmatrix &b);
qmatrix operator*(double x, const qmatrix &b);
ostream& operator<<(ostream& out, const qmatrix &b);
istream& operator>>(istream &in, qmatrix &b);

qmatrix qmat_rand(int m, int n);
qmatrix qmat_randn(int m, int n);
qmatrix qmat_zeros(int m, int n);
qmatrix qmat_ones(int m, int n);
qmatrix qmat_range(double min, double inc, double max);
qmatrix qmat_one();
qmatrix qmat_zero();
qmatrix qmat_identity(int n);
qmatrix qmat_diag(const qmatrix& diag);


#endif
