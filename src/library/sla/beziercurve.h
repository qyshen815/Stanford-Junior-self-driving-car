//-*-c++-*-
#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

/** 
 * @file beziercurve.h
 * @brief Generalized Bezier Curve class for M-Dimensional, Nth order.
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 30 August 2006 - Started (JD)
 */

#include <iostream>

#include <matmath.h>
#include <arrays.h>
#include <gemat.h>

using namespace sla;

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  //////////////////// BezierCurve ////////////////////
  /** 
   * An M-Dimensional, Nth order Bezier Curve.
   */ 
  template <class T, int M, int N> 
  class BezierCurve {
  public:
    // Constructors
    BezierCurve();

    // Mutators
    void setCoefficients(const Mat<T,M,N+1>& a);
    void computeInterpolant(const Vec<T,N+1>& t);
    T fitDataFixedEnds(const Array1<T> t, const Array1< Vec<T,M> >& p);
    T fitProjectedDataFixedEnds(const Array1<T> t, const Array1< Vec<T,M> >& p);

    // Accessors
    Vec<T,M> eval(const T t) const;
    Mat<T,M,N+1> getCoefficients() const;

    // Utility functions
    Vec<T,N+1> bezierVec(const T t) const;
    Vec<T,N+1> polyVec(const T t) const;
    SMat<T,N+1> polyMat(const Vec<T,N+1>& t) const;

    // Data
    Vec< Vec<T,M>, N+1> x;
  };

  // Declare a few common typdefs
  typedef BezierCurve<float,2,3> BezierCurve23f;
  typedef BezierCurve<double,2,3> BezierCurve23d;


  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Vec ////////////////////
  
  // Constructors
  
  /** Ctor that does no initalization.
   */
  template <class T, int M, int N> 
  inline BezierCurve<T,M,N>::BezierCurve() {}
  
  // Mutators
  
  /** Set the coefficients of the equivalent polynomial.
   */
  template <class T, int M, int N> 
  inline void BezierCurve<T,M,N>::setCoefficients(const Mat<T,M,N+1>& a) {
    int nck;
    for (int k=0;k<=N;k++) {
      if (k==0) nck = 1;
      else nck = (nck*(N-(k-1)))/k;
      x(k) = (T(1)/T(nck))*a.getCol(k);
      int kci;
      for (int i=0;i<k;i++) {
	if (i==0) kci = 1;
	else kci = (kci*(k-(i-1)))/i;
	int sgn = ((k+i)%2)?-1:1;
	x(k) -= (T(kci)*T(sgn))*x(i);
      }
    }
  }

  /** Compute the bezier curve that keeps the current endpoints and
  *  fits the rest of the data as well as possible
  * @return is the sum of squared residuals between the data points and the fit
  * @param t is the array of time values.
  * @param p is the array of data points to fit.
  */
  template <class T, int M, int N> inline 
    T BezierCurve<T,M,N>::fitDataFixedEnds(const Array1<T> t, 
    const Array1< Vec<T,M> >& p) {
      if (t.length() != p.length()) {
        cerr << "BezierCurve<T,M,N>::fitDataFixedEnds: "
          << "t and p arrays are not of same size. " << endl << flush;
        exit(1);
      }

      if (N <= 1) {
        cerr << "BezierCurve<T,M,N>::fitDataFixedEnds: "
          << "must be at least 2nd order." << endl << flush;
        exit(1);
      }

      if (t.length() == 0) {
        cerr << "BezierCurve<T,M,N>::fitDataFixedEnds: "
          << "t and p have no data!" << endl << flush;
        exit(1);
      }

      // Allocate some memory
      int num_samples = t.length();
      GEMat<double> residual(num_samples*M,1);
      GEMat<double> jacobian(num_samples*M,M*(N-1)); 
      
      GEMat<double> hessian(M*(N-1),M*(N-1));
      GEMat<double> gradient(M*(N-1),1);    

      // For each sample, add some things to the jacobian and residual
      jacobian.set(0.0);
      hessian.set(0.0);
      gradient.set(0.0);
      for (int i=0;i<t.length();i++) {
        // compute the Bezier vector
        Vec<T,N+1> b = bezierVec(t(i));

        // compute the residual contribution
        for (int j=0;j<M;j++) {
          residual(i*M+j) = double(p(i)(j) - (b(0)*x(0)(j) + b(N)*x(N)(j)));
        }

        // compute the jacobian contribution
        for (int j=0;j<M;j++) for (int k=1;k<N;k++) {
          jacobian(i*M+j,(k-1)*M+j) = b(k);
        }
      }

      // solve the system
      for (int i=0;i<hessian.size(0);i++) hessian(i,i) = 1e-5;
      jacobian.mm(jacobian,hessian,true,false,1.0,1.0);
      jacobian.mm(residual,gradient,true,false,1.0,0.0);
      hessian.sv(gradient);
      //jacobian.sv(residual);

      // Copy result into control points
      for (int k=1;k<N;k++) for (int j=0;j<M;j++) {
        x(k)(j) = (T)gradient((k-1)*M+j);
        //x(k)(j) = (T)residual((k-1)*M+j);
      }
      T sum_residual_sqr = T(0);
      for (int i=0;i<t.length();i++) {
        sum_residual_sqr += (p(i) - eval(t(i))).normSqr();
      }
      return sum_residual_sqr; 
  }

  /** Compute the bezier curve that keeps the current endpoints and
  *  fits the rest of the data as well as possible
  * @return is the sum of squared residuals between the data points and the fit
  * @param t is the array of time values.
  * @param p is the array of data points to fit.
  */
  template <> inline 
    double BezierCurve<double,2,3>::fitProjectedDataFixedEnds(const Array1<double> t, 
    const Array1< Vec<double,2> >& p) {
      if (t.length() != p.length()) {
        cerr << "BezierCurve<double,2,3>::fitDataFixedEnds: "
          << "t and p arrays are not of same size. " << endl << flush;
        exit(1);
      }

      if (t.length() == 0) {
        cerr << "BezierCurve<double,2,3>::fitDataFixedEnds: "
          << "t and p have no data!" << endl << flush;
        exit(1);
      }

      // Allocate some memory
      int num_samples = t.length();
      GEMat<double> residual(num_samples,1);
      GEMat<double> jacobian(num_samples,4); 
      GEMat<double> hessian(4,4);
      GEMat<double> gradient(4,1);    
      Array1< Vec2d > normal(t.length());
      Vec2d v0, v1;

      // For each sample, add some things to the jacobian and residual
      residual.set(0.0);
      jacobian.set(0.0);
      hessian.set(0.0);
      gradient.set(0.0);
      for (int i=0;i<t.length();i++) {
        // compute the Bezier vector
        Vec<double,4> b = bezierVec(t(i));

        // construct the normal vector
        if (i == 0)             v0 = p(0) - x(0); 
        else                    v0 = p(i) - p(i-1); 
        if (i == t.length()-1)  v1 = x(3) - p(i); 
        else                    v1 = p(i+1) - p(i); 
        normal(i) = 0.5*(v0 + v1);
        normal(i).normalize();
        normal(i).set(-normal(i)(1),normal(i)(0));

        // compute the residual contribution
        residual(i) += normal(i).dot(p(i) - (b(0)*x(0) + b(3)*x(3)));

        // compute the jacobian contribution
        jacobian(i,0) = b(1)*normal(i)(0);
        jacobian(i,1) = b(1)*normal(i)(1);
        jacobian(i,2) = b(2)*normal(i)(0);
        jacobian(i,3) = b(2)*normal(i)(1);
      }

      // solve the system
      for (int i=0;i<hessian.size(0);i++) hessian(i,i) = 1e-5;
      jacobian.mm(jacobian,hessian,true,false,1.0,1.0);
      jacobian.mm(residual,gradient,true,false,1.0,0.0);
      hessian.sv(gradient);

      // Copy result into control points
      for (int k=1;k<3;k++) for (int j=0;j<2;j++) {
        x(k)(j) = gradient((k-1)*2+j);
      }
      double sum_residual_sqr = 0.0;
      for (int i=0;i<t.length();i++) {
        sum_residual_sqr += sqr((p(i) - eval(t(i))).dot(normal(i)));
      }
      return sum_residual_sqr; 
  }



  /** Compute the bezier curve that passes through the given points.
   * @param t(i) corresponds to BezierCurve::x(i) for i=0,...,N+1
   */
  template <class T, int M, int N> 
  inline void BezierCurve<T,M,N>::computeInterpolant(const Vec<T,N+1>& t) {
    // make sure no repeats in t and that strictly increasing
    for (int i=1;i<=N;i++) {
      if (t(i) <= t(i-1)) {
	cerr << "BezierCurve<T,M,N>::computeInterpolant: "
	     << "Must provide strictly increasing values for t for fit. "
	     << "Exiting." << endl << flush;
	exit(1);
      }
    }

    // fit for each dimension independently
    Mat<T,M,N+1> a;
    for (int k=0;k<M;k++) {
      SMat<T,N+1> t_mat = polyMat(t);
      Vec<T,N+1> x_vec;
      for (int i=0;i<=N;i++) x_vec(i) = x(i)(k);
      t_mat.invert();
      Vec<T,N+1> x_out = t_mat*x_vec;
      a.setRow(k,x_out);
    }
    setCoefficients(a);
  }
  
  
  // Accessors
  
  /** Evaluate the curve at the parameter t.
   * @param t is the evaluate point.
   */
  template <class T, int M, int N> 
  inline Vec<T,M> BezierCurve<T,M,N>::eval(const T t) const {
    T omt = T(1)-t;
    switch (N) {
    case 1: return omt*x(0) + t*x(1);
    case 2: return sqr(omt)*x(0) + (T(2)*t*omt)*x(1) + sqr(t)*x(2);
    case 3: {
      T omt_sqr = sqr(omt);
      T t_sqr = sqr(t);
      return ((omt*omt_sqr)*x(0) + (T(3)*t*omt_sqr)*x(1) +
	      (T(3)*t_sqr*omt)*x(2) + (t*t_sqr)*x(3)); }
    default:
      Vec<T,M> z(T(0));
      Vec<T,N+1> b = bezierVec(t);
      for (int k=0;k<=N;k++) z += b(k)*x(k);
      return z;
    }
  }

  /** Return the coefficients of the equivalent polynomial.
   */
  template <class T, int M, int N> 
  inline Mat<T,M,N+1> BezierCurve<T,M,N>::getCoefficients() const {
    Mat<T,M,N+1> a;
    int nck;
    for (int k=0;k<=N;k++) {
      if (k==0) nck = 1;
      else nck = (nck*(N-(k-1)))/k;
      Vec<T,M> ak(T(0));
      int kci;
      for (int i=0;i<=k;i++) {
	if (i==0) kci = 1;
	else kci = (kci*(k-(i-1)))/i;
	int sgn = ((k+i)%2)?-1:1;
	ak += (T(kci)*T(sgn))*x(i);
      }
      ak *= T(nck);
      a.setCol(k,ak);
    }
    return a;
  }

  /** Return the bezier vector.
   */
  template <class T, int M, int N> 
  inline Vec<T,N+1> BezierCurve<T,M,N>::bezierVec(const T t) const {
    Vec<T,N+1> tv;
    T omt = (T(1)-t);
    switch (N) {
    case 1: {
      tv(0) = omt; 
      tv(1) = t;
      break; }
    case 2: {
      tv(0) = sqr(omt);
      tv(1) = T(2)*omt*t;
      tv(2) = sqr(t);
      break; }
    case 3: {
      T omt_sqr = sqr(omt);
      T t_sqr = sqr(t);
      tv(0) = omt*omt_sqr;
      tv(1) = T(3)*omt_sqr*t;
      tv(2) = T(3)*omt*t_sqr;
      tv(3) = t*t_sqr;
      break; }
    default: {
      int nck;
      for (int k=0;k<=N;k++) {
        if (k==0) nck = 1;
        else nck = (nck*(N-(k-1)))/k;
        tv(k) = T(nck)*T(pow(double(omt),double(N-k))*pow(double(t),double(k)));
      }
      break; }
    }
    return tv;
  }
  
  /** Return the polynomial vector.
   */
  template <class T, int M, int N> 
  inline Vec<T,N+1> BezierCurve<T,M,N>::polyVec(const T t) const {
    Vec<T,N+1> tv;
    tv(0) = T(1);
    for (int i=1;i<=N;i++) tv(i) = tv(i-1)*t;
    return tv;
  }

  /** Return the polynomial matrix.
   */
  template <class T, int M, int N> 
  inline SMat<T,N+1> BezierCurve<T,M,N>::polyMat(const Vec<T,N+1>& t) const {
    SMat<T,N+1> t_mat;      
    for (int i=0;i<=N;i++) {
      t_mat(i,0) = T(1);
      for (int j=1;j<=N;j++) {
	t_mat(i,j) = t_mat(i,j-1)*t(i);
      }
    }
    return t_mat;
  }

} // end namespace sla

#endif
