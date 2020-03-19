/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef CURVESMOOTHER_H
#define CURVESMOOTHER_H

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <cmath>

#include <poly_traj.h>
#include <planner_interface.h>

#include <trajectory_points_interface.h>

namespace vlr {

  class CurveSmoother {

  public:
    CurveSmoother();
    ~CurveSmoother();

  public:
    class point2d {
    public:
     double x, y;
     const point2d operator+(const point2d& other) const {
       point2d res;
       res.x = x + other.x;
       res.y = y + other.y;
       return res;
     }

     const point2d operator-(const point2d& other) const {
       point2d res;
       res.x = x - other.x;
       res.y = y - other.y;
       return res;
     }

     const point2d operator*(double scalar) const {
       point2d res;
       res.x = scalar * x;
       res.y = scalar * y;
       return res;
     }
   };


//    bool SmoothCurvePoints(CurvePoints& incurve,
//        CurvePoints& outcurve);

//    void setInitialTheta(double theta) {
//      m_lasttheta=theta;
//    }
//    void setInitialKappa(double kappa) {
//      m_lastkappa=kappa;
//    }

  public:
//    static void sample_as_cbspline(int degree, int dense, double* xcoord, double* ycoord, int numPoints,
//        CurvePoints& splinepoints);
//    void sampleQuarticBezier(AnnieWAY::CurvePoints& points, double theta0, double kappa0,
//                             int dense, std::vector<CurvePoint>* splinepoints);

    void sample_as_cbspline_interp(int degree, int dense, dgc::PlannerTrajectory& plan,
                                              std::vector<CurvePoint>* splinepoints);
    void sample_as_cbspline(int degree, int dense, dgc::PlannerTrajectory& plan,
                                              std::vector<CurvePoint>* splinepoints);
    void sample_as_bezier(int degree, int dense, std::vector<CurvePoint>& points,
                                              std::vector<CurvePoint>* splinepoints);

    void bspline2points(int degree, int l, double* coeff, double* knots, int dense, double* points, int* point_num);
    void bspl_to_points(int degree, int l, double* coeff, double* knots, int dense, double* points, int* point_num);

    bool clothoideSpline(std::vector<CurvePoint>& points, double theta0, double kappa0,
                                        double s, int n_lookahead, std::vector<CurvePoint>* splinepoints);
    void sampleLinearEquidist(std::vector<CurvePoint>& points,  std::vector<bool>& ignore, double point_dist,
                                             std::vector<CurvePoint>* linepoints);

    void bez_to_points(int degree, int npoints, const double* coeff, double* points);
    void bez_to_points2(const double* coeff, double delta_s0, int degree, uint32_t& npoints, double* points);
    double curvatureAt0(const std::vector<CurvePoint>& p, size_t idx, int32_t degree);
    double BezierArcLength(point2d p1, point2d p2, point2d p3, point2d p4);
    void sampleCubicBezierEquidist(const std::vector<CurvePoint>& control_polygon, double point_dist, std::vector<CurvePoint>& sampled_points);

  private:
    double* create_centripetal_knotsequence(double* data_x, double* data_y, int l, int degree);
    void set_up_system(double* knots, int l, double* alpha, double* beta, double* gamma);
    void l_u_system(double* alpha, double* beta, double* gamma, int l, double* up, double* low);
    void bessel_ends(double* data, double* knots, int l);
    double* solve_system(double* up, double* low, double* gamma, int l, double* rhs);
    double deboor(int degree, double* coeff, double* knot, double u, int i);
    void bspl_kappas(double* bspl_x, double* bspl_y, double* knot, int l, int dense, double* curvatures);
    double curvature_0(double* bez_x, double* bez_y, int degree);
    void subdiv(int degree, double* coeff, double t, double* bleft, double* bright);
    void bspline_to_bezier(double* bspl, double* knot, int l, double* bez);
    double area(double* p1, double* p2,double* p3);

    void bezier2points(int degree, int l, const double* bez, int dense, double* points, int* point_num);
    double hornbez(int degree, const double* coeff, double t);
    point2d hornbez(int degree, const point2d* p, double t);

    void bezier_kappas(double* bez_x, double* bez_y, int degree, int l, int dense, double* curvatures);
    void derivative(double t, double* bez_x, double* bez_y, double& x, double& y) const;
    void derivative2(double t, double* bez_x, double* bez_y, double& x, double& y) const;

    void sampleLineLinearEquidist(std::vector<CurvePoint>::const_iterator pit0, std::vector<
        CurvePoint>::const_iterator pit1, double point_dist, std::vector<CurvePoint>& linepoints);

    void sampleCubicBezierPoints(int npoints, const point2d* control_polygon, double& s, std::vector<CurvePoint>& sampled_points);

      double Simpson(double a, double b, int n_limit, double tolerance);
    inline double BezierArcLengthFunction(double t) {
        return sqrt(std::abs( q5_ + t*(q4_ + t*(q3_ + t*(q2_ + t*q1_))) ) );
    }

    static const unsigned int cs_temp_buf_size_ = 50000;

    double* aux_; // solve_system()
    double* coeffa_; // from deboor()
    double* xp_, *yp_;
    double* alpha_, *beta_, *gamma_;
    double* up_, *low_;
    double* ergx_, *ergy_, *curvatures_;
    double* bez_x_, *bez_y_;
    double* lookahead_x_, *lookahead_y_;
    CurvePoint cp_;

//  protected:
//    double m_lasttheta;
//    double m_lastkappa;

private:
 static const double ARC_LENGTH_TOLERANCE = 0.0000001;  // Application specific tolerance
 double q1_, q2_, q3_, q4_, q5_;      // These belong to balf()
  };

} // namespace vlr

#endif /*CURVESMOOTHER_H_*/
