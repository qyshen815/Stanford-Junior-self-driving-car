#ifndef DGC_SPLINE_H
#define DGC_SPLINE_H

#include <gl_support.h>

class cubic_spline {
public:
  vlr::dgc_point2D_t deriv_eval(double t) const;
  vlr::dgc_point2D_t deriv2_eval(double t) const;

  vlr::dgc_point2D_t eval(double t) const;
  void fit(double x1, double y1, double theta1,
           double x2, double y2, double theta2);
  double length(double t1, double t2);
  double xp(int i) { return xp_[i]; }
  double yp(int i) { return yp_[i]; }
private:
  double xp_[4];
  double yp_[4];
};

#endif
