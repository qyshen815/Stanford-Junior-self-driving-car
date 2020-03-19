#include <roadrunner.h>
#include "spline.h"

using namespace dgc;
using namespace vlr;

dgc_point2D_t cubic_spline::deriv_eval(double t) const 
{
  dgc_point2D_t point;

  point.x = xp_[1] + 2 * xp_[2] * t + 3 * xp_[3] * t * t;
  point.y = yp_[1] + 2 * yp_[2] * t + 3 * yp_[3] * t * t;
  return point;
}

dgc_point2D_t cubic_spline::deriv2_eval(double t) const 
{
  dgc_point2D_t point;

  point.x = 2 * xp_[2] + 6 * xp_[3] * t;
  point.y = 2 * yp_[2] + 6 * yp_[3] * t;
  return point;
}

dgc_point2D_t cubic_spline::eval(double t) const
{
  dgc_point2D_t point;
  double tpow;
  int i;

  point.x = 0;
  point.y = 0;
  tpow = 1;
  for(i = 0; i < 4; i++) {
    point.x += xp_[i] * tpow;
    point.y += yp_[i] * tpow;
    tpow *= t;
  }
  return point;
}

void cubic_spline::fit(double x1, double y1, double theta1,
                       double x2, double y2, double theta2)
{
  double k, kc1, ks1, kc2, ks2;
  double dx, dy;
  
  dx = x2 - x1;
  dy = y2 - y1;
  k = hypot(dx, dy);
  kc1 = k * cos(theta1);
  ks1 = k * sin(theta1);
  kc2 = k * cos(theta2);
  ks2 = k * sin(theta2);
  xp_[0] = x1;
  yp_[0] = y1;
  xp_[1] = kc1;
  yp_[1] = ks1;
  xp_[2] = 3 * dx - 2 * kc1 - kc2;
  yp_[2] = 3 * dy - 2 * ks1 - ks2;
  xp_[3] = -2 * dx + kc1 + kc2;
  yp_[3] = -2 * dy + ks1 + ks2;
}

double cubic_spline::length(double t1, double t2)
{
  int i;
  double d = 0;
  dgc_point2D_t p, last_p;

  last_p.x = 0;
  last_p.y = 0;
  for(i = 0; i < 100; i++) {
    p = eval(t1 + i / 100.0 * (t2 - t1));
    if(i > 0)
      d += hypot(p.x - last_p.x, p.y - last_p.y);
    last_p = p;
  }
  return d;
}

#ifdef blah
void draw_spline(double x1, double y1, double theta1,
                 double x2, double y2, double theta2, 
                 double origin_x, double origin_y)
{
  int i;
  dgc_point2D_t p;
  cubic_spline spline;

  spline.fit(x1, y1, theta1, x2, y2, theta2);
  glBegin(GL_LINE_STRIP);
  for(i = 0; i <= 20; i++) {
    p = spline.eval(i / 20.0);
    glVertex3f(p.x - origin_x, p.y - origin_y, 0.1);
  }
  glEnd();
}
#endif

