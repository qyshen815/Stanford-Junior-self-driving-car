#include <stdio.h>
#include <string>

#include "bezCtxPS.h"

namespace vlr {

const std::string BezCtxPS::ps_prolog_ = "%!PS\n"
  "/m { moveto } bind def\n"
  "/l { lineto } bind def\n"
  "/c { curveto } bind def\n"
  "/z { closepath } bind def\n"
  "1 -1 scale\n"
  "200 -500 translate\n";

const std::string  BezCtxPS::ps_postlog_ = "stroke\n"
  "showpage\n";

BezCtxPS::BezCtxPS(FILE* f) : is_open_(true), x_(0), y_(0), scale_(1.0), f_(f) {

}

BezCtxPS::~BezCtxPS() {
  if(!is_open_) {fprintf(f_, "z\n");}
}

  // This routine starts a new contour
void BezCtxPS::moveto(double x, double y, int is_open) {
  x*=scale_; y*=scale_;
  if (!is_open_) fprintf(f_, "z\n");
  fprintf(f_, "%.16f %.16f m\n", x, y);
  is_open_ = is_open;
  x_ = x;
  y_ = y;
}

  // This routine creates a linear spline from the previous point specified to this one
void BezCtxPS::lineto(double x, double y) {
  x*=scale_; y*=scale_;
  fprintf(f_, "%.16f %.16f l\n", x, y);
  x_ = x;
  y_ = y;
}

  // And this creates a cubic
void BezCtxPS::curveto(double x1, double y1, double x2, double y2, double x3, double y3) {
  x1*=scale_; y1*=scale_;
  x2*=scale_; y2*=scale_;
  x3*=scale_; y3*=scale_;

  fprintf(f_, "%.16f %.16f %.16f %.16f %.16f %.16f c\n", x1, y1, x2, y2, x3, y3);
  x_ = x3;
  y_ = y3;
}

} // namespace vlr
