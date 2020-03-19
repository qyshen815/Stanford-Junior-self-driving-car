#ifndef CURVESMOOTHER_BEZCTXPS_H_
#define CURVESMOOTHER_BEZCTXPS_H_

#include "clothoid.h"

namespace vlr {
class BezCtxPS : public BezierContext {

public:
  BezCtxPS(FILE* f);
  virtual ~BezCtxPS();
  void scale(double s) {scale_ = s;}

private:
  void moveto(double x, double y, int is_open);
  void lineto(double x, double y);
  void curveto(double x1, double y1, double x2, double y2, double x3, double y3);

  bool is_open_;
  double x_, y_;
  double scale_;
  FILE* f_;

public:
  static const std::string ps_prolog_, ps_postlog_;
};

} // namespace vlr

#endif
