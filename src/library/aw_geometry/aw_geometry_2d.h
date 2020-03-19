/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_GEOMETRY_2D_H
#define AW_GEOMETRY_2D_H

#include <sla.h>

namespace vlr {

typedef struct {
  double x;
  double y;
} kogmo_point_2D_t;

typedef struct {
  float x1, y1;
  float x2, y2;
} kogmo_line_2Df_t;

// squared euclidean distance
double dist2(const sla::Vec2d &a, const sla::Vec2d &b);
// Calculates the closest point to x on a line segmente a-->b
// if the distance to closest point is smaller than d2, both
// d2 and cp will be overwritten
bool closer_on_line(const sla::Vec2d &x, const sla::Vec2d &a, const sla::Vec2d &b, double &d2, sla::Vec2d &cp);
// is the circle centered at b with radius r
// fully within the rectangle centered at bc, with radius br?
bool circle_within_bounds(const sla::Vec2d &b, double r, const sla::Vec2d &bc, double br);
// is the circle centered at b with radius r
// fully within the rectangle centered from min to max?
bool circle_within_bounds(const sla::Vec2d &b, double r, const sla::Vec2d &min, const sla::Vec2d &max);
// does the circle centered at b, with radius r,
// intersect the rectangle centered at bc, with radius br?
bool bounds_overlap_circle(const sla::Vec2d &b, double r, const sla::Vec2d &bc, double br);
// Which of the four edges is point P outside of?
long bevel_1d(const sla::Vec2d &p);
// Which of the four edge lines is point P outside of?
long bevel_2d(const sla::Vec2d &p);
// 2D linear interpolation
sla::Vec2d lerp(double t, const sla::Vec2d &a, const sla::Vec2d &b);
// Test the point "alpha" of the way from P1 to P2
// See if it is on a edge of the rectangle
// Consider only faces in "mask"
bool point_on_edge(const sla::Vec2d &p1, const sla::Vec2d &p2, double alpha, long mask);
// Compute intersection of P1 --> P2 line segment with edge lines
// Then test intersection point to see if it is on cube face
// Consider only face planes in "outcode_diff"
// Note: Zero bits in "outcode_diff" means edge is outside of
bool segment_on_edge(const sla::Vec2d &p1, const sla::Vec2d &p2, long outcode_diff);
// true if line t1,t2 is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool line_outside_of_rect(const sla::Vec2d &c, double s, const sla::Vec2d &t1, const sla::Vec2d &t2);
// true if line p is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool point_outside_of_rect(const sla::Vec2d &c, double s, const sla::Vec2d &p);
// true if rectangle 1 intersects rectangle 2
// c == center
// theta == angle
// w == width
// l == length
bool rect_rect_X(const sla::Vec2d &c1, double theta1, double w1, double l1,
                 const sla::Vec2d &c2, double theta2, double w2, double l2);
// true if line intersects rectangle
// line t1-->t2
// c == center
// theta == angle
// w == width
// l == length
bool line_rect_X(const sla::Vec2d &t1, const sla::Vec2d &t2,
                 const sla::Vec2d &c, double theta, double w, double l);

} // namespace vlr

#endif // AW_GEOMETRY_2D_H
