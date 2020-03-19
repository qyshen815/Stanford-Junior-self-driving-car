/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * file .......: geometry.cpp
 * authors ....: Benjamin Pitzer
 * organization: Stanford University
 * creation ...: 02/16/2007
 * revisions ..: $Id: geometry_2d.cpp,v 1.2 2007-03-04 22:17:19 pitzer Exp $
---------------------------------------------------------------------*/

//== INCLUDES =========================================================
#include <string.h>
#include <vec2.h>
#include "geometry_2d.h"
//== CONSTANTS ========================================================
#define INSIDE  false
#define OUTSIDE true

double dist2(const sla::Vec2d &a, const sla::Vec2d &b)
{
  double x = a(0)-b(0);
  double y = a(1)-b(1);
  return x*x + y*y;
}

bool closer_on_line(const sla::Vec2d &x, const sla::Vec2d &a, const sla::Vec2d &b, double &d2, sla::Vec2d &cp)
{
  sla::Vec2d ba(b(0)-a(0), b(1)-a(1));
  sla::Vec2d xa(x(0)-a(0), x(1)-a(1));

  double xa_ba = xa.dot(ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba < 0.0) {
    double nd = dist2(x,a);
    if (nd < d2) { cp = a; d2 = nd; return true; }
    return false;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  double fact = xa_ba/ba.normSqr();
  if (fact >= 1.0) {
    float nd = dist2(x,b);
    if (nd < d2) { cp = b; d2 = nd; return true; }      return false;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  double nd = xa.normSqr() - xa_ba*fact;
  if (nd < d2) {
    d2 = nd;
    cp(0) = a(0) + fact * ba(0);
    cp(1) = a(1) + fact * ba(1);
    return true;
  }
  return false;
}


// is the circle centered at b with radius r
// fully within the rectangle centered at bc, with radius br?
bool circle_within_bounds(const sla::Vec2d &b, double r, const sla::Vec2d &bc, double br)
{
  r -= br;
  if ((b(0)  - bc(0) <= r) ||
      (bc(0) - b(0)  <= r) ||
      (b(1)  - bc(1) <= r) ||
      (bc(1) - b(1)  <= r)) return false;
  return true;
}

// is the circle centered at b with radius r
// fully within the rectangle centered from min to max?
bool circle_within_bounds(const sla::Vec2d &b, double r, const sla::Vec2d &min, const sla::Vec2d &max)
{
  if ((b(0) - min(0) <= r) ||
    (max(0) - b(0) <= r) ||
    (b(1) - min(1) <= r) ||
    (max(1) - b(1) <= r)) return false;
  return true;
}

// does the circle centered at b, with radius r,
// intersect the rectangle centered at bc, with radius br?
bool bounds_overlap_circle(const sla::Vec2d &b, double r, const sla::Vec2d &bc, double br)
{
  double r2 = sla::sqr(r);
  double R_max_x,R_max_y;
  double R_min_x,R_min_y;

  /* Translate coordinates, placing the circle at the origin. */
  R_max_x = bc(0)+br-b(0);  R_max_y = bc(1)+br-b(1);
  R_min_x = bc(0)-br-b(0);  R_min_y = bc(1)-br-b(1);

  if (R_max_x < 0)      /* R to left of circle center */
    if ( R_max_y < 0)     /* R in lower left corner */
        return ((R_max_x * R_max_x + R_max_y * R_max_y) < r2);
    else if ( R_min_y > 0)  /* R in upper left corner */
        return ((R_max_x * R_max_x +  R_min_y *  R_min_y) < r2);
    else          /* R due West of circle */
        return(abs(R_max_x) < r);
   else if ( R_min_x > 0)    /* R to right of circle center */
      if ( R_max_y < 0)   /* R in lower right corner */
          return (( R_min_x *  R_min_x + R_max_y * R_max_y) < r2);
    else if ( R_min_y > 0)    /* R in upper right corner */
        return (( R_min_x *  R_min_x +  R_min_y *  R_min_y) < r2);
    else        /* R due East of circle */
        return ( R_min_x < r);
   else        /* R on circle vertical centerline */
      if ( R_max_y < 0)   /* R due South of circle */
        return (abs(R_max_y) < r);
    else if ( R_min_y > 0)    /* R due North of circle */
        return ( R_min_y < r);
    else        /* R contains circle centerpoint */
        return(true);
}

//// does the circle centered at b, with radius r,
//// intersect the rectangle centered at bc, with radius br?
//bool bounds_overlap_circle(const sla::Vec2d &b, double r, const sla::Vec2d &bc, double br)
//  {
//    double sum = 0.0, tmp;
//    if        ((tmp = bc(0)-br - b(0)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    } else if ((tmp = b(0) - (bc(0)+br)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    }
//    if        ((tmp = bc(1)-br - b(1)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    } else if ((tmp = b(1) - (bc(1)+br)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    }
//    return (sum < r*r);
//  }

 /* Which of the four edges is point P outside of? */
long bevel_1d(const sla::Vec2d &p)
{
  long outcode = 0;
  if (p(0) >  .5) outcode |= 0x01;
  if (p(0) < -.5) outcode |= 0x02;
  if (p(1) >  .5) outcode |= 0x04;
  if (p(1) < -.5) outcode |= 0x08;
  return outcode;
}

/* Which of the four corner lines is point P outside of? */
long bevel_2d(const sla::Vec2d &p)
{
  long outcode = 0;
  if ( p(0) + p(1) > 1.0) outcode |= 0x01;
  if ( p(0) - p(1) > 1.0) outcode |= 0x02;
  if (-p(0) + p(1) > 1.0) outcode |= 0x04;
  if (-p(0) - p(1) > 1.0) outcode |= 0x08;
  return outcode;
}

// 2D linear interpolation
sla::Vec2d lerp(double t, const sla::Vec2d &a, const sla::Vec2d &b)
{
  double v[2];
  double u = 1.0 - t;
  v[0]=u*a(0)+t*b(0);
  v[1]=u*a(1)+t*b(1);
  return sla::Vec2d(v[0],v[1]);
}

/* Test the point "alpha" of the way from P1 to P2 */
/* See if it is on a edge of the rectangle         */
/* Consider only faces in "mask"                   */
bool point_on_edge(const sla::Vec2d &p1, const sla::Vec2d &p2, double alpha, long mask)
{
  sla::Vec2d line_point;
  line_point = lerp(alpha, p1, p2);
  long l = bevel_1d(line_point) & mask;
  return (l==0?INSIDE:OUTSIDE);
}

/* Compute intersection of P1 --> P2 line segment with edge lines  */
/* Then test intersection point to see if it is on cube face       */
/* Consider only face planes in "outcode_diff"                     */
/* Note: Zero bits in "outcode_diff" means edge is outside of      */
bool segment_on_edge(const sla::Vec2d &p1, const sla::Vec2d &p2, long outcode_diff)
{
  if (0x01 & outcode_diff)
    if (point_on_edge(p1,p2,( .5-p1(0))/(p2(0)-p1(0)),0xE) == INSIDE) return INSIDE;
  if (0x02 & outcode_diff)
    if (point_on_edge(p1,p2,(-.5-p1(0))/(p2(0)-p1(0)),0xD) == INSIDE) return INSIDE;
  if (0x04 & outcode_diff)
    if (point_on_edge(p1,p2,( .5-p1(1))/(p2(1)-p1(1)),0xB) == INSIDE) return INSIDE;
  if (0x08 & outcode_diff)
    if (point_on_edge(p1,p2,(-.5-p1(1))/(p2(1)-p1(1)),0x7) == INSIDE) return INSIDE;
  return OUTSIDE;
}


// the main routine
// true if line t1,t2 is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
//
bool line_outside_of_rect(const sla::Vec2d &c, double s, const sla::Vec2d &t1, const sla::Vec2d &t2)
{

  long v1_test,v2_test;

  // First compare both points tih all four rectangle edges
  // If any point is inside the rectangle, return immediately!
  sla::Vec2d v1((t1(0)-c(0))/s, (t1(1)-c(1))/s);
  if (!(v1_test = bevel_1d(v1))) return INSIDE;
  sla::Vec2d v2((t2(0)-c(0))/s, (t2(1)-c(1))/s);
  if (!(v2_test = bevel_1d(v2))) return INSIDE;
  // If both points were outside of one or more edges,
  // return immediately with a trivial rejection!
  if ((v1_test & v2_test) != 0) return OUTSIDE;

  // Now do the same trivial rejection test for the four corner lines
  v1_test |= bevel_2d(v1) << 8;
  v2_test |= bevel_2d(v2) << 8;
  if ((v1_test & v2_test) != 0) return OUTSIDE;

  /* If point 1 and 2, as a pair, cannot be trivially rejected    */
  /* by the above tests, then see if the v1-->v2 segment          */
  /* intersects the rectangle.                                    */
  /* Pass to the intersection algorithm the "OR" of the outcode   */
  /* bits, so that only those rectangle edges which are spanned   */
  /* by each triangle edge need be tested.                        */
  if (segment_on_edge(v1,v2,v1_test|v2_test) == INSIDE) return INSIDE;

//  if (point_on_edge(v1,v2,( .5-v1(0))/(v2(0)-v1(0))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,(-.5-v1(0))/(v2(0)-v1(0))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,( .5-v1(1))/(v2(1)-v1(1))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,(-.5-v1(1))/(v2(1)-v1(1))) == INSIDE) return INSIDE;

//  /* No line touched the rectangle                                   */
//  /* We're done...there was no intersection.                         */
  return OUTSIDE;
}

// true if line p is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool point_outside_of_rect(const sla::Vec2d &c, double s, const sla::Vec2d &p)
{
  sla::Vec2d v1((p(0)-c(0))/s, (p(1)-c(1))/s);
  if (!bevel_1d(v1)) return INSIDE;
  return OUTSIDE;
}
