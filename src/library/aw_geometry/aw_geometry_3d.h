/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_GEOMETRY_3D_H
#define AW_GEOMETRY_3D_H

#include <sla.h>

namespace vlr {

typedef struct {
  double x;
  double y;
  double z;
} kogmo_point_3D_t;

// two vectors, a and b, starting from c
float dot(const sla::Vec3f &a, const sla::Vec3f &b);
// two vectors, a and b
sla::Vec3f cross(const sla::Vec3f &a, const sla::Vec3f &b);
// two vectors, a and b, starting from c
sla::Vec3f cross(const sla::Vec3f &a, const sla::Vec3f &b, const sla::Vec3f &c);
float dist2(const sla::Vec3f &a, const sla::Vec3f &b);
float dist(const sla::Vec3f &a, const sla::Vec3f &b);
// linear interpolation
sla::Vec3f lerp(float t, const sla::Vec3f &a, const sla::Vec3f &b);
// is the ball centered at b with radius r
// fully within the box centered at bc, with radius br?
bool ball_within_bounds(const sla::Vec3f &b, float r,
                        const sla::Vec3f &bc, float br);
// is the ball centered at b with radius r
// fully within the box centered from min to max?
bool ball_within_bounds(const sla::Vec3f &b, float r,
                        const sla::Vec3f &min,
                        const sla::Vec3f &max);
// does the ball centered at b, with radius r,
// intersect the box centered at bc, with radius br?
bool bounds_overlap_ball(const sla::Vec3f &b, float r,
                         const sla::Vec3f &bc, float br);
bool bounds_overlap_ball(const sla::Vec3f &b, float r,
                         const sla::Vec3f &min, const sla::Vec3f &max);
// calculate barycentric coordinates of the point p
// (already on the triangle plane) with normal vector n
// and two edge vectors v1 and v2,
// starting from a common vertex t0
void bary_fast(const sla::Vec3f& p, const sla::Vec3f& n,
               const sla::Vec3f &t0, const sla::Vec3f& v1,
               const sla::Vec3f& v2, float &b1, float &b2, float &b3);
bool closer_on_lineseg(const sla::Vec3f &x, sla::Vec3f &cp, const sla::Vec3f &a,
                       const sla::Vec3f &b, float &d2);
void distance_point_line(const sla::Vec3f &x, const sla::Vec3f &a,
                         const sla::Vec3f &b, float &d2, sla::Vec3f &cp);
void distance_point_tri(const sla::Vec3f &x, const sla::Vec3f &t1,
                        const sla::Vec3f &t2, const sla::Vec3f &t3,
                        float &d2, sla::Vec3f &cp);
bool closer_on_tri(const sla::Vec3f &x, sla::Vec3f &cp,
                   const sla::Vec3f &t1, const sla::Vec3f &t2,
                   const sla::Vec3f &t3, float &d2);
// calculate the intersection of a line going through p
// to direction dir with a plane spanned by t1,t2,t3
// (modified from Graphics Gems, p.299)
bool line_plane_X(const sla::Vec3f& p, const sla::Vec3f& dir,
                  const sla::Vec3f& t1, const sla::Vec3f& t2,
                  const sla::Vec3f& t3,
                  sla::Vec3f &x, float &dist);
bool line_plane_X(const sla::Vec3f& p, const sla::Vec3f& dir,
                  const sla::Vec3f& nrm, float d, sla::Vec3f &x, float &dist);
// calculate barycentric coordinates of the point p
// on triangle t1 t2 t3
void bary(const sla::Vec3f& p,
          const sla::Vec3f& t1, const sla::Vec3f& t2, const sla::Vec3f& t3,
          float &b1, float &b2, float &b3);
// calculate barycentric coordinates for the intersection of
// a line starting from p, going to direction dir, and the plane
// of the triangle t1 t2 t3
bool bary(const sla::Vec3f& p, const sla::Vec3f& dir,
          const sla::Vec3f& t1, const sla::Vec3f& t2, const sla::Vec3f& t3,
          float &b1, float &b2, float &b3);
// calculate the intersection of a line starting from p,
// going to direction dir, and the triangle t1 t2 t3
bool line_tri_X(const sla::Vec3f& p, const sla::Vec3f& dir,
                const sla::Vec3f& t1, const sla::Vec3f& t2, const sla::Vec3f& t3,
                sla::Vec3f& x, float& d);

} // namespace vlr

#endif // AW_GEOMETRY_3D_H
