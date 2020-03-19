/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDF_RNDFGL_H
#define AW_RNDF_RNDFGL_H

#include <aw_CGAL.h>

namespace vlr {

namespace rndf {

class RoadNetwork;
class Lane;
class CheckPoint;
class intersection;

//! draws a checkpoint
void draw_checkpoint(double x, double y, double r, int num, double blend);
//! draws a stoppoint
void draw_stoppoint(double x, double y, double r, double blend);

//! draw the Lane boundary
void draw_lane_boundary(Lane *l, double origin_x, double origin_y, int left, double blend);

//! draw Lane connections
void draw_lane_connections(const rndf::Lane& l1, double center_x, double center_y, double blend);

void draw_lane_background(double x, double y, double theta, double w, double l);
void draw_lane_background(CGAL_Geometry::Point_2 p1, CGAL_Geometry::Point_2 p2, CGAL_Geometry::Vector_2 dir1, CGAL_Geometry::Vector_2 dir2, double w1, double w2);

void get_exitpoint_params(Exit* e, CGAL_Geometry::Point_2& point_from, CGAL_Geometry::Point_2& point_to, CGAL_Geometry::Vector_2& dir_from, CGAL_Geometry::Vector_2& dir_to, double& width);

void draw_arrowhead(double x, double y, double angle);
void draw_kturns(const LaneSegment* l, double center_x, double center_y, double blend);
void draw_TypesLaneSegment(const Lane& lane1, double center_x, double center_y, double blend);

  //! draw intersections
void draw_intersection(const Intersection& i, double center_x, double center_y, double blend);
std::vector<CGAL_Geometry::Point_2> calc_intermediate_points(const CGAL_Geometry::Point_2 point_from, const CGAL_Geometry::Point_2 point_to, const CGAL_Geometry::Vector_2 dir_from,
    const CGAL_Geometry::Vector_2 dir_to);

}  // namespace rndf

} // namespace vlr

#endif // AW_RNDF_RNDFGL_H_


