/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * file .......: rndf_search.h
 * authors ....: Benjamin Pitzer
 * organization: Stanford University
 * creation ...: 02/17/2007
 * revisions ..: $Id: rndf_search.h,v 1.3 2007-04-16 04:51:22 pitzer Exp $
---------------------------------------------------------------------*/
#ifndef RNDF_SEARCH_H_
#define RNDF_SEARCH_H_

//== INCLUDES =========================================================
// #include <sla.h>
#include <rndf.h>
#include <rndf_lane_quadtree.h>
#include <rndf_waypoint_quadtree.h>
#include "rndf_types.h"

//== NAMESPACES =======================================================
namespace dgc {

//== CLASS DEFINITION =================================================
class rndf_search
{
public:
  rndf_search(rndf_file* rndf);
  ~rndf_search(void);

  // methods for searching within waypoints

  // returns the closest waypoint to a point utm_x, utm_y
  rndf_waypoint_p closest_waypoint(double utm_x, double utm_y) const;
  // returns the closest waypoint and the distance to a point utm_x, utm_y
  void closest_waypoint(double utm_x, double utm_y, rndf_waypoint_p& waypoint, double& distance) const;

  // returns the closest lane to a point utm_x, utm_y
  rndf_lane_p closest_lane(double utm_x, double utm_y) const;
  // returns the closest lane and the distance to a point utm_x, utm_y
  void closest_lane(double utm_x, double utm_y, rndf_lane_p& lane, double& distance) const;
  // returns true if point utm_x, utm_y is inside a lane
  bool within_lane(double utm_x, double utm_y) const;

  rndf_lane_quadtree* get_lane_quadtree(void) { return m_lanes; }
  rndf_waypoint_quadtree* get_waypoint_quadtree(void) { return m_waypoints; }

private:
  rndf_file*                        m_rndf;
  rndf_lane_quadtree*               m_lanes;
  rndf_waypoint_quadtree*           m_waypoints;

  void inline closest_element(double utm_x, double utm_y, rndf_lane_quadtree::element_p& element, double& distance) const;
};

//=====================================================================
} // NAMESPACE dgc
//=====================================================================
//=====================================================================
#endif /*RNDF_SEARCH_H_*/
//=====================================================================
