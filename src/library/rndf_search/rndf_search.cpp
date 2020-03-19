/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * file .......: rndf_lane_quadtree.cpp
 * authors ....: Benjamin Pitzer
 * organization: Stanford University
 * creation ...: 02/15/2007
 * revisions ..: $Id: rndf_search.cpp,v 1.7 2007-04-16 04:51:22 pitzer Exp $
---------------------------------------------------------------------*/
#define DEBUG_LEVEL 0

//== INCLUDES =================================================================
#include <string.h>
#include "rndf_search.h"
#include "geometry_2d.h"

//== NAMESPACES ===============================================================
using namespace std;

namespace dgc {

//== IMPLEMENTATION ==========================================================
rndf_search::rndf_search(rndf_file* rndf)
{
  m_lanes = new rndf_lane_quadtree(rndf,1.0);
}

rndf_search::~rndf_search()
{
  delete m_lanes;
}

rndf_lane_p
rndf_search::closest_lane(double utm_x, double utm_y) const
{
  rndf_lane_p lane;
  double distance = MAXDOUBLE;
  closest_lane(utm_x,utm_y,lane,distance);
  return lane;
}

void
rndf_search::closest_lane(double utm_x, double utm_y, rndf_lane_p& lane, double& distance) const
{
  rndf_lane_quadtree::element_p element;
  closest_element(utm_x, utm_y, element, distance);
  if(element != NULL && element->l != NULL)
    lane = element->l;
  else if(DEBUG_LEVEL > 0)
    fprintf(stderr, "BUG!!! CLOSEST ELEMENT / LANE IS NULL!\n");
}

rndf_waypoint_p
rndf_search::closest_waypoint(double utm_x, double utm_y) const
{
  rndf_waypoint_p waypoint;
  double distance = MAXDOUBLE;
  closest_waypoint(utm_x, utm_y, waypoint, distance);
  return waypoint;
}

void
rndf_search::closest_waypoint(double utm_x, double utm_y, rndf_waypoint_p& waypoint, double& distance) const
{
  rndf_lane_quadtree::element_p element;
  closest_element(utm_x, utm_y, element, distance);
  if(element != NULL && element->w1 != NULL) {
    sla::Vec2d p(utm_x,utm_y);
    sla::Vec2d a(element->w1->utm_x(),element->w1->utm_y());
    sla::Vec2d b(element->w2->utm_x(),element->w2->utm_y());
    if (dist2(p,a) < dist2(p,b))
      waypoint = element->w1;
    else
      waypoint = element->w2;
  } else if(DEBUG_LEVEL > 0)
    fprintf(stderr, "BUG!!! CLOSEST ELEMENT / WAYPOINT IS NULL!\n");
}

void inline
rndf_search::closest_element(double utm_x, double utm_y, rndf_lane_quadtree::element_p& element, double& distance) const
{
  element = NULL;
  sla::Vec2d p(utm_x,utm_y);
  sla::Vec2d closest_point(MAXDOUBLE);
  distance = MAXDOUBLE;
  m_lanes->search_nearest_neighbor(p, element, distance, closest_point);
}

bool
rndf_search::within_lane(double utm_x, double utm_y) const
{
  rndf_lane_p lane = NULL;
  double distance;
  closest_lane(utm_x, utm_y,lane,distance);
  if(lane == NULL) {
    if(DEBUG_LEVEL > 0)
      fprintf(stderr, "BUG!!!! LANE IS NULL!\n");
    return false;
  }
  distance = sqrt(distance);
  if(distance > 0.55*lane->width())
    return false;
  else
    return true;
}

//=============================================================================
} // namespace dgc
//=============================================================================
