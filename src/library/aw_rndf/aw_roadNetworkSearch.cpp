#define DEBUG_LEVEL 0

//== INCLUDES =================================================================
#include <float.h>
#include <sla.h>
#include "aw_roadNetworkSearch.h"
#include "aw_lane.h"
#include "aw_perimeterPoint.h"
#include "aw_roadNetwork.h"

//== NAMESPACES ===============================================================
using namespace std;

namespace vlr {

namespace rndf {

//== IMPLEMENTATION ==========================================================
RoadNetworkSearch::RoadNetworkSearch(RoadNetwork* rndf) : m_rndf(rndf)
{
	m_lanes = new LaneQuadTree(rndf,1.0);
}

RoadNetworkSearch::~RoadNetworkSearch()
{
	delete m_lanes;
}

rndf::Lane*
RoadNetworkSearch::closest_lane(double utm_x, double utm_y) const
{
	Lane* l;
	double distance = DBL_MAX;
	closest_lane(utm_x,utm_y,l,distance);
	return l;
}

void
RoadNetworkSearch::closest_lane(double utm_x, double utm_y, Lane*& l, double& distance) const
{
	LaneQuadTree::element_p closest_element = NULL;
	sla::Vec2d p(utm_x,utm_y);
	sla::Vec2d closest_point(DBL_MAX);
	distance = DBL_MAX;
	l = NULL;
	if(!m_lanes->search_nearest_neighbor(p, closest_element, distance, closest_point))
		if(DEBUG_LEVEL > 0)
			fprintf(stderr, "BUG!!! RNDF NN SEARCH RETURNED FALSE!\n");
	if(closest_element != NULL && closest_element->l != NULL)
		l = closest_element->l;
	else if(DEBUG_LEVEL > 0)
		fprintf(stderr, "BUG!!! CLOSEST ELEMENT / LANE IS NULL!\n");
}

bool
RoadNetworkSearch::within_lane(double utm_x, double utm_y) const
{
	Lane* l = NULL;
	double distance;
	closest_lane(utm_x, utm_y,l,distance);
	if(l == NULL) {
		if(DEBUG_LEVEL > 0)
			fprintf(stderr, "BUG!!!! LANE IS NULL!\n");
		return false;
	}
	distance = sqrt(distance);
	if(distance > 0.5*l->getLaneWidth() + 0.5)
		return false;
	else
		return true;
}


WayPoint*
RoadNetworkSearch::closest_waypoint(double utm_x, double utm_y)
{
	double min_dist2 = DBL_MAX;
	WayPoint* w = NULL;

	const TWayPointMap& waypoints = m_rndf->wayPoints();
	TWayPointMap::const_iterator it, it_end;

	for(it=waypoints.begin(),it_end=waypoints.end();it!=it_end;++it) {
		double x = utm_x-(*it).second->utm_x();
		double y = utm_y-(*it).second->utm_y();
		double dist2 = x*x + y*y;
		if(dist2<min_dist2) {
			min_dist2 = dist2;
			w = (*it).second;
		}

	}

	return w;
}

PerimeterPoint* RoadNetworkSearch::closest_perimeterpoint(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  PerimeterPoint* p = NULL;

  const TPerimeterPointMap& perimeterpoints = m_rndf->perimeterPoints();
  TPerimeterPointMap::const_iterator it, it_end;

  for (it = perimeterpoints.begin(), it_end = perimeterpoints.end(); it != it_end; ++it) {
    double x = utm_x - (*it).second->utm_x();
    double y = utm_y - (*it).second->utm_y();
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      p = (*it).second;
    }
  }

  return p;
}

TrafficLight* RoadNetworkSearch::closestTrafficLight(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  TrafficLight* tl = NULL;

  const TTrafficLightMap& traffic_lights = m_rndf->trafficLights();
  TTrafficLightMap::const_iterator it, it_end;

  for (it = traffic_lights.begin(), it_end = traffic_lights.end(); it != it_end; ++it) {
    double x = utm_x - (*it).second->utm_x();
    double y = utm_y - (*it).second->utm_y();
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      tl = (*it).second;
    }
  }

  return tl;
}

} // namespace rndf

} // namespace vlr

