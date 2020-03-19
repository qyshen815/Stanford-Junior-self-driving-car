#ifndef AW_RNDFSEARCH_H_
#define AW_RNDFSEARCH_H_

#include <sla.h>
#include <aw_laneQuadTree.h>

namespace vlr {

namespace rndf
{
class WayPoint;
class PerimeterPoint;
class Lane;
class TrafficLight;
class Crosswalk;

class RoadNetworkSearch {

public:
	RoadNetworkSearch(RoadNetwork* rndf);
	~RoadNetworkSearch();

	// methods for searching within waypoints

	//  // returns the closest waypoint to a point utm_x, utm_y
	//  rndf_waypoint_p closest_waypoint(double utm_x, double utm_y) const;
	//  // returns the closest waypoint and the distance to a point utm_x, utm_y
	//  void closest_waypoint(double utm_x, double utm_y, rndf_waypoint_p& waypoint, double& distance) const;

	// returns the closest waypoint to a point utm_x, utm_y
	WayPoint* closest_waypoint(double utm_x, double utm_y);

  // returns the closest perimeter point to a point utm_x, utm_y
  PerimeterPoint* closest_perimeterpoint(double utm_x, double utm_y);

  // returns the closest perimeter point to a point utm_x, utm_y
  TrafficLight* closestTrafficLight(double utm_x, double utm_y);

	// returns the closest Lane to a point utm_x, utm_y
	Lane* closest_lane(double utm_x, double utm_y) const;
	// returns the closest Lane and the distance to a point utm_x, utm_y
	void closest_lane(double utm_x, double utm_y, Lane*& l, double& distance) const;
	// returns true if point utm_x, utm_y is inside a Lane
	bool within_lane(double utm_x, double utm_y) const;

	LaneQuadTree* get_lane_quadtree(void) { return m_lanes; }

private:
	RoadNetwork*   m_rndf;
	LaneQuadTree*  m_lanes;
};

} // NAMESPACE rndf

} // namespace vlr

#endif /*RNDFSEARCH_H_*/
