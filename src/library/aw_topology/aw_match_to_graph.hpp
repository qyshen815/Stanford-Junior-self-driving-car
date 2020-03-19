/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef MATCH_TO_GRAPH_HPP
#define MATCH_TO_GRAPH_HPP

#include <map>
#include <limits>
#include <aw_CGAL.h>
#include "aw_RndfGraph.h"
#include "aw_RndfVertex.h"
#include "aw_RndfEdge.h"

using namespace CGAL_Geometry;

namespace vlr {

using namespace RoutePlanner;

const double PENALTY_METER_PER_DEGREE = 4.5/90;
const double REPLANNING_PENALTY_METER_PER_RAD = 5./M_PI_2;
const double THRESHOLD_OLD_POSITION = 15.;


RoutePlanner::RndfEdge* match_2_graph( const double x, const double y,
				       const double yaw, const std::map<int, RoutePlanner::RndfEdge*>& edges,
				       double& d,
				       double& cp_x,
				       double& cp_y,
				       double& offset_to_first_point,
				       int& sign,
				       const double penalty_meter_per_angle = PENALTY_METER_PER_DEGREE
				       );

//! matches a vehicle to multiple edges if there is no single best solution
/*! used to get better matching results in intersections. The result is a map of
 * 	mached edges and the offeset of the matched positions
 */
std::map< RoutePlanner::RndfEdge*, double >
						multi_match_2_graph( const Vehicle& veh, const std::set<RoutePlanner::RndfEdge*>& edges );

//! matches the \a vehicle to \edges and returns a sorted map with scores and the according edges
/*! the smaller the score the better the matching
 * */
std::map<double, RoutePlanner::RndfEdge*>
						getBestMatchings( const Vehicle& vehicle, const std::map<int, RoutePlanner::RndfEdge*>& edges, const double penalty_meter_per_degree = REPLANNING_PENALTY_METER_PER_RAD, double min_score = -1.0, double max_score = numeric_limits<double>::infinity() );

//! calculates the minimal delta angle (rad) between the \vehilce s yaw and the edge out of [0, PI]
double calcDeltaAngle(const Vehicle& vehicle, const RndfEdge* edge);


void closest_point_on_segment( const double p1_x, const double p1_y, const double p2_x, const double p2_y, const double p_x, double p_y, double& pc_x, double& pc_y, double& offset_to_first_point );

double dot_product( const double p1_x, const double p1_y, const double p2_x, const double p2_y );

} // namespace vlr

#endif
