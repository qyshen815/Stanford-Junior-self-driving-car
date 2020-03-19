/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_ROUTESAMPLER_H
#define AW_ROUTESAMPLER_H

#include <aw_Topology.hpp>
#include <trajectory_points_interface.h>


namespace vlr {

//--------------------------------------------------------
//             RouteSampler
//--------------------------------------------------------

class RouteSampler
{
public:
	RouteSampler(Topology* top);
	~RouteSampler();

//	// samples the route beginnning at the current edge from the topology route
//	bool samplePoints(CurvePoints* curvepoints, double back_sampl_dist = -10., double length = 30., int point_anz = 15, double start_lateral_offset = 0., bool make_step = false, double dist_to_step = -1., double gap_length = 0., double end_lateral_offset = 0. );
//
//	static bool samplePoints(Route::RouteEdgeList::iterator edge_it, double dist_from_start, Route::RouteEdgeList* edges, CurvePoints* curvepoints, double back_sampl_dist = -10., double length = 30., int point_anz = 15, double start_lateral_offset = 0., bool make_step = false, double dist_to_step = -1., double gap_length = 0., double end_lateral_offset = 0. );

  bool sampleMission(std::vector<CurvePoint>& waypoints, std::vector<bool>& points_to_ignore);
  void vertexToCurvePoint(RndfVertex* v);


private:
	Topology* top_;
	RndfGraph* graph_;
	Route* route_;
	Route::RouteEdgeList* edges_;
	std::map<RndfEdge*, double> lastOffsets_;
//	CurvePoints oldCurvePoints_; // keep curve points from last step
  CurvePoint cp_;
};

} // namespace vlr

#endif // AW_ROUTESAMPLER_H
