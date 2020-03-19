#ifndef INTERSECTION_PATH_H_
#define INTERSECTION_PATH_H_

#include <string>
#include <limits.h>
#include <set>
#include <map>
#ifdef HAVE_PROBT
#include <pl.h>

#include <aw_RndfGraph.h>
#include <aw_Vehicle.h>
#include <aw_VehicleManager.hpp>
#include <aw_MergeFeasabilityCheck.hpp>
#include <traffic_light_interface.h>
#include <obstaclePrediction.h>


namespace vlr {

class Topology;

class IntersectionPath {
	public:
	
	IntersectionPath(RndfVertex*, RndfVertex*);
	IntersectionPath(IntersectionPath*);
	virtual ~IntersectionPath();
	
	void setSegmentsIds(ObstaclePredictor::VertexSegmentMap);
	
	RndfVertex* entranceVertex() { return entrance_vertex_; }
	RndfVertex* exitVertex() { return exit_vertex_; }
	TRndfEdgeSet& edges() { return edges_; }
	
	
	protected:
	
	RndfVertex* entrance_vertex_;
	RndfVertex* exit_vertex_;
	TRndfEdgeSet edges_;
	uint32_t entrance_segment_id_;
	uint32_t exit_segment_id_;
	
  friend class IntersectionManager;
  friend class ObstaclePredictor;
	
};
// End Stephanie

} // namespace vlr
#endif

#endif /*INTERSECTIONMANAGER_H_*/
