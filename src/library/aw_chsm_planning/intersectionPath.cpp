#include <cmath>
#include <sys/types.h>
#include <algorithm>
#include <aw_CGAL.h>
#include <aw_Topology.hpp>
#include <aw_Topology.hpp>
//#include <aw_roadNetwork.h>
//#include <aw_roadNetworkSearch.h>

#include <aw_ChsmPlanner.hpp>
#include <intersectionPath.hpp>

#ifdef HAVE_PROBT

using namespace std;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[IntersectionManager] " << str << endl;
#define TRACE(str)


IntersectionPath::IntersectionPath(IntersectionPath* intersection_path) {
  entrance_vertex_ = intersection_path->entrance_vertex_;
  exit_vertex_ = intersection_path->exit_vertex_;
  edges_ = intersection_path->edges_;
  //entrance_segment_id_ = intersection_path->entrance_segment_id_;
  //exit_segment_id_ = intersection_path->exit_segment_id_;
  //turning_angle_ = intersection_path->turning_angle_;
  //is_leftmost_turn_ = intersection_path->is_leftmost_turn_;
  //is_rightmost_turn_ = intersection_path->is_rightmost_turn_;
}



IntersectionPath::~IntersectionPath() {
    delete[] entrance_vertex_;
    delete[] exit_vertex_;
}

//void IntersectionPath::setSegmentsIds(IntersectionManager::VertexSegmentMap vertex_segment_map) {
    
    //entrance_segment_id_ = vertex_segment_map[entrance_vertex_];
    //exit_segment_id_ = vertex_segment_map[exit_vertex_];
    
//}

IntersectionPath::IntersectionPath(RndfVertex* entrance_vertex, RndfVertex* exit_vertex) {
	entrance_vertex_ = entrance_vertex;
	exit_vertex_ = exit_vertex;
	//entrance_segment_id_ = 1000000;
	//exit_segment_id_ = 1000000;
	//turning_angle_ = 0.0;
	//is_leftmost_turn_ = false;
	//is_rightmost_turn_ = false;
}

} // namespace vlr
#endif
