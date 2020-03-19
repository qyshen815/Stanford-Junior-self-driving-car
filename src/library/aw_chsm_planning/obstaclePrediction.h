#ifndef OBSTACLEPREDICTION_H_
#define OBSTACLEPREDICTION_H_

#include <vector>
#include <trajectory_points_interface.h>
#include <aw_Vehicle.h>
#include <aw_Topology.hpp>

namespace vlr {

class ObstaclePrediction {
  public:
  std::vector<vlr::MovingBox> predicted_traj_;
  ObstaclePrediction() {}
  virtual ~ObstaclePrediction() {}
};

#ifndef HAVE_PROBT
class ObstaclePredictor {
  public:
  ObstaclePredictor(IntersectionManager*, Topology*, VehicleManager*, pthread_mutex_t&) {}
  virtual ~ObstaclePredictor() {}
  void update() {}
};

#else
#include <pl.h>
//#include <intersectionPath.hpp>
//#include "aw_IntersectionManager.hpp"
//#include <aw_Vehicle.h>


namespace vlr {
	
	//class ChsmPlanner;
	class IntersectionPath;
	class IntersectionManager;
	class VehicleManager;

	class ObstaclePredictor {
		public:
		ObstaclePredictor(IntersectionManager* intersection_manager, Topology* topology, VehicleManager* vehicle_manager, pthread_mutex_t& intersection_predictor_mutex);
		virtual ~ObstaclePredictor() {}
		void predict();

	  typedef std::set<IntersectionPath*> PathSet;

	  struct VehIdVertex {
	    VehId veh_id_;
	    RndfVertex* vertex_;
	    bool operator<(const VehIdVertex& A) const {
	      if (veh_id_!=A.veh_id_) {
	        return veh_id_<A.veh_id_;
	      }
	      else {
	        return vertex_<A.vertex_;
	      }
	    }
	  };
	  struct VehIdPath {
	    VehId veh_id_;
	    IntersectionPath* path_;
	    bool operator<(const VehIdPath& A) const {
	      if (veh_id_!=A.veh_id_) {
	        return veh_id_<A.veh_id_;
	      }
	      else {
	        return path_<A.path_;
	      }
	    }
	  };
	  struct VertexEdge {
	    RndfVertex* vertex_;
	    RndfEdge* edge_;
	    bool operator<(const VertexEdge& A) const {
	      if (vertex_!=A.vertex_) {
	        return vertex_<A.vertex_;
	      }
	      else {
	        return edge_<A.edge_;
	      }
	    }
	  };
	  struct VertexVertex {
	    RndfVertex* entrance_vertex_;
	    RndfVertex* exit_vertex_;
	    bool operator<(const VertexVertex& A) const {
	      if (entrance_vertex_!=A.entrance_vertex_) {
	        return entrance_vertex_<A.entrance_vertex_;
	      }
	      else {
	        return exit_vertex_<A.exit_vertex_;
	      }
	    }
	  };
	  struct VertexPath {
	    RndfVertex* vertex_;
	    IntersectionPath* path_;
	    bool operator<(const VertexPath& A) const {
	      if (path_!=A.path_) {
	        return path_<A.path_;
	      }
	      else {
	        return vertex_<A.vertex_;
	      }
	    }
	  };
	  struct VehIdTurnSignal {
	    VehId veh_id_;
	    uint32_t turn_signal_;
	    bool operator<(const VehIdTurnSignal& A) const {
	      if (veh_id_!=A.veh_id_) {
	        return veh_id_<A.veh_id_;
	      }
	      else {
	        return turn_signal_<A.turn_signal_;
	      }
	    }
	  };

	  struct EdgeProba {
	    RndfEdge* edge_;
	    double proba_;
	    double dist_;
	  };
	  struct LeftRightSegments {
	    uint32_t leftmost_segment_;
	    uint32_t rightmost_segment_;
	  };

	  struct AngleExtreme {
	    double angle_;
	    bool is_extreme_left_;
	    bool is_extreme_right_;
	  };

	  typedef std::map<IntersectionPath*, EdgeProba> PathEdgeProbaMap;
	  typedef std::map<RndfEdge*, double> EdgeProbaMap;
	  typedef std::map<RndfVertex*, double> VertexProbaMap;
	  typedef std::map<VehIdVertex, double> VehIdVertexProbaMap;
	  typedef std::map<IntersectionPath*, double> PathProbaMap;
	  typedef std::map<VehIdPath, double> VehIdPathProbaMap;
	  typedef std::map<uint32_t, double> TurnSignalProbaMap;
	  typedef std::map<VehIdTurnSignal, double> VehIdTurnSignalProbaMap;

	  typedef std::map<uint32_t, LeftRightSegments> LeftRightSegmentsMap;

	  typedef std::map<VertexVertex, AngleExtreme> VertexVertexAngleMap;

	  typedef std::map<VertexVertex, IntersectionPath*> VertexVertexPathMap;

	  typedef std::map<RndfVertex*, TRndfEdgeSet*> VertexEdgesMap;
	  typedef std::map<RndfEdge*, TRndfVertexSet*> EdgeVerticesMap;
	  typedef std::map<RndfVertex*, TRndfVertexSet*> VertexVerticesMap;

	  typedef std::map<VertexEdge, double> VertexEdgeProbaMap;
	  typedef std::map<VertexVertex, double> VertexVertexProbaMap;
	  typedef std::map<VertexPath, double> VertexPathProbaMap;

	  typedef std::map<uint32_t, TRndfVertexSet*> SegmentVerticesMap;
	  typedef std::map<RndfVertex*, uint32_t> VertexSegmentMap;

	  typedef std::map<uint32_t, RndfVertex*> IndexVertexMap;
	  typedef std::map<RndfVertex*, uint32_t> VertexIndexMap;
	  typedef std::map<uint32_t, VertexVertex> IndexVertexVertexMap;
	  typedef std::map<VertexVertex, uint32_t> VertexVertexIndexMap;
	  typedef std::map<VertexVertex, uint32_t> VertexVertexSideMap;

	  VehIdVertexProbaMap veh_id_entrance_vertex_proba_map_;
	  VehIdPathProbaMap veh_id_path_proba_map_;
	  VehIdTurnSignalProbaMap veh_id_turn_signal_proba_map_;
	  VehIdTurnSignalProbaMap veh_id_turn_signal_int_proba_map_;
	  VehIdVertexProbaMap veh_id_exit_vertex_proba_map_;

	  //VehIdVertexProbaMap veh_id_entrance_vertex_proba_map_old_;
	  //VehIdPathProbaMap veh_id_path_proba_map_old_;
	  //VehIdTurnSignalProbaMap veh_id_turn_signal_proba_map_old_;
	  //VehIdTurnSignalProbaMap veh_id_turn_signal_int_proba_map_old_;
	  //VehIdVertexProbaMap veh_id_exit_vertex_proba_map_old_;

	  VehicleIdSet relevant_vehicles_;
	  //VehicleIdSet non_relevant_vehicles_;
	  VehicleIdSet relevant_vehicles_in_intersection_;
	  VehicleIdSet relevant_vehicles_approaching_intersection_;

	  //VehicleIdSet relevant_vehicles_old_;
	  //VehicleIdSet relevant_vehicles_in_intersection_old_;
	  //VehicleIdSet relevant_vehicles_approaching_intersection_old_;

	  void update();
	  bool isAllWayStopIntersection() { return is_all_way_stop_intersection_; }

		private:
	  inline uint32_t subToIndex(uint32_t i, uint32_t size_i, uint32_t j, uint32_t k, uint32_t size_k, uint32_t l, uint32_t size_l) {
	    return i + size_i * l + size_i * size_l * k + size_i * size_l * size_k * j;
	  }

	  inline uint32_t subToIndex(uint32_t i, uint32_t size_i, uint32_t j, uint32_t k, uint32_t size_k) {
	    return i + size_i * k + size_i * size_k * j;
	  }

	  inline uint32_t subToIndex(uint32_t i, uint32_t size_i, uint32_t j) {
	    return i + size_i * j;
	  }

	  void setIntersectionVariables();
    void determineRelevantVehicles();
    void updateVehiclesInIntersection();
    void updateVehiclesApproachingIntersection();
	  void createSegmentsIds();
	  void findExitVerticesOfVirtualEdge(RndfEdge*, TRndfVertexSet*);
	  void findEntranceVerticesOfVirtualEdge(RndfEdge*, TRndfVertexSet*, bool);
	  void findReachableVerticesFromEntranceVertex();
	  void findVerticesLeadingToExitVertices();
	  void vehicleHeadingToNextIntersection(Vehicle, RndfEdge*, bool*, uint32_t*);
	  double computeProbaVehicleOnEdge(Vehicle*, RndfEdge*);
	  //double computeWeightEntrance(double, double, double, double);
	  void insertAdjacentEdges(Vehicle*, RndfEdge*, EdgeProbaMap*, uint32_t);
	  void insertAdjacentEdgesInIntersection(Vehicle*, RndfEdge*, EdgeProbaMap*);
	  double distanceBetweenPositions(double, double, double, double);
	  void findReachableVerticesFromEdge(RndfEdge*, TRndfVertexSet*);
	  void findReachableVerticesFromEdges(EdgeProbaMap, EdgeVerticesMap*);
	  void findEdgesLeadingToVertices(EdgeVerticesMap, VertexEdgesMap*);
	  void computeDistributionEntranceVertices(VertexProbaMap*, EdgeProbaMap, VertexEdgesMap);
	  void createPath(IntersectionPath*, PathSet*, TRndfVertexSet);
	  double angleBetweenEdges(RndfEdge*, RndfEdge*);
	  bool areInSameSegment(RndfEdge*, RndfEdge*, TRndfEdgeSet*, uint32_t*);
	  double gaussian_func(double, double, double);
	  double gaussian_func2(double, double, double, double, double, double);

	  // Temporary
	  void computeDistributionExitVertices(VertexProbaMap*, PathEdgeProbaMap);
	  void computeDistributionExitVertices(VertexProbaMap*, VertexProbaMap);

	  // Variables related to the intersection
	  uint32_t nr_possible_segments_;
	  uint32_t nr_possible_entrance_segments_;
	  uint32_t nr_possible_exit_lanes_;
	  uint32_t nr_possible_entrance_lanes_;
	  uint32_t nr_possible_paths_;
	  uint32_t nr_possible_turn_signals_;
	  uint32_t nr_possible_confusion_states_;
	  bool is_all_way_stop_intersection_;
	  TRndfVertexSet entrance_vertices_;
	  TRndfVertexSet exit_vertices_;
	  VertexVertexPathMap intersection_paths_;
	  VertexVerticesMap exit_vertex_entrance_vertices_map_;
	  VertexVerticesMap entrance_vertex_exit_vertices_map_;
	  IndexVertexMap index_entrance_map_;
	  VertexIndexMap entrance_index_map_;
	  IndexVertexMap index_exit_map_;
	  VertexIndexMap exit_index_map_;
	  IndexVertexVertexMap index_path_map_;
	  VertexVertexIndexMap path_index_map_;
	  SegmentVerticesMap segment_vertices_map_;
	  VertexSegmentMap vertex_segment_map_;
	  VertexVertexAngleMap entrance_exit_angle_map_;
	  VertexVertexSideMap side_vertices_in_same_segment_;

	  // Variables related to the Bayesian network

	  double proba_wrong_lane_;
	  double proba_wrong_segment_;
	  //double proba_max_wrong_path_;
	  //double proba_choose_incorrect_turn_signal_consistent_;
	  //double proba_choose_no_turn_signal_consistent_;
	  //double proba_choose_incorrect_turn_signal_inconsistent_;
	  //double proba_choose_no_turn_signal_inconsistent_;
	  //double proba_max_choose_correct_turn_signal_multiple_consistent_extreme_;
	  //double proba_max_choose_correct_turn_signal_unique_consistent_extreme_;
	  //double proba_max_choose_correct_turn_signal_inconsistent_extreme_;
	  //double proba_choose_correct_turn_signal_middle_;
	  //double proba_choose_incorrect_turn_signal_middle_;
	  //double proba_max_choose_correct_turn_signal_int_;
	  //double proba_choose_incorrect_turn_signal_int_;
	  ////double proba_choose_incorrect_turn_signal_int_;
	  ////double proba_choose_no_turn_signal_int_;
	  //double proba_choose_incorrect_turn_signal_extreme_;
	  //double proba_min_choose_correct_turn_signal_;
	  double proba_min_choose_correct_turn_signal_;
	  double proba_choose_incorrect_turn_signal_extreme_;
	  double proba_max_choose_correct_turn_signal_multiple_consistent_extreme_;
	  double proba_max_choose_correct_turn_signal_unique_consistent_extreme_;
	  double proba_choose_correct_turn_signal_consistent_middle_;
	  double proba_choose_incorrect_turn_signal_consistent_middle_;
	  double proba_choose_correct_turn_signal_inconsistent_;
	  double proba_choose_incorrect_turn_signal_inconsistent_;
	  double proba_max_choose_correct_turn_signal_int_;
	  double proba_choose_incorrect_turn_signal_int_;

	  plSymbol bn_entrance_;
	  plSymbol bn_exit_;
	  plSymbol bn_segment_;
	  plSymbol bn_wrong_lane_;
	  plSymbol bn_wrong_segment_;
	  plSymbol bn_path_;
	  plSymbol bn_turn_signal_;
	  plSymbol bn_turn_signal_int_;
	  plSymbol bn_coherence_entrance_;
	  plSymbol bn_coherence_turn_signal_;
	  plSymbol bn_coherence_turn_signal_int_;
	  plSymbol bn_coherence_path_;
	  plSymbol bn_soft_evidence_entrance_;
	  plSymbol bn_soft_evidence_turn_signal_;
	  plSymbol bn_soft_evidence_turn_signal_int_;
	  plSymbol bn_soft_evidence_path_;
	  plDistributionTable P_coherence_entrance_;
	  plDistributionTable P_coherence_turn_signal_;
	  plDistributionTable P_coherence_turn_signal_int_;
	  plDistributionTable P_coherence_path_;
	  plProbTable P_soft_evidence_entrance_;
	  plProbTable P_soft_evidence_turn_signal_;
	  plProbTable P_soft_evidence_turn_signal_int_;
	  plProbTable P_soft_evidence_path_;
	  plUniform P_exit_;
	  plProbTable P_wrong_lane_;
	  plProbTable P_wrong_segment_;
	  plDistributionTable P_segment_;
	  plDistributionTable P_entrance_;
	  plDistributionTable P_path_;
	  plDistributionTable P_turn_signal_;
	  plDistributionTable P_turn_signal_int_;
	  plJointDistribution jd_;

		IntersectionManager* intersection_manager_;
		Topology* topology_;
		VehicleManager* vehicle_manager_;
		RndfIntersection* intersection_;

		pthread_mutex_t& intersection_predictor_mutex_;
	};

#endif
} // namespace vlr

#endif // OBSTACLEPREDICTION_H_

