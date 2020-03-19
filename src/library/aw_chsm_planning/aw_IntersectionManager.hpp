/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_INTERSECTIONMANAGER_H
#define AW_INTERSECTIONMANAGER_H

#include <string>
#include <limits.h>
#include <set>
#include <map>

#include <aw_RndfGraph.h>
#include <aw_Vehicle.h>
#include <aw_VehicleManager.hpp>
#include <aw_MergeFeasabilityCheck.hpp>
#include <aw_RndfGraphSearch.h>
#include <traffic_light_interface.h>

#include <obstaclePrediction.h>
#include <intersectionPath.hpp>
/*
 typedef enum SymbolType {PL_BINARY_TYPE=1, PL_INT_TYPE};

class plIntegerType {
  plIntegerType(int range_min, int range_max) : range_min_(range_min), range_max_(range_max), val_(range_min_) {

  }
  virtual ~plIntegerType() {}
  inline void rangeMin(int range_min) {range_min = range_min_;}
  inline int rangeMin() const {return range_min_;}

  inline void rangeMax(int range_max) {range_max = range_max_;}
  inline int rangeMax() const {return range_max_;}
  inline int val() const { return val_;}
  inline void val(int new_val) {
    if(new_val >= range_min_ && new_val <=range_max) {
      val_=new_val;
    }
    else {
      throw Exception("value out of range.");
    }
  }

private:
  int range_min_, range_max_;
  int val_;
};

class plSymbol {
public:
  plSymbol(std::string name, SymbolType type) : name_(name), type_(type), ival_(NULL) {
    if(type_ == PL_BINARY_TYPE) {
     ival_ = new plIntegerType(0,1);
    }
    else if(type_ == PL_INT_TYPE) {
     ival_ = new plIntegerType(INT_MIN, INT_MAX);
    }
  }
  plSymbol(std::string name, plIntegerType type) : name_(name), type_(PL_INT_TYPE), ival_(NULL) {
     ival_ = new plIntegerType(type.rangeMin(), type.rangeMax());
  }
  virtual ~plSymbol() {
    if(ival_) {delete ival_;}
  }

private:
  std::string name_;
  SymbolType type_;
  plIntegerType* ival_;
};

typedef double plProbValue;

class plDistributionTable {

};
class plProbTable {

};
class plUniform {

};
class plJointDistribution {

};
*/
namespace vlr {

class Topology;
class IntersectionPath;

class IntersectionManager
{
public:
	IntersectionManager(Topology& topology_, VehicleManager& vehicle_manager, double max_merge_speed, std::map<std::string, TrafficLightState*>& tl_states, pthread_mutex_t& intersection_predictor_mutex);
	~IntersectionManager();

	/*
	 * Checks for precedence on intersections with stop lines
	 * Precondition: vehicle stopped on stopline
	 */
	void stoppedOnStopline();

	//! returns true if vehicle is on an priority lane
	bool isOnPrio();

	//! generally returns true if the vehicle has the right of way at an intersection
	bool hasRightOfWay();

	//! returns true if vehicle has to stop at intersection
	bool hasToStop();

	bool isVehicleOnIntersectionInFront();

	bool isInfrontMergePoint();

	//! distance and vehicle pointer to next vehicle - uses multi-matching
	//std::pair<double, Vehicle*> distToVehicleOnWayThrouIntersection();

	const VehicleMap getVehiclesWithRow() const { return vehicles_with_row; }

	const VehicleMap getVehiclesWithPriority() const { return vehicles_with_priority; }

	RndfIntersection* getIntersection() { return intersection; };

	std::pair< bool, double > isPrioOppositeMergeAllowed(double velocity_desired);

	// is there activity on any prio lane? (do no normal recover)
	bool hasPrioMovement();

  inline ObstaclePredictor& obstaclePredictor() const {return *obstacle_predictor_;}

	// vehicles need to make 0.75m progress in 20 seconds
	static const double last_pose_pose_threshold; // [m]
	static const double last_pose_time_threshold;  // [s]
	static const double last_pose_time_diffusion;  // [s] random +-
	
protected:
	bool isMergeAllowed();

	void do_merge_check(MergeFeasabilityCheck::Entity& ego, GraphPlace& ego_place, bool on_prio, size_t& merge_allowed, const Vehicle& veh, const RndfEdge* edge);
	bool _hasPrioMovement(VehicleMap& map);

	std::pair< RndfEdge*, double > getMergingPointDistance();

	// get rid of non-moving obstacles
	void purgeFakeVehicles(VehicleMap& map, size_t& counter);

	// clears the timestamps used for purgeFakeVehicles
	void clearTimestamps(VehicleMap& map);

	Topology* topology_;
	RndfGraph* graph;
	VehicleManager* vehicle_manager_;
	RndfIntersection* intersection;
	MergeFeasabilityCheck* mfc;
	double max_merge_speed_;
	ObstaclePredictor* obstacle_predictor_;

	VehicleMap vehicles_with_row; /*!< row = right of way */
	VehicleMap vehicles_on_stopline;
	VehicleMap vehicles_with_priority;
	VehicleMap vehicles_on_opposite_prio;
	bool right_to_drive;
	VehiclePoseMap last_pose;
	VehiclePoseMap last_prio_pose;
	VehicleIdSet ignored_vehicles;

	//! indicates the direction of the turn (-1 left, 0 straight, 1 right)
	int turn_dir;
	std::map<std::string, TrafficLightState*>& traffic_light_states_;
};

} // namespace vlr

#endif /*INTERSECTIONMANAGER_H_*/
