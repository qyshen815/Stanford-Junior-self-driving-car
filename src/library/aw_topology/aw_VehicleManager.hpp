#ifndef AW_VEHICLE_MANAGER_HPP
#define AW_VEHICLE_MANAGER_HPP

#include <cmath>
#include <perception_interface.h>
#include <aw_timestamp.hpp>
#include <aw_RndfGraph.h>
#include <aw_Vehicle.h>
#include <aw_MergeFeasabilityCheck.hpp>

namespace vlr {

class Topology;

//--------------------------------------------------------
//             VehicleManager
//--------------------------------------------------------

class VehicleManager {
public:
  VehicleManager(Topology& topology);

  bool updateVehicles(PerceptionDynamicObstacle* dyn_obstacles, unsigned int num_dyn_obstacles, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res);
  std::map<int, Vehicle>& getVehicles(void) {return vehicle_map;}

protected:
  int add_vehicle(const PerceptionDynamicObstacle& vehicle, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res);
  void delete_vehicle(const int id);
  int update_vehicle(const PerceptionDynamicObstacle& vehicle, double offset_x, double offset_y);

  class VehicleState {
    public:
    double x,y,speed;
  	Timestamp time;
  	bool valid;
  	VehicleState() : x(0), y(0), time(), valid(false) { time.now(); };
  	VehicleState(double x, double y) : x(x), y(y), time(), valid(true) { time.now(); };
  	bool isMoving(const Vehicle& veh) {
  		// TODO: tune this constants
  		const double DELTA_D = 0.75;
  		const double DELTA_T = 5;
  		const double SPEED_THRESHOLD = 1.0;

  		bool isAtIntersection = false;
  		// in intersection
  		if (veh.edge() && veh.edge()->getIntersection()) {
  			isAtIntersection = true;
  		}
  		// at stopline
  		if (veh.edge() && veh.edge()->toVertex() && veh.edge()->toVertex()->isStopVertex() && veh.distToEnd() - veh.length()/2.0 < 4.0) {
  			isAtIntersection = true;
  		}
  		if (!valid || isAtIntersection
  				|| std::abs(veh.xMatchedFrom() - x) > DELTA_D
  				|| std::abs(veh.yMatchedFrom() - y) > DELTA_D
  				|| std::abs(veh.speed()) > SPEED_THRESHOLD) {
	  		x = veh.xMatchedFrom();
	  		y = veh.yMatchedFrom();
	  		speed = veh.speed();
	  		valid = true;
	  		time.now();
	  		return true;
  		} else {
  			Timestamp now;
  			now.now();
  			return now < time + DELTA_T;
  		}
  	};
  };
  std::map<int, VehicleState> state_map;

  void update_state(Vehicle& veh);
public:
  // mapping from internal id to vehicles
  std::map<int, Vehicle> vehicle_map; // this map holds all vehicle objects
  std::map<int, Vehicle*> moving_map; // this map holds all moving vehicles
  std::map<int, Vehicle*> blockage_map; // this map holds all "vehicles" classified as blockages

  Topology& topology;
  Vehicle robot;

  void setBlocked(int veh_id);

  void updateBlockages();

  bool isMoving(int veh_id);

  bool intersectsWithAnotherVehicle(const Vehicle& veh) const;
};


void block_adjacent_lanechange_edges(RoutePlanner::RndfEdge* edge);

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------

std::ostream& operator << (std::ostream& ostrm, const Vehicle& obj);

} // namespace vlr

#endif
