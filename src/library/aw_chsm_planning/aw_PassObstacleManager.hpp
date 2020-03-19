/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_PASSOBSTACLEMANAGER_HPP
#define AW_PASSOBSTACLEMANAGER_HPP

#include <pose.h>
#include <aw_Vehicle.h>
#include <aw_RndfGraph.h>
#include <aw_MergeFeasabilityCheck.hpp>
#include <aw_Route.h>

namespace vlr {

class Topology;
class VehicleManager;

class PassObstacleManager
{
public:
	PassObstacleManager(Topology* top);
	virtual ~PassObstacleManager();

	bool isInited() const { return inited; };

	// init manager with obstacle to pass
	void setObstacle(Vehicle* veh);

	// get called every cycle, updates internals
	void update();

	// true if there is a neigbor lane
	bool isPassPossible();

	// true if passing is allowed
	bool mayPass(Pose pose, double speed, double desired_speed);

	// generates trajectory for passing an obstacle
	void generateTrajectory(Pose pose, std::map<double, CurvePoint>& center_line, double front_sample_length, double back_sample_length);

	double getFollowingSpeed();

private:
	enum IterationDirection {
		INVALID,
		FORWARD,
		BACKWARD
	};
	bool inited;
	Topology* top;
	Vehicle obstacle; // local copy
	VehicleManager* vman;
	RoutePlanner::RndfEdge* passEdge;
	IterationDirection edgeIterationDirection;
	RoutePlanner::RndfGraph::EdgeMap edges_map;
	RoutePlanner::Route::RouteEdgeList edges_list;
	MergeFeasabilityCheck* mfc;

	void cleanup();
};

} // namespace vlr

#endif // AW_PASSOBSTACLEMANAGER_HPP
