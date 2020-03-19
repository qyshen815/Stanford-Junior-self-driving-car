/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_LANECHANGEMANAGER_H
#define AW_LANECHANGEMANAGER_H

#include <aw_RndfGraph.h>
#include <aw_Topology.hpp>
#include <aw_VehicleManager.hpp>
//#include <string>
//#include <set>
//#include <map>

namespace vlr {

class LaneChangeManager
{
	//	typedef int VehId;
	//  typedef std::set<Vehicle> VehicleSet;
	//  typedef std::set<VehId> VehicleIdSet;
	//  typedef std::map<VehId, Vehicle> VehicleMap;

	//states: enter, calc, exit

public:
	static void annotateLaneChanges(Topology* top); // requires planned mission. propagate planned lane changes as far as possible, so that vehicle has a bigger region to perform lane change
	// when executing the mission, this allows for an easy check to detect a required lane change as early as possible by checking for   currentAnnotatedRouteEdge.laneChange != AnnotatedRouteEdge::LC_NONE (not defined yet)

	LaneChangeManager(Topology* top, VehicleManager* vman);
	~LaneChangeManager();


protected:
	Topology* top;
	RndfGraph* graph;
	VehicleManager* vman;
};

} // namespace vlr

#endif // AW_LANECHANGEMANAGER_H
