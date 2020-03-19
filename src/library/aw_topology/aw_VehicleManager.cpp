/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_VehicleManager.hpp"
#include "aw_Vehicle.h"
#include "aw_Topology.hpp"
#include "aw_match_to_graph.hpp"
#include "aw_RndfGraphSearch.h"

namespace vlr {

using namespace RoutePlanner;

#undef TRACE

#define TRACE(str) std::cout << "[VehicleManager] "<< str << std::endl

//-----------------------------------------------------------------------------
//             VehicleManager
//-----------------------------------------------------------------------------

VehicleManager::VehicleManager(Topology& topology) :
	topology(topology) {
}

int VehicleManager::add_vehicle(const PerceptionDynamicObstacle& vehicle, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res)
{
	if (isfinite(vehicle.x) && isfinite(vehicle.y) && isfinite(vehicle.direction) && isfinite(vehicle.length) && isfinite(vehicle.width)) {
		vehicle_map[ vehicle.id ] = Vehicle(vehicle, offset_x, offset_y);
		Vehicle& veh = vehicle_map[ vehicle.id ];
		veh.match_to_graph( topology.complete_graph->edgeMap );

      // TODO: Until ghost cars are gone, only consider those that are already on lanes
		if(veh.distToEdge(veh.edge()) > 0.5*veh.edge()->getLaneWidth()) {
		  delete_vehicle(vehicle.id);
		  return -1;
		}

		veh.rematch_in_intersection();
		veh.vehicleManager(this);

		update_state(vehicle_map[ vehicle.id ]);
		veh.predict(t0, checked_horizon, time_sample_res);
	}
	return vehicle.id;
}

void VehicleManager::delete_vehicle(const int id)
{
	// calls vehicle destructor
	if( vehicle_map[id].edge() ) {
		vehicle_map[id].edge()->vehicles_on_edge.erase( id );
		//		if (vehicle_map[id].isBlockage) {
		//			vehicle_map[id].edge->setBlocked(false);
		//		}
	}

	if ( ! vehicle_map[id].edges().empty() ) {
		for (map< RndfEdge*, double >::iterator it = vehicle_map[id].edges().begin(); it != vehicle_map[id].edges().end(); ++it)
			it->first->vehicles_on_edge.erase( id );
	}

	vehicle_map[id].vehicleManager(NULL);
	vehicle_map.erase( id );
	moving_map.erase( id );
	blockage_map.erase ( id );
}

int VehicleManager::update_vehicle(const PerceptionDynamicObstacle& vehicle, double offset_x, double offset_y)
{
	if (isfinite(vehicle.x) && isfinite(vehicle.y) && isfinite(vehicle.direction) && isfinite(vehicle.length) && isfinite(vehicle.width)) {
		vehicle_map[ vehicle.id ].update( vehicle, offset_x, offset_y);
		vehicle_map[ vehicle.id ].match_to_graph( topology.complete_graph->edgeMap );
		vehicle_map[ vehicle.id ].rematch_in_intersection();
		update_state(vehicle_map[ vehicle.id ]);
	}
	return vehicle.id;
}

void VehicleManager::update_state(Vehicle& veh)
{
	if (state_map[ veh.id() ].isMoving(veh)) {
		moving_map[ veh.id() ] = &veh;
		blockage_map.erase ( veh.id() );
		veh.isBlocking(false);
		//if (veh.edge) veh.edge->setBlocked(false);
	} else {
		//		blockage_map[ veh.id ] = &veh;
		//		moving_map.erase ( veh.id );
		//		veh.isBlockage = true;
		//if (veh.edge) veh.edge->setBlocked(true);
	}
}

bool VehicleManager::updateVehicles(PerceptionDynamicObstacle* dyn_obstacles, unsigned int num_dyn_obstacles, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res)
{
  // TODO: Only update/add/delete necessary vehicles
  while(!vehicle_map.empty()) {
    delete_vehicle((*vehicle_map.begin()).second.id());
  }

  for(unsigned int i=0; i<num_dyn_obstacles; i++) {
    if(dyn_obstacles[i].obstacleType == OBSTACLE_PEDESTRIAN) {continue;}
    if(dyn_obstacles[i].width < 1 && dyn_obstacles[i].length < 1) {continue;} // TODO: remove temp size filter

//    if(dyn_obstacles[i].obstacleType == OBSTACLE_PEDESTRIAN ||
//       dyn_obstacles[i].obstacleType == OBSTACLE_UNKNOWN) {continue;} // did not seem to work yet..
    dyn_obstacles[i].id = i; // TODO: id's are messed up (at least) in simulation
    add_vehicle(dyn_obstacles[i], offset_x, offset_y, t0, checked_horizon, time_sample_res);
  }

	return true;
}

void VehicleManager::setBlocked(int veh_id)
{
	if (vehicle_map.find(veh_id)!= vehicle_map.end()) {
		blockage_map[ veh_id ] = &vehicle_map[veh_id];
		moving_map.erase ( veh_id );
		vehicle_map[veh_id].isBlocking(true);

		Vehicle* veh = &vehicle_map[veh_id];
		if (veh->edge() && ! veh->edge()->isZoneEdge()) veh->edge()->setBlocked();
		for (map< RndfEdge*, double >::iterator it = veh->edges().begin(); it != veh->edges().end(); ++it) {
			RndfEdge* blocked_edge = it->first;
			if ( ! blocked_edge->isZoneEdge() ) blocked_edge->setBlocked();
		}
	}
}

void VehicleManager::updateBlockages()
{
	// Edges von blockierenden Fahrzeugen blockieren
	std::map<int, Vehicle*>::const_iterator iter = blockage_map.begin();
	std::map<int, Vehicle*>::const_iterator end = blockage_map.end();
	for (; iter != end; ++iter) {
		Vehicle* veh = iter->second;
		TRACE("");
		TRACE("updateBlockages for " << (*veh));
		// Matching Kante blockieren
		if (veh->edge() && ! veh->edge()->isZoneEdge()) veh->edge()->setBlocked(true);
		// Multimatching Kanten blocckieren
		if (veh->edges().size()) {
			map< RoutePlanner::RndfEdge*, double >::const_iterator edges_iter = veh->edges().begin();
			map< RoutePlanner::RndfEdge*, double >::const_iterator edges_end = veh->edges().end();
			for (; edges_iter != edges_end; ++edges_iter) {
				if ( ! edges_iter->first->isZoneEdge() )
					edges_iter->first->setBlocked(true);
			}
		}
		// Nachfolgende Lanechange Kanten blockieren
		if ( veh->edge() ) {
			for (set< RndfEdge* >::iterator it = veh->edge()->toVertex()->getOutEdges().begin(); it != veh->edge()->toVertex()->getOutEdges().end(); ++it)
				block_adjacent_lanechange_edges( *it );
		}

	}

	// Lanechange Edges von Fahrzeugen blockieren
	for (map<int, Vehicle>::const_iterator it = vehicle_map.begin(); it != vehicle_map.end(); ++it) {
		const Vehicle& veh = it->second;
		block_adjacent_lanechange_edges( veh.edge() );
		for (map< RoutePlanner::RndfEdge*, double >::const_iterator me_it = veh.edges().begin(); me_it != veh.edges().end(); ++me_it)
			block_adjacent_lanechange_edges( me_it->first );
	}
}

bool VehicleManager::isMoving(int veh_id)
{
	return moving_map.find(veh_id) != moving_map.end();
}

bool VehicleManager::intersectsWithAnotherVehicle(const Vehicle& veh) const
{
	for (map<int, Vehicle>::const_iterator it = vehicle_map.begin(); it != vehicle_map.end(); ++it) {
		if ( veh.intersects( it->second ) ) return true;
	}
	return false;
}


std::ostream& operator << (std::ostream& ostrm, const Vehicle& obj)
{
	return ostrm << "Vehicle [A4Id "<< obj.id() <<"]  Pos: (x " << obj.xMatchedFrom() << ", y " << obj.yMatchedFrom() << ", yaw "<< obj.yawMatchedFrom() <<") isOn " << *obj.edge() << std::flush;
}

void block_adjacent_lanechange_edges(RndfEdge* edge)
{
	if ( edge == NULL ) return;
	GraphPlace place( edge, edge->getLength() / 2. );

	if ( ! place.valid ) return;

	if ( place.edge->isLaneChangeEdge() ) {
		place.goToLeftEdge();
		if ( ! place.valid ) return;
	}

	for (set< RndfEdge* >::iterator lc_it = place.edge->fromVertex()->getOutEdges().begin(); lc_it != place.edge->fromVertex()->getOutEdges().end(); ++lc_it)
		if ( (*lc_it)->isLaneChangeEdge() )	(*lc_it)->setBlocked(true);
	for (set< RndfEdge* >::iterator lc_it = place.edge->toVertex()->getInEdges().begin(); lc_it != place.edge->toVertex()->getInEdges().end(); ++lc_it)
		if ( (*lc_it)->isLaneChangeEdge() )	(*lc_it)->setBlocked(true);
}

} // namespace vlr
