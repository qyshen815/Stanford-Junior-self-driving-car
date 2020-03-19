/*
 * Copyright (C) 2009
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: aw_IntersectionTrafficLightWait.cpp
 * authors ....: Soeren Kammel
 * organization: Robert Bosch LLC
 * creation ...: Nov 24, 2009
 * modified ...: $Date:$
 * changed by .: $Author:$
 * revision ...: $Revision:$
 */
#include "aw_StPause.hpp"
#include "aw_StDrive.hpp"
#include "aw_StReplan.hpp"
#include "aw_StIntersectionTrafficLightWait.hpp"
#include "aw_StIntersectionTrafficLightStop.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightStop
 *---------------------------------------------------------------------------*/
StIntersectionTrafficLightStop::StIntersectionTrafficLightStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionTrafficLightStop"))
{
	context<ChsmPlanner>().bIntersection = true;
}

StIntersectionTrafficLightStop::~StIntersectionTrafficLightStop()
{
}

sc::result StIntersectionTrafficLightStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	Topology* topology = context<ChsmPlanner>().topology;
	assert(isec_man);


  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

  // Transition: Replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
	  return transit<StReplan>();

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

    // Transition: If traffic light switched to green while slowing down cross intersection
	if(!context<StIntersection>().isec_man->hasToStop()) {
    return transit<StIntersectionDriveInside>();
  }

	// Abstände berechnen
	// TODO Sicherstellen, dass die Stoplinie zur Kreuzung gehört
	double traffic_light_dist = topology->dist_to_traffic_light(isec_man->getIntersection(), NULL, &planner.stop_point_);
	double intersec_dist = topology->dist_to_intersection(isec_man->getIntersection());
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	// set turn signal
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

//	printf("STOP IN %f m;\t CURRENT SPEED %f\n", traffic_light_dist, currentPose().v());

  // Transition: Wait at Stopline (because ego_vehicle stopped at stopline)
	if ( traffic_light_dist < TRAFFIC_LIGHT_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
    return transit<StIntersectionTrafficLightWait>();
	}
	else if (traffic_light_dist == std::numeric_limits<double>::infinity()) {
	  std::cout << "WE RAN OVER A (NON GREEN) TRAFFIC LIGHT!!\n";
    return transit<StIntersectionDriveInside>();
	}

	// Transition: Queueing (falls Fahrzeug zurückgesetzt hat)
	if ( traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && (mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist) )
		return transit<StIntersectionQueue>();
	// Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
	if ( intersec_dist <= -0.1 ) {
		return transit<StDrive>();
	}

	// generate curvepoints
	context<StIntersection>().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);
	return forward_event();
}

} // namespace vlr

