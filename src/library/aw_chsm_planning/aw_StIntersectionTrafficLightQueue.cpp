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
 * file .......: aw_IntersectionTrafficLightQueue.cpp
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
#include "aw_StLaneChange.hpp"
//#include "aw_StIntersectionTrafficLightWait.hpp"
#include "aw_StIntersectionTrafficLightStop.hpp"
#include "aw_StIntersectionTrafficLightQueue.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightQueue
 *---------------------------------------------------------------------------*/
StIntersectionTrafficLightQueue::StIntersectionTrafficLightQueue(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionTrafficLightQueue"))
{
//	context<ChsmPlanner>().bIntersection = true; // ?!?
  congestionTimeout.now();
  congestionTimeout += RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner>().currentPose();
}

StIntersectionTrafficLightQueue::~StIntersectionTrafficLightQueue()
{
}

sc::result StIntersectionTrafficLightQueue::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
  IntersectionManager* isec_man = context<StIntersection>().isec_man;

  // calculate distances
  double traffic_light_dist = topology->dist_to_traffic_light(isec_man->getIntersection(), NULL, &planner.stop_point_);
  double intersec_dist = topology->dist_to_intersection(isec_man->getIntersection());
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}

  // intersection flag setzen
  if ( intersec_dist < 40) {
    context<ChsmPlanner>().bIntersection = true;
  }

  // TODO: fieser hack: wir wollen eigentlich nicht einfach so mit a* ueber die kreuzung brettern
  if (planner.params().enable_recovery && (/*intersec_dist < TRIGGER_DIST_INTERSECTION_RECOVER && */context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

  // Transition: Replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
	  return transit<StReplan>();

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();


  bool onPrio = isec_man->isOnPrio();
  bool hasToStop = isec_man->hasToStop();

  // Blinker setzen
  if ( intersec_dist < TRIGGER_DIST_BLINK )
    planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

  // Transition: Intersection Approach (because fwd vehicles disappeared)
  if ( intersec_dist < std::min(sv_veh_dist, mv_veh_dist) ) {
    return transit<StIntersectionApproach>();
  }

  // Transition: Lanechange
  double dist_to_lanechange = topology->dist_to_next_lanechange();
  if ( dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE)
    return transit<StLaneChange>();


  // transition: stop at traffic light
  if ( !onPrio && hasToStop && traffic_light_dist <= TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist >= TRAFFIC_LIGHT_DIST_THRESHOLD &&
      traffic_light_dist <= sv_veh_dist && traffic_light_dist <= mv_veh_dist)
    return transit<StIntersectionTrafficLightStop>();

  // Transition: stop at intersection (because Intersection is blocked)
  if ( onPrio && hasToStop && intersec_dist < TRIGGER_DIST_TRAFFIC_LIGHT && intersec_dist > TRAFFIC_LIGHT_DIST_THRESHOLD &&
      intersec_dist <= sv_veh_dist && intersec_dist <= mv_veh_dist) {
    return transit<StIntersectionPrioStop>();
  }

  // Transition: Intersection Drive
  if ( fabs(intersec_dist) <= 0.01 ) {
    if (onPrio) {
      return transit<StIntersectionPrioDriveInside>();
    } else {
      return transit<StIntersectionDriveInside>();
    }
  }

  // Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
  if ( intersec_dist <= -0.1 )
    return transit<StDrive>();

  if ( onPrio ) {
    std::pair< bool, double > prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
    if (!prioMerge.first) {
      traffic_light_dist = prioMerge.second;
    }
  }

  // generate curvepoints
  context<StIntersection>().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);
  return forward_event();
}

} // namespace vlr

