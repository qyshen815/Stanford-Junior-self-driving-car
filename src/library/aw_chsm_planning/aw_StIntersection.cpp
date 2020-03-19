/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <aw_kogmo_math.h>

#include "aw_StPause.hpp"
#include "aw_StStop.hpp"
#include "aw_StDrive.hpp"
#include "aw_StReplan.hpp"
#include "aw_StIntersection.hpp"
#include "aw_StIntersectionTrafficLightStop.hpp"
#include "aw_StLaneChange.hpp"
#include "aw_StError.hpp"
#include "aw_RndfVertex.h"
#include "aw_RndfEdge.h"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersection
 *---------------------------------------------------------------------------*/
StIntersection::StIntersection(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersection"))
, recover_recover_edge(0), inIntersection(false), recover_mode(RECOVER_TO_ENTRY) {
	// entry
	isec_man = new IntersectionManager(*context<ChsmPlanner>().topology, *context<ChsmPlanner>().vehicle_manager,
	    context<ChsmPlanner>().params().max_speed_merge_intersection, context<ChsmPlanner>().traffic_light_states_, context<ChsmPlanner>().intersection_predictor_mutex_);
	turnDirection = context<ChsmPlanner>().topology->nextTurnDirection();
	context<ChsmPlanner>().topology->intersection_manager = isec_man; // for visualisation

	max_wait_at_intersection.now();
	max_wait_at_intersection += INTERSECTION_MAX_WAIT_TIME;

	setRecoveryTime(INTERSECTION_RECOVERY_TIMEOUT);
	clearRecoveryIndicator();
}

StIntersection::~StIntersection() {
	context<ChsmPlanner>().topology->intersection_manager = NULL;
	delete isec_man;
	context<ChsmPlanner>().vehiclecmd.turnsignal = TURN_SIGNAL_NONE;
	context<ChsmPlanner>().bIntersection = false;
}

sc::result StIntersection::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// get the last edge we were on BEFORE the intersection begins. used to recover to this edge
	if (!context<ChsmPlanner>().topology->route_is_finished()) {
		RndfEdge* currentEdge = (*context<ChsmPlanner>().topology->current_edge_it)->getEdge();
		if (!inIntersection && currentEdge->getIntersection() == 0) {
			recover_recover_edge = currentEdge;
		}
		if (currentEdge->getIntersection()) {
			inIntersection = true;
		}
	}
	//context<ChsmPlanner>().vehiclecmd.turnsignal = context<StIntersection>().turnDirection;
	// forward event
	return forward_event();
}

void StIntersection::generateCurvepoints(const double stop_distance, const double max_speed)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	double sv_veh_dist, mv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	//std::pair<double, Vehicle*> nextVehicle = isec_man->distToVehicleOnWayThrouIntersection();
	double std_dist = std::min(sv_veh_dist, mv_veh_dist);
	//double obstacle_dist = min(nextVehicle.first, std_dist);
	planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StIntersectionApproach
 *---------------------------------------------------------------------------*/
StIntersectionApproach::StIntersectionApproach(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionApproach")) {

}

StIntersectionApproach::~StIntersectionApproach() {

}

sc::result StIntersectionApproach::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	IntersectionManager* isec_man = context<StIntersection>().isec_man;

  isec_man->obstaclePredictor().update();

	// calculate distances
	// TODO Sicherstellen, dass der die Stoplinie zur Kreuzung gehört
	double intersec_dist = topology->dist_to_intersection(isec_man->getIntersection());
	CurvePoint stop_point, tl_point;
	double stopline_dist = topology->dist_to_stopline(isec_man->getIntersection(), &stop_point); // TODO: make a nicer access to planner...

	double traffic_light_dist = topology->dist_to_traffic_light(isec_man->getIntersection(), NULL, &tl_point); // TODO: make a nicer access to planner...
	//	if(traffic_light_dist < 100) {printf("tld: %f\n",traffic_light_dist);}
	if(traffic_light_dist<stopline_dist) {planner.stop_point_ = tl_point;} else {planner.stop_point_ = stop_point;}
	
	double event_dist = std::min(traffic_light_dist, stopline_dist);

	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	bool onPrio = isec_man->isOnPrio();
	bool hasToStop = isec_man->hasToStop();

	// intersection flag setzen
	if ( intersec_dist < 40) {
		context<ChsmPlanner>().bIntersection = true;
	}

	// activate turn signal
	if ( intersec_dist < TRIGGER_DIST_BLINK )
		planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
	// TODO: fieser hack: wir wollen eigentlich nicht einfach so mit a* ueber die kreuzung brettern
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (vehicle is off track)
	if (topology->isOffTrack())
		return transit<StReplan>();

	// Transition: Replanning (route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	// TODO: TRAFFIC LIGHT!!
	// Transition: Queueing
	if ( mv_veh_dist < intersec_dist || sv_veh_dist < intersec_dist ||
			( stopline_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < stopline_dist || sv_veh_dist < stopline_dist) ) )
		return transit<StIntersectionQueue>();

	// Transition: Lanechange
	double dist_to_lanechange = topology->dist_to_next_lanechange();
	if ( dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE)
		return transit<StLaneChange>();

	if (traffic_light_dist < stopline_dist) {
    // Transition: Stop at traffic light
    if (hasToStop && traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist > TRAFFIC_LIGHT_DIST_THRESHOLD) {
      return transit<StIntersectionTrafficLightStop> ();
    }
  }
  else {
    // Transition: Stop at stop sign
    if (!onPrio && hasToStop && stopline_dist < TRIGGER_DIST_STOPLINE && stopline_dist > STOP_DIST_THRESHOLD) {
      return transit<StIntersectionStop> ();
    }
    // Transition: Stop at Intersection (because Intersection is blocked)
    if (onPrio && hasToStop && intersec_dist < TRIGGER_DIST_STOPLINE && intersec_dist > STOP_DIST_THRESHOLD) {
      return transit<StIntersectionPrioStop> ();
    }
  }

	// Transition: To get states right if something goes wrong: leave intersection mode if we are behind intersection
	if ( intersec_dist <= -0.1 )
		return transit<StDrive>();

	// Transition: Intersection Drive
	if ( onPrio && fabs(intersec_dist) <= 0.01 ) {
		return transit<StIntersectionPrioDriveInside>();
	}
	if ( !onPrio && fabs(event_dist) <= 0.01 ) {
		return transit<StIntersectionDriveInside>();
	}

	double max_speed = planner.params().max_speed_intersection_approach;

	if ( onPrio ) {
		std::pair< bool, double > prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
		if (!prioMerge.first) {
		  event_dist = prioMerge.second;
			max_speed = planner.params().max_speed_intersection;
		}
	}

	if (event_dist < TRIGGER_DIST_BLINK) {
		max_speed = planner.params().max_speed_intersection; // not planner.params().max_speed_intersection_approach ?!?
	}

	// generate curvepoints
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	double std_dist = std::min(sv_veh_dist, mv_veh_dist);
	planner.generateCurvePoints(event_dist, std_dist, max_speed);
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StIntersectionQueue
 *---------------------------------------------------------------------------*/
StIntersectionQueue::StIntersectionQueue(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionQueue"))
{
	congestionTimeout.now();
	congestionTimeout += RECOVERY_TIMEOUT;
	congestion_last_pose_ = context<ChsmPlanner>().currentPose();
}

StIntersectionQueue::~StIntersectionQueue()
{
}

sc::result StIntersectionQueue::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	
	isec_man->obstaclePredictor().update();

	// Abstände berechnen
	// TODO Sicherstellen, dass der die Stoplinie zur Kreuzung gehört
	double intersec_dist = topology->dist_to_intersection(isec_man->getIntersection());
	double stopline_dist = topology->dist_to_stopline(isec_man->getIntersection(), &planner.stop_point_);
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

	// Transition: Intersection Approach (because fwd vehicles disapeared)
	if ( intersec_dist < std::min(sv_veh_dist,mv_veh_dist) )
		return transit<StIntersectionApproach>();

	// Transition: Lanechange
	double dist_to_lanechange = topology->dist_to_next_lanechange();
	if ( dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE)
		return transit<StLaneChange>();


	// Transition: Stop at Stopline
	if ( !onPrio && hasToStop && stopline_dist <= TRIGGER_DIST_STOPLINE && stopline_dist >= STOP_DIST_THRESHOLD &&
			stopline_dist <= sv_veh_dist && stopline_dist <= mv_veh_dist)
		return transit<StIntersectionStop>();

	// Transition: Stop at Intersection (because Intersection is blocked)
	if ( onPrio && hasToStop && intersec_dist < TRIGGER_DIST_STOPLINE && intersec_dist > STOP_DIST_THRESHOLD &&
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

	// Transition: To get states right if something goes wrong: leave intersection mode if we are behind intersection
	if ( intersec_dist <= -0.1 )
		return transit<StDrive>();

	if ( onPrio ) {
		std::pair< bool, double > prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
		if (!prioMerge.first) {
			stopline_dist = prioMerge.second;
		}
	}

	// generate curvepoints
	context<StIntersection>().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);

	return forward_event();
}



/*---------------------------------------------------------------------------
 * StIntersectionStop
 *---------------------------------------------------------------------------*/
StIntersectionStop::StIntersectionStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionStop"))
{
	context<ChsmPlanner>().bIntersection = true;
}

StIntersectionStop::~StIntersectionStop()
{
}

sc::result StIntersectionStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	Topology* topology = context<ChsmPlanner>().topology;
	
  isec_man->obstaclePredictor().update();

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

	context<StIntersection>().isec_man->hasToStop(); // for debug visualisation

	// Abstände berechnen
	// TODO Sicherstellen, dass die Stoplinie zur Kreuzung gehört
	double stopline_dist = topology->dist_to_stopline(isec_man->getIntersection(), &planner.stop_point_);
	double intersec_dist = topology->dist_to_intersection(isec_man->getIntersection());
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	// set turn signal
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

//	printf("STOP IN %f m;\t CURRENT SPEED %f\n", stopline_dist, planner.currentSpeed());

  // Transition: Wait at Stopline (because ego_vehicle stopped at stopline)
	if ( stopline_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
    return transit<StIntersectionWait>();
	}
	else if (stopline_dist == std::numeric_limits<double>::infinity()) {
	  printf("WE RAN OVER STOP SIGN!!\n");
    return transit<StIntersectionDriveInside>();
	}

	// Transition: Queueing (falls Fahrzeug zurückgesetzt hat)
	if ( stopline_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < stopline_dist || sv_veh_dist < stopline_dist) )
		return transit<StIntersectionQueue>();
	// Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
	if ( intersec_dist <= -0.1 ) {
		return transit<StDrive>();
	}

	// generate curvepoints
	context<StIntersection>().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);
	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionStop
 *---------------------------------------------------------------------------*/
StIntersectionWait::StIntersectionWait(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionWait"))
{
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	isec_man->stoppedOnStopline();
	stop_time.now();
	stop_time += MIN_WAIT_STOPLINE;
	context<StIntersection>().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionWait::~StIntersectionWait() {
}

sc::result StIntersectionWait::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
  isec_man->obstaclePredictor().update();

	// Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	// stoppen
	planner.generateStopTrajectory();

	// Blinker setzen
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

	context<StIntersection>().isec_man->hasToStop(); // for debug visualisation

//	// Transition: drive on intersection
//  // We will go if it's our turn and rely on vehicle prediction
//  // to not run into anybody
//  //
//  //  if (isec_man->hasRightOfWay() && isExpired(stop_time)) {
    if (isec_man->hasRightOfWay() && isExpired(stop_time) && !isec_man->isVehicleOnIntersectionInFront()) {
		return transit<StIntersectionDriveInside>();
  }

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioStop
 *---------------------------------------------------------------------------*/
StIntersectionPrioStop::StIntersectionPrioStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionPrioStop"))
{
	context<ChsmPlanner>().bIntersection = true;
}

StIntersectionPrioStop::~StIntersectionPrioStop() {
}

sc::result StIntersectionPrioStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	Topology* topology = context<ChsmPlanner>().topology;
	
  isec_man->obstaclePredictor().update();

	bool hasToStop = context<StIntersection>().isec_man->hasToStop();

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	// Transition: Replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
		  return transit<StReplan>();



	// Abstände berechnen
	double intersection_dist = topology->dist_to_intersection(isec_man->getIntersection()); // Für die Reifenstellung
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	// Abstand ein wenig verkürzen für bessere Reifenstellung
	if (intersection_dist > 0.) {
		intersection_dist = std::max( intersection_dist -1.0 , 0. );
	}

	// Blinker setzen
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

	// Transition: To get states right if something goes wrong: leave intersection mode if we are behind intersection
	if ( intersection_dist <= -0.1 )
		return transit<StDrive>();

	if (hasToStop) {

		// Transition: Wait for Prio-Lane (because ego_vehicle stopped at intersection)
		if (intersection_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
			return transit<StIntersectionPrioWait>();
		}

		// Transition: Queueing (falls Fahrzeug zurückgesetzt hat)
		if ( intersection_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < intersection_dist || sv_veh_dist < intersection_dist))
			return transit<StIntersectionQueue>();

		// generate curvepoints
		context<StIntersection>().generateCurvepoints(intersection_dist, planner.params().max_speed_intersection);
	} else {
		// Transition: pass intersection on prio lane, no vehicle has priority
		if (intersection_dist < STOP_DIST_THRESHOLD ) {
			return transit<StIntersectionPrioDriveInside>();
		}

		// generate curvepoints without stopline
		context<StIntersection>().generateCurvepoints(std::numeric_limits<double>::infinity(), planner.params().max_speed_intersection);
	}

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioWait
 *---------------------------------------------------------------------------*/
StIntersectionPrioWait::StIntersectionPrioWait(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionPrioWait"))
{
	context<StIntersection>().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionPrioWait::~StIntersectionPrioWait() {
}

sc::result StIntersectionPrioWait::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
  isec_man->obstaclePredictor().update();

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	// stoppen
	planner.generateStopTrajectory();

	// Blinker setzen
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

	context<StIntersection>().isec_man->hasToStop(); // for debug visualisation

	// Transition: drive on intersection
	if (isec_man->hasRightOfWay())
		return transit<StIntersectionPrioDriveInside>();

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionDriveInside
 *---------------------------------------------------------------------------*/
StIntersectionDriveInside::StIntersectionDriveInside(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionDriveInside"))
{
	intersectionEntered = false;
	context<StIntersection>().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionDriveInside::~StIntersectionDriveInside() {
}

sc::result StIntersectionDriveInside::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;
	Vehicle* ego_veh = &topology->ego_vehicle;
	assert(ego_veh->edge());
	RndfEdge* edge = ego_veh->edge();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
  isec_man->obstaclePredictor().update();
	
  //double stopline_dist = topology->dist_to_stopline(isec_man->getIntersection(), &stop_point_);
  GraphTools::PlaceOnGraph place( topology->current_edge_it, topology->ego_vehicle.distFromStart(), topology->route.route );
  assert(*place.edge);
  double stopline_fwd_dist;
  place.go_fwd_to_stopline( stopline_fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec_man->getIntersection() );
  stopline_fwd_dist -= FRONT_BUMPER_DELTA;
  bool isNearStopline = fabs(stopline_fwd_dist) < 0.3;

	isec_man->hasToStop(); // for debug visualisation

  // Transition: Recovery Mode
	if (!isNearStopline && isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	//bool onPrio = context<StIntersection>().isec_man->isOnPrio();
	bool isMergeAllowed = !isNearStopline || isec_man->hasRightOfWay();


	// Blinker setzen
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

	// Sicherstellen das man die Kreuzung betreten hat (wegen Hinterachspukt-Matching)
	if (!intersectionEntered && edge->getIntersection())
		intersectionEntered = true;

	// Transition: normales Drive (da Kreuzung verlassen)
	if (intersectionEntered && !edge->getIntersection())
		return transit<StDrive>();

	//     We will go if it's our turn and rely on vehicle prediction
	//     to not run into anybody
//  bool isVehOnIntersection = isec_man->isVehicleOnIntersectionInFront();
//  if (isVehOnIntersection || !isMergeAllowed) {
  if (!isMergeAllowed) {
		planner.generateStopTrajectory();
	} else {
		// generate curvepoints without stopline
		context<StIntersection>().generateCurvepoints(std::numeric_limits<double>::infinity(), planner.params().max_speed_intersection);
	}

	// special case: mission end is the intersection exit vertex
	double dist_to_end = topology->dist_to_mission_end();
	if (dist_to_end < TRIGGER_DIST_NASTY_GOAL) {
		return transit<StStop>();
	}

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioDriveInside
 *---------------------------------------------------------------------------*/
StIntersectionPrioDriveInside::StIntersectionPrioDriveInside(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionPrioDriveInside"))
{
	intersectionEntered = false;
	context<StIntersection>().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionPrioDriveInside::~StIntersectionPrioDriveInside() {
}

sc::result StIntersectionPrioDriveInside::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;
	Vehicle* ego_veh = &topology->ego_vehicle;
	assert(ego_veh->edge());
	RndfEdge* edge = ego_veh->edge();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;

	isec_man->hasToStop(); // for debug visualisation
  isec_man->obstaclePredictor().update();

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	//bool onPrio = context<StIntersection>().isec_man->isOnPrio();
	//bool isVehOnIntersection = context<StIntersection>().isec_man->isVehicleOnIntersectionInFront();

	bool hasToStop = isec_man->isInfrontMergePoint() && isec_man->hasToStop();

	// set turn signal
	planner.vehiclecmd.turnsignal = context<StIntersection>().turnDirection;

	if (hasToStop) {
		planner.generateStopTrajectory();
		return forward_event();
	}

	// Sicherstellen das man die Kreuzung betreten hat (wegen Hinterachspukt-Matching)
	if (!intersectionEntered && edge->getIntersection())
		intersectionEntered = true;

	// Transition: normales Drive (da Kreuzung verlassen)
	if (intersectionEntered && !edge->getIntersection())
		return transit<StDrive>();

	double stopline_dist = std::numeric_limits<double>::infinity();

	std::pair< bool, double > prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
	if (!prioMerge.first) {
		stopline_dist = prioMerge.second;
	}

	// generate curvepoints
	context<StIntersection>().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);

	// special case: mission end is the intersection exit vertex
	double dist_to_end = topology->dist_to_mission_end();
	if (dist_to_end < TRIGGER_DIST_NASTY_GOAL) {
		return transit<StStop>();
	}

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionRecover
 *---------------------------------------------------------------------------*/
StIntersectionRecover::StIntersectionRecover(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionRecover"))
{
	map_timer.now();
	map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
}

StIntersectionRecover::~StIntersectionRecover() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StIntersectionRecover::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return discard_event(); // intersection state don't get this
}

/*---------------------------------------------------------------------------
 * StIntersectionRecoverPrepare
 *---------------------------------------------------------------------------*/
StIntersectionRecoverPrepare::StIntersectionRecoverPrepare(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionRecoverPrepare"))
{
}

StIntersectionRecoverPrepare::~StIntersectionRecoverPrepare() {
}

sc::result StIntersectionRecoverPrepare::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	if (isExpired(context<StIntersectionRecover>().map_timer)) {
		return transit<StIntersectionRecoverToExit>();
	} else {
		context<ChsmPlanner>().generateStopTrajectory();
		return forward_event();
	}
}

/*---------------------------------------------------------------------------
 * StIntersectionRecoverToExit
 *---------------------------------------------------------------------------*/
StIntersectionRecoverToExit::StIntersectionRecoverToExit(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionRecoverToExit")),
targetEdge(0)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;

	if (context<StIntersection>().recover_mode == StIntersection::RECOVER_TO_EXIT) {

		bool inIntersection = false;
		RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

		while (targetEdge==0 && anno_edge_it != topology->route.route.end()) {
			assert((*anno_edge_it));
			assert((*anno_edge_it)->getEdge());
			RndfEdge* edge = (*anno_edge_it)->getEdge();
			if (!inIntersection && edge->getIntersection()) {
				inIntersection = true;
			}
			if ( inIntersection && (edge->getIntersection()==0)) {

				targetEdge = edge;

				double x = edge->fromVertex()->x();
				double y = edge->fromVertex()->y();
				double psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(),
						edge->toVertex()->x() - edge->fromVertex()->x());

				// add an afterburner distance to make sure you end up outside the intersection
				x+=AFTER_BURNER_DIST*cos(psi);
				y+=AFTER_BURNER_DIST*sin(psi);

				// start plannning
				planner.navigator_control.x = x;
				planner.navigator_control.y = y;
				planner.navigator_control.psi = psi;
				planner.navigator_control.mode = UC_NAVI_PARKING;

				planner.addMessage("Recover to intersection exit");

				break;
			}
			++anno_edge_it;
		}
	} else {

		RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

		while (targetEdge==0 && anno_edge_it != topology->route.route.end()) {
			assert((*anno_edge_it));
			assert((*anno_edge_it)->getEdge());
			RndfEdge* edge = (*anno_edge_it)->getEdge();
			if ( (edge->getIntersection())) {

				targetEdge = edge;

				double x = edge->fromVertex()->x();
				double y = edge->fromVertex()->y();
        double psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(),
            edge->toVertex()->x() - edge->fromVertex()->x());

				// substract front bumper distance to stop at stopline/intersection
				x-=(FRONT_BUMPER_DELTA+0.9)*cos(psi);
				y-=(FRONT_BUMPER_DELTA+0.9)*sin(psi);

				// start plannning
				planner.navigator_control.x = x;
				planner.navigator_control.y = y;
				planner.navigator_control.psi = psi;
				planner.navigator_control.mode = UC_NAVI_PARKING;

				planner.addMessage("Recover to intersection entry");

				break;
			}
			++anno_edge_it;
		}
	}
	planner.generateStopTrajectory();

  // reset recover indicator of intersection state
  context<StIntersection>().clearRecoveryIndicator();
}

StIntersectionRecoverToExit::~StIntersectionRecoverToExit() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StIntersectionRecoverToExit::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;
	navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

  // Transition: Recovery Mode
  if (targetEdge==0 || checkRecovery()) { // recover mode has its own recover timer
  	planner.addMessage("Recover to exit was not successful");
  	return transit<StIntersectionRecoverBackupToEntry>();
  }

	// check if vehicle is at exit
	RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

	// Transition: if A* is complete, do a replan
	if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST/* || (*topology->current_edge_it)->getEdge() == targetEdge*/) {
		return transit<StReplan>();
	}

	// copy curvepoints if data available
	if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
		//memcpy(planner.curvepoints_, &navigator_feedback->curvepoints,sizeof(CurvePoints));  // TODO: substitute CurvePoints
	} else {
		planner.generateStopTrajectory();
	}

	return discard_event(); // intersection state don't get this
}


/*---------------------------------------------------------------------------
 * StIntersectionRecoverBackupToEntry
 *---------------------------------------------------------------------------*/
StIntersectionRecoverBackupToEntry::StIntersectionRecoverBackupToEntry(my_context ctx) : my_base(ctx), kogmo_base(std::string("StIntersectionRecoverBackupToEntry"))
{
	ChsmPlanner& planner = context<ChsmPlanner>();

	RndfEdge* edge = context<StIntersection>().recover_recover_edge;

	if (edge) {
		double x = edge->fromVertex()->x();
		double y = edge->fromVertex()->y();
    double psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(),
        edge->toVertex()->x() - edge->fromVertex()->x());

		// start plannning
		planner.navigator_control.x = x;
		planner.navigator_control.y = y;
		planner.navigator_control.psi = psi;
		planner.navigator_control.mode = UC_NAVI_PARKING;

		planner.addMessage("Recover to intersection entry");
	}
	planner.generateStopTrajectory();

  // reset recover indicator of intersection state
  context<StIntersection>().clearRecoveryIndicator();
}

StIntersectionRecoverBackupToEntry::~StIntersectionRecoverBackupToEntry() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StIntersectionRecoverBackupToEntry::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;
  isec_man->obstaclePredictor().update();

  // Transition: Recovery Mode
  if (context<StIntersection>().recover_recover_edge==0) {
  	planner.addMessage("Recover was not successful -> don't know what to do");
  	return transit<StError>();
  }

	// Transition: if A* is complete, mark intersection blocked and do a replan
	if(planner.params().enable_blockade_detection && navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST) {
  	TRndfEdgeSet::const_iterator iter, end;
  	for (iter=isec_man->getIntersection()->getEdges().begin(),end=isec_man->getIntersection()->getEdges().end(); iter!=end; ++iter) {
  		planner.blockade_manager->forceBlockade((*iter));
  	}
		return transit<StReplan>();
	}

	// copy curvepoints if data available
	if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
		// memcpy(planner.curvepoints_, &navigator_feedback->curvepoints,sizeof(CurvePoints));  // TODO: substitute CurvePoints
	} else {
		planner.generateStopTrajectory();
	}

	return discard_event(); // intersection state don't get this
}

} // namespace vlr

