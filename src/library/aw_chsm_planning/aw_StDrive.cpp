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
#include "aw_StIntersection.hpp"
#include "aw_StStop.hpp"
#include "aw_StDrive.hpp"
#include "aw_StZone.hpp"
#include "aw_StTrafficLight.hpp"
#include "aw_StCrosswalk.hpp"
#include "aw_StReplan.hpp"
#include "aw_StLaneChange.hpp"

namespace vlr {

//#define TRACE(x) 	std::cout << context<ChsmPlanner>().name() << " [StDrive] " << x << std::endl;
#define TRACE(x)

/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
StDrive::StDrive(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrive"))
{
}

StDrive::~StDrive() {
}

sc::result StDrive::react(const EvStop&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  return transit<StStop>();
}

sc::result StDrive::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;

  double dist_to_missionend = topology->dist_to_mission_end();
  CurvePoint cp; //dummy point: not forwarded to planner since in this stage it's unclear which the next stop point will be

//  printf("distances:\n zone:\t\t%f\nstop:\t\t%f\nisec:\t\t%f\nkturn:\t\t%f\nlach:\t\t%f\ntrali:\t\t%f\ncrow:\t\t%f\n\n",
//      topology->dist_to_next_zone(), topology->dist_to_next_sole_stopline(cp),
//      topology->dist_to_next_intersection(), topology->dist_to_next_kturn(),
//      topology->dist_to_next_lanechange(), topology->dist_to_next_traffic_light(),
//      topology->dist_to_next_crosswalk());
  // select minimum trigger dist
  // ATTN: order does matter!
  // ATTN: also add transition in StDriveStart
  std::map<double, TransitionEnum> transit_map;
  transit_map[topology->dist_to_next_zone()]         = TRANSIT_TO_ZONE;
  transit_map[topology->dist_to_next_sole_stopline(cp)]= TRANSIT_TO_STOPLINE;
  transit_map[topology->dist_to_next_intersection()] = TRANSIT_TO_INTERSECTION;
  transit_map[topology->dist_to_next_kturn()]        = TRANSIT_TO_KTURN;
//  transit_map[topology->dist_to_next_lanechange()]   = TRANSIT_TO_LANECHANGE;
  transit_map[topology->dist_to_next_traffic_light()]   = TRANSIT_TO_TRAFFIC_LIGHT;
  transit_map[topology->dist_to_next_crosswalk()]   = TRANSIT_TO_CROSSWALK;
  transit_map[dist_to_missionend]                    = TRANSIT_TO_GOAL;

  // failsafe, would be really bad to go in a wrong state at mission end
  if (dist_to_missionend < 0.0) {
  	return transit<StStop>();
  }

	double min_dist = transit_map.begin()->first;
	switch (transit_map.begin()->second) {
	case TRANSIT_TO_GOAL: if (min_dist < TRIGGER_DIST_GOAL) return transit<StStop>(); break;
	case TRANSIT_TO_ZONE: if (min_dist > 0.0 && min_dist < TRIGGER_DIST_ENTERING_ZONE) return transit<StZone>(); break;
	case TRANSIT_TO_KTURN: if (/*min_dist >= 0.0 && */min_dist < TRIGGER_DIST_KTURN) return transit<StDriveKTurn>(); break;
	case TRANSIT_TO_INTERSECTION: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_INTERSECTION) return transit<StIntersection>(); break;
	case TRANSIT_TO_LANECHANGE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange>(); break;
  case TRANSIT_TO_STOPLINE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_STOPLINE) return transit<StDriveStop>(); break;
  case TRANSIT_TO_TRAFFIC_LIGHT: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_TRAFFIC_LIGHT) return transit<StTrafficLight>(); break;
  case TRANSIT_TO_CROSSWALK: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_CROSSWALK) return transit<StCrosswalk>(); break;
	case TRANSIT_DEFAULT: default: break;
	}

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
StDriveStart::StDriveStart(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveStart"))
{
}

StDriveStart::~StDriveStart() {
}

sc::result StDriveStart::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) {return transit<StGlobalRecover>();}

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;

  double dist_to_missionend = topology->dist_to_mission_end();
  CurvePoint cp; //dummy point: not forwarded to planner since in this stage it's unclear which the next stop point will be

  // select minimum trigger dist
  std::map<double, TransitionEnum> transit_map;
  transit_map[topology->dist_to_next_zone()]         = TRANSIT_TO_ZONE;
  transit_map[topology->dist_to_next_sole_stopline(cp)]= TRANSIT_TO_STOPLINE;
  transit_map[topology->dist_to_next_intersection()] = TRANSIT_TO_INTERSECTION;
  transit_map[topology->dist_to_next_kturn()]        = TRANSIT_TO_KTURN;
//  transit_map[topology->dist_to_next_lanechange()]   = TRANSIT_TO_LANECHANGE;
  transit_map[topology->dist_to_next_traffic_light()]   = TRANSIT_TO_TRAFFIC_LIGHT;
  transit_map[topology->dist_to_next_crosswalk()]   = TRANSIT_TO_CROSSWALK;
  transit_map[dist_to_missionend]                    = TRANSIT_TO_GOAL;

//  printf("distance to:\nzone: %f\nsole stop line: %f\nintersection: %f\nkturn: %f\ntraffic light: %f\n crosswalk: %f\n EOM: %f\n\n",
//          topology->dist_to_next_zone(), topology->dist_to_next_sole_stopline(cp), topology->dist_to_next_intersection(),
//          topology->dist_to_next_kturn(), topology->dist_to_next_traffic_light(), topology->dist_to_next_crosswalk(),dist_to_missionend);
  	// TODO: Transition *from* lane change

  // failsafe, would be really bad to go in a wrong state at mission end
  if (dist_to_missionend < 0.0) {
  	return transit<StStop>();
  }

  bool inZone = (*topology->current_edge_it)->getEdge()->isZoneEdge() && !(*topology->current_edge_it)->hasAnnotation(UC_MANEUVER_ZONE_EXIT);

	double min_dist = transit_map.begin()->first;
	switch (transit_map.begin()->second) {
	case TRANSIT_TO_GOAL: if (min_dist < TRIGGER_DIST_GOAL) return transit<StStop>(); break;
	case TRANSIT_TO_ZONE: if ( (min_dist > 0.0 && min_dist < TRIGGER_DIST_ENTERING_ZONE) || inZone) return transit<StZone>(); break;
	case TRANSIT_TO_KTURN: if (min_dist < TRIGGER_DIST_KTURN) return transit<StDriveKTurn>(); break;
    case TRANSIT_TO_INTERSECTION: if (min_dist < TRIGGER_DIST_APPROACH_INTERSECTION) return transit<StIntersection>(); break;
	case TRANSIT_TO_LANECHANGE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange>(); break;
	case TRANSIT_TO_STOPLINE:
		if (dist_to_missionend < TRIGGER_DIST_GOAL) return transit<StStop>(); break; // special case: mission end is a stoppoint
		if (min_dist < TRIGGER_DIST_STOPLINE && min_dist >= STOP_DIST_THRESHOLD) return transit<StDriveStop>();
		break;
	case TRANSIT_TO_TRAFFIC_LIGHT: if (min_dist < TRIGGER_DIST_APPROACH_TRAFFIC_LIGHT) return transit<StTrafficLight>(); break;
	case TRANSIT_TO_CROSSWALK: if (min_dist < TRIGGER_DIST_APPROACH_CROSSWALK) return transit<StCrosswalk>(); break;
	case TRANSIT_DEFAULT: default: return transit<StDriveOnLane>(); break;
	}

	return transit<StDriveOnLane>(); // this case DOES happen !? - so don't delete this line
}

/*---------------------------------------------------------------------------
 * StDriveOnLane
 *---------------------------------------------------------------------------*/
StDriveOnLane::StDriveOnLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveOnLane"))
{
	setRecoveryTime(RECOVERY_TIMEOUT_DRIVEONLANE);
	clearRecoveryIndicator();
}

StDriveOnLane::~StDriveOnLane() {
}

sc::result StDriveOnLane::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* top = planner.topology;

  // Transition: Replanning (because ego vehicle is off track)
  if ( top->isOffTrack() )
	  return transit<StReplan>();

  // Transition: Replanning (because route is blocked)
  if ( top->isRouteBlocked() )
	  return transit<StReplan>();

  // Transition: Recover if we do not make progress
  if ( checkRecovery() ) {
	  if ( top->isRouteBlocked( FROG_MODE_LEAP_DIST, false ) )
		  return transit<StReplan>();
	  else
		  return transit<StDriveRecover>();
  }

  // Transition: lane change
  // TODO: replace with trajectory test
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

//	if (sv_veh_dist < mv_veh_dist && sv_veh_dist < TRIGGER_DIST_OBSTACLE && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
//		return transit<StLaneChange>();
//	}

//	printf("mv_veh_dist: %f, sv_veh_dist: %f\n", mv_veh_dist, sv_veh_dist);
	double veh_dist = std::min(mv_veh_dist, sv_veh_dist);
//	TRACE("veh_dist = " << veh_dist << " ~ " << top->dist_to_next_standing_veh());
	// generate curvepoints
  planner.generateCurvePoints(std::numeric_limits<double>::infinity(), veh_dist, planner.params().max_speed_drive);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StDriveRecover
 *---------------------------------------------------------------------------*/
StDriveRecover::StDriveRecover(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveRecover"))
{
	map_timer.now();
	map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
}

StDriveRecover::~StDriveRecover() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StDriveRecover::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return discard_event(); // intersection state don't get this
}

/*---------------------------------------------------------------------------
 * StDriveRecoverPrepare
 *---------------------------------------------------------------------------*/
StDriveRecoverPrepare::StDriveRecoverPrepare(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveRecoverPrepare"))
{
}

StDriveRecoverPrepare::~StDriveRecoverPrepare() {
}

sc::result StDriveRecoverPrepare::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	if (isExpired(context<StDriveRecover>().map_timer)) {
		return transit<StDriveRecoverAStar>();
	} else {
		context<ChsmPlanner>().generateStopTrajectory();
		return forward_event();
	}
}

/*---------------------------------------------------------------------------
 * StDriveRecoverAStar
 *---------------------------------------------------------------------------*/
StDriveRecoverAStar::StDriveRecoverAStar(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveRecoverAStar")),
targetEdge(0)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;

	GraphTools::PlaceOnGraph recover_place( topology->current_edge_it, topology->ego_vehicle.distFromStart(), topology->route.route );
	recover_place += FROG_MODE_LEAP_DIST;
	targetEdge = (*recover_place.edge)->getEdge();

	if ( targetEdge ) {

			double x = recover_place.point().x();
			double y = recover_place.point().y();;
			double psi = atan2(targetEdge->toVertex()->y() - targetEdge->fromVertex()->y(),
                         targetEdge->toVertex()->x() - targetEdge->fromVertex()->x());

			// start plannning
			planner.navigator_control.x = x;
			planner.navigator_control.y = y;
			planner.navigator_control.psi = psi;
			planner.navigator_control.mode = UC_NAVI_PARKING;

			planner.addMessage("Recover to route point ahead");

	}
	planner.generateStopTrajectory();
}

StDriveRecoverAStar::~StDriveRecoverAStar() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StDriveRecoverAStar::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;
	navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

  // Transition: Recovery Mode
  if (targetEdge==0) {
  	planner.addMessage("Recover not successful");
  	return transit<StPause>();
  }

	// check if vehicle is at exit
	// Transition: if A* is complete, do a replan
	if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST/* || (*topology->current_edge_it)->getEdge() == targetEdge*/) {
		if ( (*topology->current_edge_it)->getEdge()->isZoneEdge() && ! (*topology->current_edge_it)->hasAnnotation(UC_MANEUVER_ZONE_EXIT) )
			return transit<StZone>();
		else
			return transit<StDrive>();
	}

	// copy curvepoints if data available
	if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
		// memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints)); // TODO: substitute CurvePoints
	} else {
		planner.generateStopTrajectory();
	}

	return discard_event(); // intersection state don't get this
}


/*---------------------------------------------------------------------------
 * StDriveStop
 *---------------------------------------------------------------------------*/
StDriveStop::StDriveStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveStop"))
{
}

StDriveStop::~StDriveStop() {
}

sc::result StDriveStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;

  // Transition: Replanning (because ego vehicle is off track)
  if (topology->isOffTrack())
	  return transit<StReplan>();

  // Transition: Replanning (because route is blocked)
  if ( topology->isRouteBlocked() )
	  return transit<StReplan>();

  double stopline_dist = topology->dist_to_next_sole_stopline(planner.stop_point_);  // TODO: make a nicer access to planner...
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	double veh_dist = std::min(mv_veh_dist, sv_veh_dist);

	if (stopline_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
		return transit<StDriveStopped>();
	} else {

	  // Transition: Recover if we do not make progress
	  if (checkRecovery()) {
	  	return transit<StDriveRecover>();
	  }

		planner.generateCurvePoints(stopline_dist, veh_dist-1.0, planner.params().max_speed_intersection_approach); // TODO: following distance?
		//return forward_event();
		return discard_event();
	}

}

/*---------------------------------------------------------------------------
 * StDriveStopped
 *---------------------------------------------------------------------------*/
StDriveStopped::StDriveStopped(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveStopped"))
{
	stop_timer.now();
	stop_timer += MIN_WAIT_STOPLINE;
}

StDriveStopped::~StDriveStopped() {
}

sc::result StDriveStopped::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();

	if (isExpired(stop_timer)) {
		return transit<StDrive>();
	} else {
		planner.generateStopTrajectory();
		return discard_event();
	}

}

} // namespace vlr
