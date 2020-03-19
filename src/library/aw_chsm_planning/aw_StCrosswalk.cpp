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
#include "aw_StCrosswalk.hpp"
#include "aw_StLaneChange.hpp"
#include "aw_StError.hpp"
#include "aw_RndfVertex.h"
#include "aw_RndfEdge.h"

namespace vlr {

/*---------------------------------------------------------------------------
 * StCrosswalk
 *---------------------------------------------------------------------------*/
StCrosswalk::StCrosswalk(my_context ctx) : my_base(ctx), kogmo_base(std::string("StCrosswalk")) {
	cwm_ = new CrosswalkManager(context<ChsmPlanner>().topology);

//	max_wait_at_intersection.now();
//	max_wait_at_intersection += INTERSECTION_MAX_WAIT_TIME;
//
//	setRecoveryTime(INTERSECTION_RECOVERY_TIMEOUT);
//	clearRecoveryIndicator();
}

StCrosswalk::~StCrosswalk() {
	delete cwm_;
}

sc::result StCrosswalk::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();
	return forward_event();
}

void StCrosswalk::generateCurvepoints(const double stop_distance, const double max_speed)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	double sv_veh_dist, mv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	double std_dist = std::min(sv_veh_dist, mv_veh_dist);
	//double obstacle_dist = min(nextVehicle.first, std_dist);
	planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StCrosswalkApproach
 *---------------------------------------------------------------------------*/
StCrosswalkApproach::StCrosswalkApproach(my_context ctx) : my_base(ctx), kogmo_base(std::string("StCrosswalkApproach")) {

}

StCrosswalkApproach::~StCrosswalkApproach() {

}

sc::result StCrosswalkApproach::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

    // get context data
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	CrosswalkManager* cwm = context<StCrosswalk>().cwm_;

  CurvePoint cw_point;
	double crosswalk_dist = topology->dist_to_next_crosswalk(NULL, &cw_point);

  planner.stop_point_ = cw_point;

	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

	// transition: replanning (vehicle is off track)
	if (topology->isOffTrack())
		return transit<StReplan>();

	// transition: replanning (route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();


	// transition: queueing
	if ( mv_veh_dist < crosswalk_dist || sv_veh_dist < crosswalk_dist )
		return transit<StCrosswalkQueue>();


  // transition: stop at occupied crosswalk
  if (hasToStop && crosswalk_dist < TRIGGER_DIST_CROSSWALK) { // && crosswalk_dist > CROSSWALK_DIST_THRESHOLD) {
    return transit<StCrosswalkStop>();
  }


	if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
		return transit<StDrive>();
	}

	double max_speed;

	// generate curvepoints
	if(!hasToStop) {
	  crosswalk_dist = std::numeric_limits<double>::infinity();
	   max_speed = planner.params().max_speed_crosswalk_approach_empty;
	  }
	else {
  max_speed = planner.params().max_speed_crosswalk_approach_occupied;
 }
	double std_dist = std::min(sv_veh_dist, mv_veh_dist);
	planner.generateCurvePoints(crosswalk_dist, std_dist, max_speed);
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StCrosswalkQueue
 *---------------------------------------------------------------------------*/
StCrosswalkQueue::StCrosswalkQueue(my_context ctx) : my_base(ctx), kogmo_base(std::string("StCrosswalkQueue"))
{
	congestionTimeout.now();
	congestionTimeout += RECOVERY_TIMEOUT;
	congestion_last_pose_ = context<ChsmPlanner>().currentPose();
}

StCrosswalkQueue::~StCrosswalkQueue()
{
}

sc::result StCrosswalkQueue::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// get context data
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	CrosswalkManager* cwm = context<StCrosswalk>().cwm_;

  CurvePoint cw_point;
  double crosswalk_dist = topology->dist_to_next_crosswalk(NULL, &cw_point);
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

//  planner.stop_point_ = cw_point;

	// transition: replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
		return transit<StReplan>();

	// transition: replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

	  // transition: crosswalk approach (vehicles ahead disappeared)
	if ( crosswalk_dist < std::min(sv_veh_dist, mv_veh_dist) ) {
		return transit<StCrosswalkApproach>();
	}

	// transition: stop at crosswalk
	if (hasToStop && crosswalk_dist <= TRIGGER_DIST_CROSSWALK && crosswalk_dist >= CROSSWALK_DIST_THRESHOLD &&
	    crosswalk_dist <= sv_veh_dist && crosswalk_dist <= mv_veh_dist) {
		return transit<StCrosswalkStop>();
	}

  if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
    return transit<StDrive>();
  }

//	// Transition: Crosswalk Drive
//	if ( fabs(intersec_dist) <= 0.01 ) {
//		if (onPrio) {
//			return transit<StCrosswalkPrioDriveInside>();
//		} else {
//			return transit<StCrosswalkDriveInside>();
//		}
//	}
//
//	// Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
//	if ( intersec_dist <= -0.1 )
//		return transit<StDrive>();
//
    double max_speed;
    if(!hasToStop) {
       max_speed = planner.params().max_speed_crosswalk_approach_empty;
      }
    else {
    max_speed = planner.params().max_speed_crosswalk_approach_occupied;
   }
    // generate curvepoints
	context<StCrosswalk>().generateCurvepoints(crosswalk_dist, max_speed);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StCrosswalkStop
 *---------------------------------------------------------------------------*/
StCrosswalkStop::StCrosswalkStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StCrosswalkStop"))
{
}

StCrosswalkStop::~StCrosswalkStop()
{
}

sc::result StCrosswalkStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// get context data
	ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;
	CrosswalkManager* cwm = context<StCrosswalk>().cwm_;

    // transition: replanning (because ego vehicle is off track)
	if (topology->isOffTrack()) {
	  return transit<StReplan>();
	}

    // transition: replanning (because route is blocked)
	if ( topology->isRouteBlocked() ) {
		return transit<StReplan>();
	}

  CurvePoint cw_point;
  double crosswalk_dist = topology->dist_to_next_crosswalk(NULL, &cw_point);

  planner.stop_point_ = cw_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

  // transition: wait at crosswalk (because we stopped already)
	if ( hasToStop && crosswalk_dist < CROSSWALK_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
    return transit<StCrosswalkWait>();
	}
  else if (hasToStop && crosswalk_dist == std::numeric_limits<double>::infinity()) {
    printf("WE RAN OVER A CROSSWALK!!\n");
    return transit<StDrive>();
  }
  else if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
    return transit<StDrive>();
  }

    // transition: queueing (in case vehicle backed up)
	if ( crosswalk_dist < TRIGGER_DIST_CROSSWALK && (mv_veh_dist < crosswalk_dist || sv_veh_dist < crosswalk_dist) ) {
		return transit<StCrosswalkQueue>();
	}

    // generate curvepoints
//	context<StCrosswalk>().generateCurvepoints(crosswalk_dist);
  double max_speed;
  if(!hasToStop) {
     crosswalk_dist = std::numeric_limits<double>::infinity();
     max_speed = planner.params().max_speed_crosswalk_approach_empty;
    }
  else {
  max_speed = planner.params().max_speed_crosswalk_approach_occupied;
 }
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(crosswalk_dist, std_dist,max_speed);
	return forward_event();
}

/*---------------------------------------------------------------------------
 * StCrosswalkStop
 *---------------------------------------------------------------------------*/
StCrosswalkWait::StCrosswalkWait(my_context ctx) : my_base(ctx), kogmo_base(std::string("StCrosswalkWait"))
{
//	stop_time.now();
//	stop_time += MIN_WAIT_STOPLINE;
}

StCrosswalkWait::~StCrosswalkWait() {
}

sc::result StCrosswalkWait::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) {return transit<StGlobalRecover>();}

	// get context data
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	CrosswalkManager* cwm = context<StCrosswalk>().cwm_;

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

	// stoppen
	planner.generateStopTrajectory();

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

	// Transition: drive on intersection
	if (!hasToStop) { // && isExpired(stop_time) && !isec_man->isVehicleOnCrosswalkInFront()) {
		return transit<StDrive>();
	}

	return forward_event();
}

} // namespace vlr

