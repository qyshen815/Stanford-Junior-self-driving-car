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
//#include <aw_kogmo_math.h>

#include <traffic_light_interface.h>

#include "aw_StPause.hpp"
#include "aw_StDrive.hpp"
#include "aw_StReplan.hpp"
#include "aw_StTrafficLight.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StTrafficLight
 *---------------------------------------------------------------------------*/
StTrafficLight::StTrafficLight(my_context ctx) : my_base(ctx), kogmo_base(std::string("StTrafficLight")) {

  try {
    tlm_ = new TrafficLightManager(context<ChsmPlanner>().topology);
  }
  catch(vlr::Exception e) {
    throw e;
  }

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;

  std::vector<std::string> tl_names;
  topology->dist_to_next_traffic_light(&tl_names, NULL);
  std::vector<std::string>::const_iterator tlnit=tl_names.begin(), tlnit_end=tl_names.end();
  TrafficLightPose tl_pose;

  pthread_mutex_lock(&planner.traffic_light_poses_mutex_);
  planner.publish_traffic_lights_=true;
  planner.traffic_light_poses_.clear();

  for(; tlnit != tlnit_end; tlnit++) {
    rndf::TrafficLight* tl = const_cast<rndf::RoadNetwork*>(&topology->roadNetwork())->trafficLight(*tlnit);
    if(tl) {
      tl_pose.lat = tl->lat();
      tl_pose.lon = tl->lon();
      tl_pose.z = tl->z();
      tl_pose.orientation = tl->orientation();
      strcpy(tl_pose.name, tl->name().c_str());

      planner.traffic_light_poses_.push_back(tl_pose);
    }
  }

  planner.publish_traffic_lights_=true;

  pthread_mutex_unlock(&planner.traffic_light_poses_mutex_);

  //  max_wait_at_intersection.now();
//  max_wait_at_intersection += INTERSECTION_MAX_WAIT_TIME;
//
//  setRecoveryTime(INTERSECTION_RECOVERY_TIMEOUT);
//  clearRecoveryIndicator();
}

StTrafficLight::~StTrafficLight() {
  ChsmPlanner& planner = context<ChsmPlanner>();
  pthread_mutex_lock(&planner.traffic_light_poses_mutex_);
  planner.publish_traffic_lights_=false;
  planner.traffic_light_poses_.clear();
  pthread_mutex_unlock(&planner.traffic_light_poses_mutex_);
  delete tlm_;
}

sc::result StTrafficLight::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();
  return forward_event();
}

void StTrafficLight::generateCurvepoints(const double stop_distance, const double max_speed)
{
  ChsmPlanner& planner = context<ChsmPlanner>();
  double sv_veh_dist, mv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  //double obstacle_dist = min(nextVehicle.first, std_dist);
  planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StTrafficLightApproach
 *---------------------------------------------------------------------------*/
StTrafficLightApproach::StTrafficLightApproach(my_context ctx) : my_base(ctx), kogmo_base(std::string("StTrafficLightApproach")) {

}

StTrafficLightApproach::~StTrafficLightApproach() {

}

sc::result StTrafficLightApproach::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

    // get context data
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;
  TrafficLightManager* tlm = context<StTrafficLight>().tlm_;

  CurvePoint tl_point;
  double traffic_light_dist = topology->dist_to_next_traffic_light(NULL, &tl_point);

  planner.stop_point_ = tl_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

  // transition: replanning (vehicle is off track)
  if (topology->isOffTrack())
    return transit<StReplan>();

  // transition: replanning (route is blocked)
  if ( topology->isRouteBlocked() )
    return transit<StReplan>();

  // transition: queueing
  if ( mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist )
    return transit<StTrafficLightQueue>();

  // transition: stop at occupied traffic light
  if (hasToStop && traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT) { // && traffic_light_dist > TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StTrafficLightStop>();
  }

  if (!hasToStop && (traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive>();
  }

  double max_speed = planner.params().max_speed_traffic_light_approach;

  // generate curvepoints
  if(!hasToStop) {traffic_light_dist = std::numeric_limits<double>::infinity();}
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(traffic_light_dist, std_dist, max_speed);
  return forward_event();
}


/*---------------------------------------------------------------------------
 * StTrafficLightQueue
 *---------------------------------------------------------------------------*/
StTrafficLightQueue::StTrafficLightQueue(my_context ctx) : my_base(ctx), kogmo_base(std::string("StTrafficLightQueue"))
{
  congestionTimeout.now();
  congestionTimeout += RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner>().currentPose();
}

StTrafficLightQueue::~StTrafficLightQueue()
{
}

sc::result StTrafficLightQueue::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;
  TrafficLightManager* tlm = context<StTrafficLight>().tlm_;

  CurvePoint tl_point;
  double traffic_light_dist = topology->dist_to_next_traffic_light(NULL, &tl_point);
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

//  planner.stop_point_ = tl_point;

  // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack())
    return transit<StReplan>();

  // transition: replanning (because route is blocked)
  if ( topology->isRouteBlocked() )
    return transit<StReplan>();

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

    // transition: crosswalk approach (vehicles ahead disappeared)
  if ( traffic_light_dist < std::min(sv_veh_dist, mv_veh_dist) ) {
    return transit<StTrafficLightApproach>();
  }

  // transition: stop at crosswalk
  if (hasToStop && traffic_light_dist <= TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist >= TRAFFIC_LIGHT_DIST_THRESHOLD &&
      traffic_light_dist <= sv_veh_dist && traffic_light_dist <= mv_veh_dist) {
    return transit<StTrafficLightStop>();
  }

  if (!hasToStop && (traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive>();
  }

    // generate curvepoints
  context<StTrafficLight>().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);

  return forward_event();
}


/*---------------------------------------------------------------------------
 * StTrafficLightStop
 *---------------------------------------------------------------------------*/
StTrafficLightStop::StTrafficLightStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StTrafficLightStop"))
{
}

StTrafficLightStop::~StTrafficLightStop()
{
}

sc::result StTrafficLightStop::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;
  TrafficLightManager* tlm = context<StTrafficLight>().tlm_;

    // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) {
    return transit<StReplan>();
  }

    // transition: replanning (because route is blocked)
  if ( topology->isRouteBlocked() ) {
    return transit<StReplan>();
  }

  CurvePoint tl_point;
  double traffic_light_dist = topology->dist_to_next_traffic_light(NULL, &tl_point);

  planner.stop_point_ = tl_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

  // transition: wait at crosswalk (because we stopped already)
  if ( hasToStop && traffic_light_dist < TRAFFIC_LIGHT_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
    return transit<StTrafficLightWait>();
  }
  else if (hasToStop && traffic_light_dist == std::numeric_limits<double>::infinity()) {
    printf("WE RAN OVER A TRAFFIC LIGHT!!\n");
    return transit<StDrive>();
  }
  else if ((traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive>();
  }

    // transition: queueing (in case vehicle backed up)
  if ( traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && (mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist) ) {
    return transit<StTrafficLightQueue>();
  }

    // generate curvepoints
//  context<StTrafficLight>().generateCurvepoints(traffic_light_dist);
  if(!hasToStop) {traffic_light_dist = std::numeric_limits<double>::infinity();}
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(traffic_light_dist, std_dist, planner.params().max_speed_traffic_light_approach);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StTrafficLightStop
 *---------------------------------------------------------------------------*/
StTrafficLightWait::StTrafficLightWait(my_context ctx) : my_base(ctx), kogmo_base(std::string("StTrafficLightWait"))
{
//  stop_time.now();
//  stop_time += MIN_WAIT_STOPLINE;
}

StTrafficLightWait::~StTrafficLightWait() {
}

sc::result StTrafficLightWait::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) {return transit<StGlobalRecover>();}

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;
  TrafficLightManager* tlm = context<StTrafficLight>().tlm_;

  // Transition: Replanning (because route is blocked)
  if ( topology->isRouteBlocked() )
    return transit<StReplan>();

  // stoppen
  planner.generateStopTrajectory();

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);
  CurvePoint tl_point;
  double traffic_light_dist = topology->dist_to_next_traffic_light(NULL, &tl_point);

  // Transition: drive on intersection
  if (!hasToStop || traffic_light_dist < -.5 || traffic_light_dist == std::numeric_limits<double>::infinity()) { // && isExpired(stop_time) && !isec_man->isVehicleOnCrosswalkInFront()) {
    return transit<StDrive>();
  }

  return forward_event();
}

} // namespace vlr

