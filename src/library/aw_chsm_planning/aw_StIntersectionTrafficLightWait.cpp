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

namespace vlr {

StIntersectionTrafficLightWait::StIntersectionTrafficLightWait(my_context ctx) :
  my_base(ctx), kogmo_base(std::string("StIntersectionTrafficLightWait")) {
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  assert(isec_man);
  isec_man->stoppedOnStopline();
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionTrafficLightWait::~StIntersectionTrafficLightWait() {
}

sc::result StIntersectionTrafficLightWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get global data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology;
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;
  assert(isec_man);

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }

  // measure progress in the parent state
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery())) {
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // generate stop trajectory
  planner.generateStopTrajectory();

  // set turn signal
  planner.vehiclecmd.turnsignal = context<StIntersection> ().turnDirection;

////     Transition: drive in intersection
////
////     We will go if it's our turn and rely on vehicle prediction
////     to not run into anybody
////     if (isec_man->hasRightOfWay() && !context<StIntersection>().isec_man->hasToStop()) {
  if (isec_man->hasRightOfWay() && !context<StIntersection>().isec_man->hasToStop() && !isec_man->isVehicleOnIntersectionInFront()) {
    return transit<StIntersectionDriveInside> ();
  }


  return forward_event();
}

} // namespace vlr
