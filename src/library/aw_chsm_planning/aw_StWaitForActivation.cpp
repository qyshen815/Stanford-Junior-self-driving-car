/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_StWaitForActivation.hpp"
#include "aw_StActive.hpp"
#include "aw_StPause.hpp"
#include "aw_StReplan.hpp"

namespace vlr {

StWaitForActivation::StWaitForActivation(my_context ctx) :
  my_base(ctx), kogmo_base(std::string("StWaitForActivation")), moved(false) {
  waitUntil.now();
  waitUntil += MIN_WAIT_ACTIVATION;
  ChsmPlanner& planner = context<ChsmPlanner> ();
  if (planner.distance(planner.robot_pose_in_pause_) > TRIGGER_DIST_CLEAR_HISTORY) {
    planner.addMessage("car moved while in pause -> clearing history");
    context<ChsmPlanner> ().clear_deep_history<StActive, 0> ();
    moved = true;
  }
  else {
    planner.addMessage("resuming with history");
  }
  context<ChsmPlanner> ().velocity_desired_ = 0;
}

StWaitForActivation::~StWaitForActivation() {
}

sc::result StWaitForActivation::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology;

  // set velocity to 0 -> park mode
  context<ChsmPlanner> ().generateStopTrajectory();

  context<ChsmPlanner> ().vehiclecmd.beeper_on = 1;
  context<ChsmPlanner> ().vehiclecmd.hazard_lights_on = 1;

  if (isExpired(waitUntil)) {

    // Transition: Replanning (because ego vehicle is off track)
    if (topology->isOffTrack() || moved) return transit<StReplan> ();

    // Transition: History restoren
    return transit<StActiveHistory> ();

  }
  else {
    return forward_event();
  }
}
sc::result StWaitForActivation::react(const sc::exception_thrown&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  try {
    throw ;
  }
  /* we can catch special exceptions here and handle them
   catch ( const std::runtime_error & )
   {
   // only std::runtime_errors will lead to a transition
   // to Defective ...
   return transit< StError >();
   }*/
  catch ( ... ) {
    return forward_event();
  }
}

} // namespace vlr
