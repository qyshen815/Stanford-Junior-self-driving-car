/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_StWaitForActivation.hpp"
#include "aw_StPause.hpp"
#include "aw_StActive.hpp"

namespace vlr {
StPause::StPause(my_context ctx) :
  my_base(ctx), kogmo_base(std::string("StPause")) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  planner.robot_pose_in_pause_ = planner.currentPose();  // TODO: causes mutex lock
  //save_curvepoints = planner.curvepoints;
  planner.inPause = true;

}

StPause::~StPause() {
  context<ChsmPlanner> ().inPause = false;
  //context<ChsmPlanner>().curvepoints = save_curvepoints;
}

sc::result StPause::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
//  planner.generateStopTrajectory(true); ?!?
  planner.vehiclecmd.beeper_on = 0;
  planner.vehiclecmd.hazard_lights_on = 0;
  return forward_event();// Stephanie
  //return transit<StIntersectionApproach>();
}
sc::result StPause::react(const sc::exception_thrown&) {
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
  catch ( ... )
  {
    return forward_event();
  }
}

StPauseShortTerm::StPauseShortTerm(my_context ctx) : my_base(ctx), kogmo_base(std::string("StPauseShortTerm"))
{
  switchTime.now();
  switchTime += SWITCH_TO_LONGTERM_PAUSE;
}

StPauseShortTerm::~StPauseShortTerm()
{
}

sc::result StPauseShortTerm::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  if (isExpired(switchTime)) {
    return forward_event();// Stephanie
    //return transit<StIntersectionApproach>();
  }
  else {
    return forward_event();// Stephanie
    //return transit<StIntersectionApproach>();
  }
}

StPauseLongTerm::StPauseLongTerm(my_context ctx) : my_base(ctx), kogmo_base(std::string("StPauseLongTerm"))
{
  // TODO: auf Handbremse wechseln
}

StPauseLongTerm::~StPauseLongTerm()
{
  // TODO: Handbremse loesen
}

sc::result StPauseLongTerm::react(const EvProcess&)
{
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  return forward_event();// Stephanie
  //return transit<StIntersectionApproach>();
}
} // namespace vlr
