/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_StPause.hpp"
#include "aw_StDrive.hpp"
#include "aw_StStop.hpp"

namespace vlr {
  /*---------------------------------------------------------------------------
   * StStop
   *---------------------------------------------------------------------------*/
  StStop::StStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StStop"))
  {
  }

  StStop::~StStop() {
  }

  sc::result StStop::react(const EvProcess&) {
    return forward_event();
  }

  sc::result StStop::react(const EvDrive&) {
    if (detectedErrornousTransitions()) return transit<StGlobalRecover>();
    return transit<StDrive>();
  }

  /*---------------------------------------------------------------------------
   * StStopping
   *---------------------------------------------------------------------------*/
  StStopping::StStopping(my_context ctx): my_base(ctx), kogmo_base(std::string("StStopping"))
  {
  }

  StStopping::~StStopping() {
  }

  sc::result StStopping::react(const EvProcess&) {
    if (detectedErrornousTransitions()) return transit<StGlobalRecover>();
    // calculate distance to stop point
    double goal_dist = context<ChsmPlanner>().topology->dist_to_mission_end();

    // generate curvepoints
    context<ChsmPlanner>().generateCurvePoints(context<ChsmPlanner>().params().max_speed_goal);

    // transitions
    if( goal_dist < STOP_DIST_THRESHOLD &&
        context<ChsmPlanner>().currentPose().v() < STOP_SPEED_THRESHOLD ) {
      return transit<StPause>();
    }

    return forward_event();
  }

} // namespace vlr
