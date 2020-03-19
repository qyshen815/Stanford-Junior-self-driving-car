/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_StWaitForActivation.hpp"
#include "aw_StError.hpp"

namespace vlr {

StError::StError(my_context ctx) :
  my_base(ctx), kogmo_base(std::string("StError")) {
  // TODO: recreate as much as possible: topology, vehiclemananger, routesampler - or restart whole program?
}

StError::~StError() {
}

sc::result StError::react(const EvProcess&) {
  if (detectedErrornousTransitions()) std::cout << "oooops" << std::endl;

  ChsmPlanner& planner = context<ChsmPlanner> ();
  // set velocity to zero
  planner.generateStopTrajectory();
  planner.velocity_desired_ = 0; // park mode
  planner.vehiclecmd.beeper_on = 1;
  planner.vehiclecmd.hazard_lights_on = 1;
  planner.vehiclecmd.warninglights_on = 1; // indicate error
  return forward_event();
}

} // namespace vlr
