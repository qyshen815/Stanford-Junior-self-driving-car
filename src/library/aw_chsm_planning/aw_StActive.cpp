/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <aw_RndfVertex.h>
#include <aw_kogmo_math.h>

#include "aw_StActive.hpp"
#include "aw_StPause.hpp"
#include "aw_StDriveKTurn.hpp"
#include "aw_StError.hpp"

namespace vlr  {

StActive::StActive(my_context ctx) : my_base(ctx), kogmo_base(std::string("StActive"))
{
	setRecoveryTime(GLOBAL_RECOVERY_TIMEOUT);
	clearRecoveryIndicator();
}

StActive::~StActive()
{
}

sc::result StActive::react(const EvProcess&)
{
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }
	return forward_event();
}

sc::result StActive::react(const EvAfterProcess&)
{
  // TODO: check when this is called...
  printf("StActive::%s\n", __FUNCTION__);
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }
	context<ChsmPlanner>().vehiclecmd.beeper_on = 1;
	context<ChsmPlanner>().vehiclecmd.hazard_lights_on = 1;
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }
	if (detectedErrornousTransitions()) return transit<StError>();

	if (checkRecovery()) {
		return transit<StGlobalRecover>();
	}

	return forward_event();
}

sc::result StActive::react(const sc::exception_thrown&)
{
	detectedErrornousTransitions();
    try
    {
      throw;
    }
    /* we can catch special exceptions here and handle them
    catch ( const std::runtime_error & )
    {
      // only std::runtime_errors will lead to a transition
      // to Defective ...
      return transit< StError >();
    ChsmPlanner& planner = context<ChsmPlanner>();
    if(planner.emergencyStopInitiated()) {
      return transit<StPause>();
    }
    }*/
    catch ( ... )
    {
      return forward_event();
    }
}


StGlobalRecover::StGlobalRecover(my_context ctx) : my_base(ctx), kogmo_base(std::string("StGlobalRecover")),
  done(false), checkpoint(0)
{
  ChsmPlanner& planner = context<ChsmPlanner>();

  // extract all parking spots
  Topology* topology = planner.topology;

  topology->getNextCheckpointIter();

  done = topology->next_check_point_it == topology->checkpoints.end();

  if (!done) {
  	checkpoint = *topology->next_check_point_it;

    double x,y,psi;

    x = checkpoint->x();
    y = checkpoint->y();

    if (checkpoint->numInEdges()) {
    	RndfVertex* v = (*(checkpoint->getInEdges().begin()))->fromVertex();
    	psi = atan2(v->y() - y, v->x() - x);
    } else if (checkpoint->numOutEdges()) {
    	RndfVertex* v = (*(checkpoint->getOutEdges().begin()))->toVertex();
    	psi = atan2(y - v->y(), x - v->x());
    } else {
    	psi = 0;
    }

    // add an afterburner distance to make sure you end up outside the zone
//    x+=AFTER_BURNER_DIST*cos(psi);
//    y+=AFTER_BURNER_DIST*sin(psi);

    // start plannning
    planner.navigator_control.x = x;
    planner.navigator_control.y = y;
    planner.navigator_control.psi = psi;
    planner.navigator_control.mode = UC_NAVI_PARKING;
  }
}

StGlobalRecover::~StGlobalRecover()
{
  // idle navigator
  ChsmPlanner& planner = context<ChsmPlanner>();
  planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
  planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StGlobalRecover::react(const EvAfterProcess&)
{
	detectedErrornousTransitions();
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;
  navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

  // check if vehicle is at exit
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

  if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST) {
    return transit<StDrive>();
  }

  // copy curvepoints if data available
  if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
    //memcpy(planner.curvepoints_, &navigator_feedback->curvepoints,sizeof(CurvePoints)); // TODO: substitute CurvePoints
  }

  return forward_event();
}

} // namespace vlr
