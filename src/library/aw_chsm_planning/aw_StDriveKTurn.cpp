/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <sla.h>

#include <aw_kogmo_math.h>

#include "aw_StDriveKTurn.hpp"
#include "aw_StPause.hpp"

namespace vlr {

#define C2_KTURN_SPEED      2.0 /*[m/s]*/
#define C2_KTURN_CHANGELANE 1
#define C2_KTURN_SETBACK    2
#define C2_KTURN_FINISH     3
int c2_circlePoints(double xc, double yc, double r, double theta_start, double theta_end, double* pointsX, double* pointsY, double* pointsTheta, int numberOfPoints);
//int c2_kturn(double delta, double r_turn_min, CurvePoints* curvepointsObj,int submaneuver);


/*---------------------------------------------------------------------------
 * StKTurn
 *---------------------------------------------------------------------------*/
StDriveKTurn::StDriveKTurn(my_context ctx)
: my_base(ctx), kogmo_base("StDriveKTurn")
{
#ifdef A_STAR_KTURN
  map_timer.now();
  map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
#else
  // get local copy of parameter set so that all three phases react with the same parameter
  radius = context<ChsmPlanner>().params().kturn_radius;
  delta = context<ChsmPlanner>().params().kturn_delta;
  switch_speed = context<ChsmPlanner>().params().kturn_switch_speed;
  switch_distance = context<ChsmPlanner>().params().kturn_switch_distance;
#endif
}

StDriveKTurn::~StDriveKTurn()
{
#ifdef A_STAR_KTURN
  // idle navigator
  ChsmPlanner& planner = context<ChsmPlanner>();
  planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
  planner.navigator_control.mode = UC_NAVI_IDLE;
#endif
}

sc::result StDriveKTurn::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // do _not_ bubble up the event, we want to go through all our phases
  return discard_event();
}

/*---------------------------------------------------------------------------
 * StDriveKTurnApproach
 *---------------------------------------------------------------------------*/
StDriveKTurnApproach::StDriveKTurnApproach(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveKTurnApproach"))
{
#ifdef A_STAR_KTURN
  // initialize navigator
  ChsmPlanner& planner = context<ChsmPlanner>();
  planner.navigator_control.mode = UC_NAVI_INITIALIZE;
  planner.kturn_approach_edge = planner.topology->current_edge_it;
#endif
}

StDriveKTurnApproach::~StDriveKTurnApproach() {
}

sc::result StDriveKTurnApproach::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // calculate distance to stop point
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;
  double kturn_dist = topology->dist_to_next_kturn();

  // switch to phase 1 when vehicle is close to kturn point
#ifdef A_STAR_KTURN
  if (kturn_dist < STOP_DIST_THRESHOLD) {
  	if (isExpired(context<StDriveKTurn>().map_timer)) { // TODO: mergecheck or at least "is there something" check

  		return transit<StDriveKTurnAStar>();
  	} else {
  		context<ChsmPlanner>().generateStopTrajectory();
  	}
  }
#else
  if (kturn_dist < STOP_DIST_THRESHOLD) {
    return transit<StDriveKTurnPhase1>();
  }
#endif
  // generate curvepoints
  context<ChsmPlanner>().generateCurvePoints(planner.params().max_speed_kturn);

  return forward_event();
}

#ifdef A_STAR_KTURN
/*---------------------------------------------------------------------------
 * StDriveKTurnAStar
 *---------------------------------------------------------------------------*/
StDriveKTurnAStar::StDriveKTurnAStar(my_context ctx)
: my_base(ctx), kogmo_base("StDriveKTurnAStar")
{
  return;

  double x,y,psi;
  ChsmPlanner& planner = context<ChsmPlanner>();
  RoutePlanner::Route::RouteEdgeList::iterator& opposite_lane_edge = context<StDriveKTurn>().opposite_lane_edge;

  // get kturn edge
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(planner.topology->current_edge_it_on_complete_mission_graph);
  std::cout << "Looking for kturn maneuvers: "<< std::endl;
  while (anno_edge_it != planner.topology->route.route.end())
  {
    if ( (*anno_edge_it)->hasAnnotation( UC_MANEUVER_U_TURN) ) {
      ++anno_edge_it;
      if(anno_edge_it != planner.topology->route.route.end()) {
        ++anno_edge_it;
        if(anno_edge_it != planner.topology->route.route.end())
          opposite_lane_edge = anno_edge_it;
      }
      break;
    }
    ++anno_edge_it;
  }

  // set end position
  RoutePlanner::AnnotatedRouteEdge* edge = *opposite_lane_edge;
  assert(edge);
  x = edge->getEdge()->fromVertex()->x();
  y = edge->getEdge()->fromVertex()->y();
  psi = atan2( edge->getEdge()->toVertex()->y() - edge->getEdge()->fromVertex()->y(),
                edge->getEdge()->toVertex()->x() - edge->getEdge()->fromVertex()->x());
//  psi = kogmo_normalize_theta(planner.robot_pose.yaw - M_PI);

  // start plannning
  planner.navigator_control.x = x;
  planner.navigator_control.y = y;
  planner.navigator_control.psi = psi;

  // add an afterburner distance to make sure you don't have to stop
  x+=AFTER_BURNER_DIST*cos(psi);
  y+=AFTER_BURNER_DIST*sin(psi);

  planner.navigator_control.mode = UC_NAVI_PARKING;
  //planner.createBoundaryLines( true, true, false, true );

  // setup perimeter for cspace, take them from lane boundaries
  NavigatorPerimeter& navigator_perimeter = planner.navigator_perimeter;

  navigator_perimeter.entry_present = false;
  navigator_perimeter.exit_present = false;

  std::cout << "nogo segments: " << planner.nogoBoundaries.size() << "\n";

  for( std::vector<kogmo_line_2Df_t>::iterator it = planner.nogoBoundaries.begin(); it!= planner.nogoBoundaries.end(); it++ )
    {
      if( (unsigned int)(navigator_perimeter.num_points) > NAVIGATOR_PERIMETER_MAX_POINTS )
	{
	  std::cout << "WAAARNING! NavigatorPerimeter::MAX_POINTS exceeded! This is very bad!\n";
	  continue;
	}
      navigator_perimeter.points[ navigator_perimeter.num_points ].x = it->x1;
      navigator_perimeter.points[ navigator_perimeter.num_points ].y = it->y1;
      navigator_perimeter.num_points++;


      navigator_perimeter.points[ navigator_perimeter.num_points ].x = it->x2;
      navigator_perimeter.points[ navigator_perimeter.num_points ].y = it->y2;
      navigator_perimeter.num_points++;
    }

  std::cout << "Navigator interface nor reimplemented\n"; // TODO
  //planner.ipc_->Publish(AWNavigatorPerimeterID, &planner.navigator_perimeter);
}

StDriveKTurnAStar::~StDriveKTurnAStar()
{
  ChsmPlanner& planner = context<ChsmPlanner>();

  NavigatorPerimeter& navigator_perimeter = planner.navigator_perimeter;
  navigator_perimeter.num_points -= 2 * planner.nogoBoundaries.size();
//  planner.ipc_->Publish(AWNavigatorPerimeterID, &planner.navigator_perimeter);
  std::cout << "Navigator interface not reimplemented\n"; // TODO
}

sc::result StDriveKTurnAStar::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();
  navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

  // blinker setzen: links
  planner.vehiclecmd.turnsignal = TURN_SIGNAL_LEFT;

  RoutePlanner::Route::RouteEdgeList::iterator current_edge_it(planner.topology->current_edge_it_on_complete_mission_graph);

  // check if vehicle is parked
  if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST) {
    planner.vehiclecmd.turnsignal = TURN_SIGNAL_NONE;
    return transit<StActive>();
  }

  // copy curvepoints if data available
  if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
    // memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints)); // TODO: substitute CurvePoints
  }

  return forward_event();
}
#else
/*---------------------------------------------------------------------------
 * StDriveKTurnPhaseBaseImpl
 *---------------------------------------------------------------------------*/

void StDriveKTurnPhaseBaseImpl::transform_curvepoints_and_define_goal(dgc_pose_t& goal, CurvePoints* curvepoints, const dgc_pose_t& start)
{
	double z=0;
	kogmo_transform_t t;
	kogmo_transform_identity(t);
	kogmo_transform_rotate_z(t, start.yaw);
	kogmo_transform_translate(t, start.x, start.y, 0);
	for(int i=0; i < curvepoints->numberValidPoints; i++) {
		kogmo_transform_point(&curvepoints->curvepoints[i].x,
				&curvepoints->curvepoints[i].y,
				&z
				,t);
		curvepoints->curvepoints[i].theta =
			kogmo_normalize_theta(curvepoints->curvepoints[i].theta + start.yaw);

	}

	// define subgoal
	int lastIndex = curvepoints->numberValidPoints-1;
	goal.x = curvepoints->curvepoints[lastIndex].x;
	goal.y = curvepoints->curvepoints[lastIndex].y;
	goal.yaw = curvepoints->curvepoints[lastIndex].theta;
}


bool StDriveKTurnPhaseBaseImpl::goal_reached(const dgc_pose_t& goal, double switch_distance, double switch_speed, const dgc_pose_t& robot_pose, double speed)
{
	return hypot(robot_pose.x - goal.x, robot_pose.y - goal.y) < switch_distance && speed < switch_speed;
}

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase1
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase1::StDriveKTurnPhase1(my_context ctx)
: my_base(ctx), kogmo_base("StDriveKTurnPhase1")
{
  // save start pose
  context<StDriveKTurn>().start = context<ChsmPlanner>().robot_pose;
	double delta = context<StDriveKTurn>().delta;
	dgc_pose_t pose = context<ChsmPlanner>().robot_pose;
	dgc_pose_t start = context<StDriveKTurn>().start;
	CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
	double dx = delta * cos(pose.yaw + PI_2);
	double dy = delta * sin(pose.yaw + PI_2);
	dgc_pose_t overall_goal = start;
	overall_goal.x+=dx;
	overall_goal.y+=dy;
	overall_goal.yaw=kogmo_normalize_theta(goal.yaw+TWOPI);


	c2_kturn(delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_CHANGELANE);
	transform_curvepoints_and_define_goal(curvepoints, start);

	// need local copies of this because StDriveKTurnPhaseBase cannot access context<>() method
	switch_distance = context<StDriveKTurn>().switch_distance;
	switch_speed = context<StDriveKTurn>().switch_speed;

}


StDriveKTurnPhase1::~StDriveKTurnPhase1()
{
}

sc::result StDriveKTurnPhase1::react(const EvProcess& event) {

	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// blinker setzen: links
	context<ChsmPlanner>().vehiclecmd.turnsignal = TURN_SIGNAL_LEFT;

	if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().v())) {
		return transit<StDriveKTurnPhase2>();
	} else {
		return forward_event();
	}

}

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase2
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase2::StDriveKTurnPhase2(my_context ctx)
: my_base(ctx), kogmo_base("StDriveKTurnPhase2")
{
	CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
	c2_kturn(context<StDriveKTurn>().delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_SETBACK);
	transform_curvepoints_and_define_goal(curvepoints, context<StDriveKTurn>().start);

	switch_distance = context<StDriveKTurn>().switch_distance;
	switch_speed = context<StDriveKTurn>().switch_speed;
}

StDriveKTurnPhase2::~StDriveKTurnPhase2()
{
}

sc::result StDriveKTurnPhase2::react(const EvProcess& event) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// blinker setzen: rechts
	context<ChsmPlanner>().vehiclecmd.turnsignal = TURN_SIGNAL_RIGHT;

	if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().currentPose().v())) {
		return transit<StDriveKTurnPhase3>();
	} else {
		return forward_event();
	}
}


/*---------------------------------------------------------------------------
 * StDriveKTurnPhase3
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase3::StDriveKTurnPhase3(my_context ctx)
: my_base(ctx), kogmo_base("StDriveKTurnPhase3")
{
	CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
	c2_kturn(context<StDriveKTurn>().delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_FINISH);
	transform_curvepoints_and_define_goal(curvepoints, context<StDriveKTurn>().start);

	switch_distance = context<StDriveKTurn>().switch_distance;
	switch_speed = context<StDriveKTurn>().switch_speed;
}

StDriveKTurnPhase3::~StDriveKTurnPhase3()
{
}

sc::result StDriveKTurnPhase3::react(const EvProcess& event) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// blinker setzen: links
	context<ChsmPlanner>().vehiclecmd.turnsignal = TURN_SIGNAL_LEFT;

	if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().currentPose().v())) {
		return transit<StActive>(); // let StActive decide what to do next
	} else {
		return forward_event();
	}
}





int c2_circlePoints(double xc, double yc, double r, double theta_start, double theta_end, double* pointsX, double* pointsY, double* pointsTheta, int numberOfPoints)
{
  double dTheta = ( theta_end - theta_start ) / ( numberOfPoints - 1.0 );
  int i;
  double phi;
  double theta;
  int direction = ((theta_start < theta_end) ? 1 : -1);

  for(i=0;i<numberOfPoints;i++)
  {

    phi = theta_start + i*dTheta;
    pointsX[i]    = xc + r*cos(phi);
    pointsY[i]    = yc + r*sin(phi);
    theta     = phi + PI/2.0*direction;
    pointsTheta[i]  = theta + floor(0.5-theta/(2*PI))*(2*PI);
  }
  return 1;
}

int c2_kturn(double delta, double r_turn_min, CurvePoints* curvepointsObj,int submaneuver)
{
  double xc1, xc2, xc3;
  double yc1, yc2, yc3;
  double theta_start1, theta_start2, theta_start3;
  double theta_end1, theta_end2, theta_end3;
  double term = sqrt( 4.0* ( r_turn_min )*( r_turn_min ) - ( 2*r_turn_min - delta )*( 2*r_turn_min - delta )/4.0 );
  double kappa = 0;
  double x[C2_CURVEPOINTS_POINTSMAX];
  double y[C2_CURVEPOINTS_POINTSMAX];
  double theta[C2_CURVEPOINTS_POINTSMAX];
  int i;
  int sgnSpeed=0;

   if ( r_turn_min <= 0) return 0;

  switch ( submaneuver )
  {
  case C2_KTURN_CHANGELANE:
    xc1 = 0;
    yc1 = r_turn_min;
    theta_start1 = -PI/2.0;
    theta_end1 = atan2(-(r_turn_min-delta/2.0), term);
    //fprintf(stderr,"theta_start1 %f\n",theta_start1);
    //fprintf(stderr,"theta_end1 %f\n",theta_end1);
    if(!c2_circlePoints(xc1, yc1, r_turn_min, theta_start1, theta_end1, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    kappa = (1.0/r_turn_min);
    sgnSpeed = 1;
    break;

  case C2_KTURN_SETBACK:
    xc2 = term;
    yc2 = delta/2.0;
    theta_start2 = atan2( r_turn_min-delta/2.0, -term );
    theta_end2 = theta_start2 + 2*atan( (r_turn_min-delta/2.0) /term );
    //fprintf(stderr,"theta_start2 %f\n",theta_start2);
    //fprintf(stderr,"theta_end2 %f\n",theta_end2);
    if(!c2_circlePoints(xc2, yc2, r_turn_min, theta_start2, theta_end2, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    for(i=0;i<C2_CURVEPOINTS_POINTSMAX;++i) theta[i]+=PI;
    kappa = -(1.0/r_turn_min);
    sgnSpeed = -1;
    break;

  case C2_KTURN_FINISH:
    xc3 = 0;
    yc3 = -(r_turn_min-delta);
    theta_start3 = atan2( r_turn_min-delta/2.0, term);
    theta_end3   = PI/2.0;
    if(!c2_circlePoints(xc3, yc3, r_turn_min, theta_start3, theta_end3, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    kappa = (1.0/r_turn_min);
    sgnSpeed = 1;
    break;
  }
  for(i=0;i<C2_CURVEPOINTS_POINTSMAX;i++)
  {
    curvepointsObj->curvepoints[i].x = x[i];
    curvepointsObj->curvepoints[i].y = y[i];
    curvepointsObj->curvepoints[i].theta = theta[i];
    curvepointsObj->curvepoints[i].kappa = kappa;
  }

  curvepointsObj->state = C2_CURVEPOINTS_STATE_STOP_DISTANCE;
  curvepointsObj->velocity_desired = sgnSpeed*C2_KTURN_SPEED;
  curvepointsObj->numberValidPoints = C2_CURVEPOINTS_POINTSMAX;
  return 1;
}

#endif /*A_STAR_KTURN */

} // namespace vlr
