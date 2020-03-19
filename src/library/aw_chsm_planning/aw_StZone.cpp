/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <aw_kogmo_math.h>
#include <aw_graph_tools.hpp>
//#include <rtdb_path.hpp>

#include "aw_StZone.hpp"
#include "aw_StPause.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StZone
 *---------------------------------------------------------------------------*/
StZone::StZone(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZone")), no_perimeter_points( 0 )
{
  ChsmPlanner& planner = context<ChsmPlanner>();

  // extract all parking spots
  Topology* topology = planner.topology;
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);
  RoutePlanner::Route::RouteEdgeList::iterator scan_for_zone_it( anno_edge_it );

  RoutePlanner::AnnotatedRouteEdge* entry_edge = topology->next_edge_with_maneuver( UC_MANEUVER_ZONE_ENTRY );
  RoutePlanner::AnnotatedRouteEdge* exit_edge = topology->next_edge_with_maneuver( UC_MANEUVER_ZONE_EXIT, 100000 );

  std::string entry_point_name = "";
  if (entry_edge) entry_point_name = entry_edge->getEdge()->toVertex()->name();
  if (exit_edge) entry_point_name = exit_edge->getEdge()->fromVertex()->name();

  /*
  bool found_exit = false;
  std::string exit_point_name;
  if( exit_edge )
    {
      found_exit = true;
      exit_point_name = exit_edge->getEdge()->fromVertex()->name();
    }
  */
  /*
  while( !( ( *scan_for_zone_it )->getEdge()->isZoneEdge() ) )
    {
      std::cout << "schwupp die zone...\n";
      scan_for_zone_it++;
    }

  std::string entry_point_name = (*scan_for_zone_it)->getEdge()->toVertex()->name();
  std::cout << "entering zone @ point: " << entry_point_name << "\n";

  RoutePlanner::Route::RouteEdgeList::iterator scan_for_zone_end_it( scan_for_zone_it );

  while(  ( ( *scan_for_zone_end_it )->getEdge()->isZoneEdge() ) )
    {
      std::cout << "schwupp die zone weiter...\n";
      scan_for_zone_end_it++;
      if( scan_for_zone_end_it == topology->complete_mission_graph.end() )
	break;
      if( !(( *scan_for_zone_end_it )->getEdge() ) )
	break;

    }
  std::cout << "schwupped!";
  scan_for_zone_end_it--;

  std::string exit_point_name = (*scan_for_zone_end_it)->getEdge()->toVertex()->name();
  std::cout << "will leave zone @ point: " << exit_point_name << "\n";

  */

   // TODO: make independent of rndf lib...
  // put perimeter points of zone entered into db
  const rndf::Perimeter* current_perimeter = topology->roadNetwork().perimeterPoints().find( entry_point_name )->second->getPerimeter();

  /*
  const rndf::PerimeterPoint* entry_point = topology->roadNetwork()->perimeterPoints().find( entry_point_name )->second;
  const rndf::PerimeterPoint* exit_point = topology->roadNetwork()->perimeterPoints().find( exit_point_name )->second;
  */

  NavigatorPerimeter& navigator_perimeter_struct = planner.navigator_perimeter;
  //  navigator_perimeter_struct.num_points = 0;

  if( entry_edge )
    {
      navigator_perimeter_struct.entry.x = entry_edge->getEdge()->toVertex()->x();
      navigator_perimeter_struct.entry.y = entry_edge->getEdge()->toVertex()->y();
      navigator_perimeter_struct.entry_present = true;
    }
  else
    navigator_perimeter_struct.entry_present = false;

  if( exit_edge )
    {
      navigator_perimeter_struct.exit.x = exit_edge->getEdge()->fromVertex()->x();
      navigator_perimeter_struct.exit.y = exit_edge->getEdge()->fromVertex()->y();
      navigator_perimeter_struct.exit_present = true;
    }
  else
    navigator_perimeter_struct.exit_present = false;

  // this is a bag of line segments now (no longer a line loop) so most points are double, now
  navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].x = current_perimeter->perimeterPoints().front()->x();
  navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].y = current_perimeter->perimeterPoints().front()->y();
  navigator_perimeter_struct.num_points++;
  no_perimeter_points++;

  for( rndf::TPerimeterPointVec::const_iterator it = current_perimeter->perimeterPoints().begin(); it != current_perimeter->perimeterPoints().end(); it++ )
    {
      std::cout << "perimeter point: " << (*it)->x() << " " << (*it)->y() << "\n";
      navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].x = (*it)->x();
      navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].y = (*it)->y();
      navigator_perimeter_struct.num_points++;
      no_perimeter_points++;
      navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].x = (*it)->x();
      navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].y = (*it)->y();
      navigator_perimeter_struct.num_points++;
      no_perimeter_points++;
      if( navigator_perimeter_struct.num_points >= (int)NAVIGATOR_PERIMETER_MAX_POINTS ) continue; // pray this will never happen!
    }

  navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].x = current_perimeter->perimeterPoints().front()->x();
  navigator_perimeter_struct.points[ navigator_perimeter_struct.num_points ].y = current_perimeter->perimeterPoints().front()->y();
  navigator_perimeter_struct.num_points++;
  no_perimeter_points++;

  std::cout << "aliased perimeter has " << planner.navigator_perimeter.num_points << " pts, non-aliased has " << planner.navigator_perimeter.num_points << "\n";

  std::cout << "Navigator interface nor reimplemented\n"; // TODO
  //planner.ipc_->Publish(AWNavigatorPerimeterID, &planner.navigator_perimeter);

  while (anno_edge_it != topology->route.route.end())
  {
	  // Park Maneuver adden
	  if ( (*anno_edge_it)->hasAnnotation( UC_MANEUVER_PARKING) ) {
		  assert( (*anno_edge_it)->getEdge()->isZoneEdge() );
		  zone_maneuvers.push_back( *anno_edge_it );
	  }
	  // Exit Maneuver merken
	  if ( (*anno_edge_it)->hasAnnotation( UC_MANEUVER_ZONE_EXIT) ) {
		  assert( (*anno_edge_it)->getEdge()->isZoneEdge() );
		  zone_maneuvers.push_back( *anno_edge_it );
		  break;
	  }
	  // Mission End adden
	  if ( (*anno_edge_it)->hasAnnotation( UC_MANEUVER_GOAL_REACHED) ) {
		  assert( (*anno_edge_it)->getEdge()->isZoneEdge() );
		  zone_maneuvers.push_back( *anno_edge_it );
		  break;
	  }
	  ++anno_edge_it;
  }
}

StZone::~StZone() {

  context<ChsmPlanner>().navigator_perimeter.entry_present = false;
  context<ChsmPlanner>().navigator_perimeter.exit_present = false;
  // idle navigator
  ChsmPlanner& planner = context<ChsmPlanner>();
  planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
  planner.navigator_control.mode = UC_NAVI_IDLE;
  planner.navigator_perimeter.num_points -= no_perimeter_points;
  std::cout << "Navigator interface nor reimplemented\n"; // TODO
  //planner.ipc_->Publish(AWNavigatorPerimeterID, &planner.navigator_perimeter);
}

sc::result StZone::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneApproach
 *---------------------------------------------------------------------------*/
StZoneApproach::StZoneApproach(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneApproach"))
{

}

StZoneApproach::~StZoneApproach()
{

}

sc::result StZoneApproach::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // Daten holen
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology;

  // Abstände berechnen
  double zone_dist = topology->dist_to_next_zone();
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // Transition: Intersection Drive
  if ( fabs(zone_dist) <= 1.0 )
    return transit<StZoneEntering>();

  // generate curvepoints
  double stop_distance = zone_dist;
  double follow_distance = std::min(sv_veh_dist,mv_veh_dist);
  planner.generateCurvePoints(stop_distance,follow_distance, planner.params().max_speed_enter_zone);

  // initialize plannning
  planner.navigator_control.mode = UC_NAVI_INITIALIZE;

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneEntering
 *---------------------------------------------------------------------------*/
StZoneEntering::StZoneEntering(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneEntering"))
{

}

StZoneEntering::~StZoneEntering()
{
}

sc::result StZoneEntering::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone>().zone_maneuvers;

  if ( zone_maneuvers.empty() )
    return transit<StDrive>();
  else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_GOAL_REACHED ) )
    return transit<StZoneParking>();
  else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_PARKING ) )
    return transit<StZoneParking>();
  else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_ZONE_EXIT ) )
    return transit<StZoneDriveToExit>();
  else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_TRAVEL ) )
    return transit<StZoneDriveToExit>();
  else
	  assert(false);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneParking
 *---------------------------------------------------------------------------*/
StZoneParking::StZoneParking(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneParking"))
{
  double x,y,psi;
  ChsmPlanner& planner = context<ChsmPlanner>();
  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone>().zone_maneuvers;

  // get parking spot
  assert( ! zone_maneuvers.empty() );
  RoutePlanner::AnnotatedRouteEdge* edge = zone_maneuvers[0];
  x = edge->getEdge()->toVertex()->x();
  y = edge->getEdge()->toVertex()->y();
  psi = atan2(edge->getEdge()->toVertex()->y() - edge->getEdge()->fromVertex()->y(),
              edge->getEdge()->toVertex()->x() - edge->getEdge()->fromVertex()->x());

  // subtract front bumper distance, because we want to park the car
  // with the front bumber on this waypoint
  x-= (FRONT_BUMPER_DELTA-1.5)*cos(psi);
  y-= (FRONT_BUMPER_DELTA-1.5)*sin(psi);

  // start plannning
  planner.navigator_control.x = x;
  planner.navigator_control.y = y;
  planner.navigator_control.psi = psi;
  planner.navigator_control.mode = UC_NAVI_PARKING;
}

StZoneParking::~StZoneParking()
{
}

sc::result StZoneParking::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

//  ChsmPlanner& planner = context<ChsmPlanner>();
  navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

  // check if vehicle is parked
  if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST) {
    return transit<StZoneParked>();
  }

  // copy curvepoints if data available
  if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
    // memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints)); // TODO: substitute CurvePoints
  }

  // Transition: Recovery Mode
  if (checkRecovery()) {
    return transit<StZoneRecover>();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneParked
 *---------------------------------------------------------------------------*/
StZoneParked::StZoneParked(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneParked"))
{
  stop_time.now();
  stop_time += MIN_WAIT_PARKED;
}

StZoneParked::~StZoneParked() {
}

sc::result StZoneParked::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  if(isExpired(stop_time))
  {
	  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone>().zone_maneuvers;
	  assert( !zone_maneuvers.empty() );

	  // erase previous maneuver
	  zone_maneuvers.erase( zone_maneuvers.begin() );

	  if ( zone_maneuvers.empty() )
      return transit<StDrive>();
    else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_GOAL_REACHED ) )
      return transit<StZoneParking>();
    else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_PARKING ) )
      return transit<StZoneParking>();
    else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_ZONE_EXIT ) )
      return transit<StZoneDriveToExit>();
    else if ( (*zone_maneuvers.begin())->hasAnnotation( UC_MANEUVER_TRAVEL ) )
      return transit<StZoneDriveToExit>();
    else
      assert(false);
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneDriveToExit
 *---------------------------------------------------------------------------*/
StZoneDriveToExit::StZoneDriveToExit(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneDriveToExit"))
{
  double x,y,psi;
  ChsmPlanner& planner = context<ChsmPlanner>();

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone>().zone_maneuvers;

  // get zone exit
  assert( ! zone_maneuvers.empty() );
  RoutePlanner::AnnotatedRouteEdge* edge = (*zone_maneuvers.begin());

  x = edge->getEdge()->fromVertex()->x();
  y = edge->getEdge()->fromVertex()->y();
  psi = atan2(edge->getEdge()->toVertex()->y() - edge->getEdge()->fromVertex()->y(),
              edge->getEdge()->toVertex()->x() - edge->getEdge()->fromVertex()->x());

  // add an afterburner distance to make sure you end up outside the zone
  x+=AFTER_BURNER_DIST*cos(psi);
  y+=AFTER_BURNER_DIST*sin(psi);

  // start plannning
  planner.navigator_control.x = x;
  planner.navigator_control.y = y;
  planner.navigator_control.psi = psi;
  planner.navigator_control.mode = UC_NAVI_PARKING;
  planner.navigator_feedback_->drive_state = UC_NAVI_DRIVE_UNDEFINED;

}

StZoneDriveToExit::~StZoneDriveToExit() {
}

sc::result StZoneDriveToExit::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

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
    //memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints)); // TODO: substitute CurvePoints
  }

  // Transition: Recovery Mode
  if (checkRecovery()) {
    return transit<StZoneRecover>();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneRecover
 *---------------------------------------------------------------------------*/
StZoneRecover::StZoneRecover(my_context ctx) : my_base(ctx), kogmo_base(std::string("StZoneRecover"))
{
}

StZoneRecover::~StZoneRecover() {
}

sc::result StZoneRecover::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology;

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone>().zone_maneuvers;

  if(zone_maneuvers.size() > 1)
  {
    planner.addMessage("Resuming to next parking maneuver.");
    if (!zone_maneuvers.empty())
      return transit<StZoneParked>();
  }

  // add a new maneuver
  planner.addMessage("Adding a new maneuver to next waypoint outside the zone.");
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);
  while (anno_edge_it != topology->route.route.end()) {
    // add next travel maneuver
    if ( (*anno_edge_it)->hasAnnotation( UC_MANEUVER_TRAVEL) ) {
      zone_maneuvers.push_back( *anno_edge_it );
      return transit<StZoneParked>();
    }
  }

  // if nothing helps...global recover! :-(
  return transit<StGlobalRecover>();
}

} // namespace vlr
