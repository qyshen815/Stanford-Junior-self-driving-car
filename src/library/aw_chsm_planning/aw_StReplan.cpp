/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <iostream>
#include <aw_kogmo_math.h>
#include "aw_StPause.hpp"
#include "aw_StReplan.hpp"
#include "aw_StStop.hpp"
#include "aw_match_to_graph.hpp"

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[StReplan] "<< str << std::endl

/*---------------------------------------------------------------------------
 * StReplan
 *---------------------------------------------------------------------------*/
StReplan::StReplan(my_context ctx) : my_base(ctx), kogmo_base(std::string("StReplan"))
{
}

StReplan::~StReplan()
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.stop_before_replanning = true;
	planner.forced_start_edge = NULL;
}

sc::result StReplan::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return forward_event();

}

/*---------------------------------------------------------------------------
 * StReplanStop
 *---------------------------------------------------------------------------*/
StReplanStop::StReplanStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StReplanStop"))
{
}

StReplanStop::~StReplanStop() {
}

sc::result StReplanStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();


	if (planner.currentPose().v() < 0.5 || ! planner.stop_before_replanning ) {
	  return(transit<StReroute>());
	} else {
		planner.generateCurvePoints(0.001);
	}
	return forward_event();

}

/*---------------------------------------------------------------------------
 * StReroute
 *---------------------------------------------------------------------------*/
StReroute::StReroute(my_context ctx) : my_base(ctx), kogmo_base(std::string("StReroute"))
{
}

StReroute::~StReroute() {
}

sc::result StReroute::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	TRACE("************** Replanning now !!! ***********************");

	// Daten holen
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;

	// Auf Missionsende prüfen
	if ( topology->route_is_finished() )
		return (transit<StPause>());

	//==================== Beste Kante ermitteln ==============================

	// Beste Kante und nächsten Checkpoint ermitteln
	topology->getBestReplanningEdge();
	deque<RndfVertex*>& checkpoints = topology->checkpoints;
	deque<RndfVertex*>::iterator cp_it = topology->next_check_point_it;
	//	deque<RndfVertex*>::iterator cp_it = topology->getNextCheckpointIter();

	// Checkpoints ausgeben
	TRACE("Checkpoints:");
	int i=0;
	for (deque<RndfVertex*>::iterator it = checkpoints.begin(); it != checkpoints.end(); ++it, ++i)
		TRACE("  "<< i << ".  "<< (*it)->name() << ( it == topology->next_check_point_it ? "  <-- next Checkpoint" : "") );

	if ( cp_it == topology->checkpoints.end() ) {
		TRACE("-> kein Checkpoint mehr Uebrig => Stop");
		return transit<StPause>(); // route is probably broken, so we don't drive it till the end
	}

	RndfEdge* best_edge = ( planner.forced_start_edge ? planner.forced_start_edge : topology->best_alternative_edge );
	while ( ! best_edge && cp_it != checkpoints.end() )
	{
		topology->getBestReplanningEdge();
		best_edge = topology->best_alternative_edge;

		// Nächsten Checkpoint skippen da nicht erreichbar
		if ( ! best_edge ) {
			TRACE("  -> Naechster Checkpoint nicht erreichbar (skip)");
			cp_it = checkpoints.erase( cp_it );
			if ( cp_it != checkpoints.end() )
				TRACE("  -> Naechster Checkpoint ist jetzt "<< (*cp_it)->name());
			else
				TRACE("  -> Keine Checkpoints mehr uebrig");
		}
	}

	// Transition: Stop (falls keine Kante in Frage kommt)
	//	assert( best_edge );
	if ( ! best_edge ) {
		TRACE("=> Es gibt keine Kante die in Frage kommt -> Stop");
		return transit<StPause>();
	}

	TRACE("Beste Kante: "<< best_edge->name());


	//==================== Nächsten Checkpoint ermitteln ======================

	//	// Abgefahrene Checkpoints und Edges aus der Route schmeißen
	//	deque<RndfVertex*>& checkpoints = topology->checkpoints;
	////	deque<RndfVertex*>::iterator cp_it = topology->next_check_point_it;
	//	deque<RndfVertex*>::iterator cp_it = topology->getNextCheckpointIter();
	//	assert( cp_it != checkpoints.end() );
	cp_it = checkpoints.erase( checkpoints.begin(), cp_it );
	//	while ( (*edge_it)->getEdge()->toVertex() != *cp_it ) ++edge_it;
	//	route.erase( route.begin(), ++edge_it );
	checkpoints.erase( checkpoints.begin(), cp_it );


	//==================== Neue Route generieren ==============================

	// Neue Gesamtroute generieren und best_edge einfügen
	RoutePlanner::Route new_route;
	new_route.addEdge( best_edge );

	// Neuen Routenabschnitt berechen
	RndfGraph::EdgeList* edgeList = topology->complete_graph->searchPath( best_edge->toVertex(), *cp_it );
	assert(edgeList);
	//	TRACE("Neue Route von der BestEdge zum nächsten Checkpoint:");
	//	for (RndfGraph::EdgeList::iterator it = edgeList->begin(); it != edgeList->end(); ++it) {
	//		TRACE("  "<< (*it)->name());
	//	}

	// neue geplante Strecke einfügen
	new_route.addEdges( edgeList );
	delete edgeList; // Speicher freigeben

	// Alle nachfolgenden Routenabschnitte auf ihre Fahrbarkeit prüfen und gegebenfalls
	// neuplanen (zB wegen Straßenblockaden)
	// BETTER Nur die Abschnitte neu planen die nicht mehr befahrbar sind
	for (cp_it = checkpoints.begin(); cp_it + 1 != checkpoints.end(); )
	{
		deque< RndfVertex* >::iterator n_it = cp_it + 1;

		// Route zwischen den beiden Checkpoints planen
		TRACE("Streckenabschnitt planen von "<< (*cp_it)->name() <<" nach "<< (*n_it)->name() );
		RndfGraph::EdgeList* edgeList = topology->complete_graph->searchPath( *cp_it, *n_it );

		// Falls für einen Streckenabschnitt keine Route gefunden wurde Blockaden vergessen
		if ( ! edgeList ) {
			TRACE("-> keinen Weg gefunden => Blocken ignorieren und erneut planen");
			edgeList = topology->complete_graph->searchPath( *cp_it, *n_it, true );
		}
		assert( edgeList ); // Punkt ist nicht erreichbar

		// Checkpoint überspringen falls keine Route planbar
		if ( ! edgeList ) {
			TRACE("-> wieder keinen Weg gefunden => Checkpoint überspringen");
			checkpoints.erase( n_it );
			continue;
		}

		// Kanten in die Route übernehmen
		new_route.addEdges( edgeList );
		delete edgeList;

		++cp_it;
	}

	// Neue Route annotieren
	// CHECK ob die Member Vars der Route aktualisiert werden müssen
	new_route.init();
	new_route.annotateRoute();


	//==================== Topology aktualisieren =============================

	// alte route loeschen
	for (Route::RouteEdgeList::iterator anno_it = topology->complete_mission_graph.begin(); anno_it != topology->complete_mission_graph.end(); ++anno_it) {
		delete *anno_it;
	}

	// Neue Route in Topology übertragen
	topology->route.route = new_route.route;
	topology->complete_mission_graph = new_route.route;
	topology->current_edge_it = topology->route.route.begin();
	topology->current_edge_it_on_complete_mission_graph = topology->complete_mission_graph.begin();

	// Visited Markierungen aller Edges löschen
	for (RndfGraph::EdgeMap::iterator it = topology->complete_graph->edgeMap.begin(); it != topology->complete_graph->edgeMap.end(); ++it) {
		it->second->visited = false;
	}

	// Ego Vehicle neu matchen und interne Datenstrukturen der Topology zu aktualisieren
	// Aufrufe müssen in dieser Reihenfolge stattfinden damit das Matching richtig funktioniert
	topology->extract_relevant_part_of_route();
	topology->handle_ego_vehicle_update();

	// BestEdge auf NULL setzen um neue BestEdgeSuche bei erneuten Replanning zu garantieren
	topology->best_alternative_edge = NULL;


	//==================== Transitions durchführen ============================

	// Transition: GetBackOnTRack
	if ( ego_veh.distToMatchedEdge() > TRIGGER_DIST_GET_BACK_ON_TRACK || ego_veh.angleToMatchedEdge() > M_PI_4)
		return transit<StGetBackOnTrack>();

	// Transition: Drive
	// BETTER Gematchte Kante analysieren entsprechende Transitions durchführen
	return transit<StDrive>();

	//	return transit<StPause>();
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrack
 *---------------------------------------------------------------------------*/
StGetBackOnTrack::StGetBackOnTrack(my_context ctx) : my_base(ctx), kogmo_base(std::string("StGetBackOnTrack"))
{
	map_timer.now();
	map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
}

StGetBackOnTrack::~StGetBackOnTrack()
{
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StGetBackOnTrack::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrackPrepare
 *---------------------------------------------------------------------------*/
StGetBackOnTrackPrepare::StGetBackOnTrackPrepare(my_context ctx) : my_base(ctx), kogmo_base(std::string("StGetBackOnTrackPrepare"))
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.addMessage("Stop for navigation");
}

StGetBackOnTrackPrepare::~StGetBackOnTrackPrepare()
{
}

sc::result StGetBackOnTrackPrepare::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	//	return transit<StDrive>();
	//	return transit<StPause>();

	ChsmPlanner& planner = context<ChsmPlanner>();

	if (isExpired(context<StGetBackOnTrack>().map_timer)) {
		return transit<StGetBackOnTrackAStar>();
	} else {
		planner.generateStopTrajectory();
		return forward_event();
	}
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrackAStar
 *---------------------------------------------------------------------------*/
StGetBackOnTrackAStar::StGetBackOnTrackAStar(my_context ctx) : my_base(ctx), kogmo_base(std::string("StGetBackOnTrackAStar"))
{

	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;

	assert( ! topology->route_is_finished() );
	RndfEdge* edge = (*topology->complete_mission_graph.begin())->getEdge();

	double x, y, psi;

	//	bool same_dir = topology->ego_vehicle.angleToEdge( edge ) > M_PI_2;
	//	Point_2 entry_point;
	//	if ( same_dir ) {
	//		entry_point = edge->toVertex()->point();
	//	} else {
	//		entry_point = edge->fromVertex()->point();
	//	}
	//	x = entry_point.x();
	//	y = entry_point.y();

	// Entry Point berechnen
	Point_2 pp = edge->getLine().projection( topology->ego_vehicle.point() );
	Vector_2 nvec = edge->getVector();
	nvec = nvec / sqrt( nvec.squared_length() );
	pp = pp + nvec * 12.;
	Segment_2 seg = edge->getSegment();
	if ( squared_distance( seg, pp ) > 0.001 ) {
		if ( squared_distance( pp, edge->fromVertex()->point() ) < squared_distance( pp, edge->toVertex()->point() ) )
			pp = edge->fromVertex()->point();
		else
			pp = edge->toVertex()->point();
	}

	// Prüfen ob er nicht zu nah an einer blockierten Kante liegt
	if ( sqrt( squared_distance( pp, edge->fromVertex()->point() ) ) < 5. ) {
		bool bl_edge = false;
		for (set< RndfEdge* >::iterator it = edge->fromVertex()->getInEdges().begin(); it != edge->fromVertex()->getInEdges().end(); ++it) {
			if ( (*it)->isBlockedEdge() )
				bl_edge = true;
		}
		if ( bl_edge )
			pp = edge->toVertex()->point();
	}

	// Endpunkt setzen
	x = pp.x();
	y = pp.y();
  psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(),
      edge->toVertex()->x() - edge->fromVertex()->x());

	planner.enable_road_boundaries( true );
	// start plannning
	planner.navigator_control.x = x;
	planner.navigator_control.y = y;
	planner.navigator_control.psi = psi;
	planner.navigator_control.mode = UC_NAVI_PARKING;

	next_replan = Timestamp::getNow() + GETBACKONTRACK_REPLAN_INTERVAL;

}

StGetBackOnTrackAStar::~StGetBackOnTrackAStar()
{
}

sc::result StGetBackOnTrackAStar::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	//	return transit<StDrive>();
	//	return transit<StPause>();

//	ChsmPlanner& planner = context<ChsmPlanner>();
//	Topology* topology = planner.topology;
	navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

	// Transition: Drive (because back on track)
//	if( navigator_feedback.drive_state == UC_NAVI_DRIVE_STOP_DEST ||
//			( topology->ego_vehicle.distToMatchedEdge() < 0.4 && topology->ego_vehicle.angleToMatchedEdge() < M_PI_4 * 0.6 ))
	if( navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST )
		return transit<StDrive>();

	// copy curvepoints if data available
	if( navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED ) {
		// memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints));  // TODO: substitute CurvePoints
	}

//	if (isExpired(next_replan)) {
//		planner.addMessage("check for replan");
//
//		// Auf Missionsende prüfen
//		if (!topology->route_is_finished() ) {
//			topology->getBestReplanningEdge();
//			RndfEdge* best_edge = topology->best_alternative_edge;
//			if (best_edge && best_edge != (*topology->current_edge_it)->getEdge()) {
//			  planner.addMessage("found better edge -> replan");
//			  return transit<StReplanStop>();
//			}
//		}
//		next_replan = Timestamp::getNow() + GETBACKONTRACK_REPLAN_INTERVAL;
//	}

	return forward_event();
}

} // namespace vlr
