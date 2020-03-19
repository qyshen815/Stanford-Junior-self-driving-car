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
#include "aw_StLaneChange.hpp"
#include "aw_StPause.hpp"
#include "aw_StStop.hpp"
#include "aw_StReplan.hpp"
#include "aw_StDrive.hpp"
#include "aw_StActive.hpp"


namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[StLaneChange] "<< str << std::endl

/*---------------------------------------------------------------------------
 * StLaneChange
 *---------------------------------------------------------------------------*/
StLaneChange::StLaneChange(my_context ctx) : my_base(ctx), kogmo_base(std::string("StLaneChange")),
change_type(LC_IMPOSSIBLE),
merge_allowed(false), has_to_stop(false),
recover_type(LC_RECOVER_TO_ENDPOINT)
{
	mfc = new MergeFeasabilityCheck(mfcPassObstacle, MergeFeasabilityCheck::Merge);
}


StLaneChange::~StLaneChange()
{
	context<ChsmPlanner>().vehiclecmd.turnsignal = TURN_SIGNAL_NONE;

	delete mfc;
	context<ChsmPlanner>().lane_change_data = NULL;
}

sc::result StLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	//	return forward_event();
	return discard_event();
}

sc::result StLaneChange::react(const EvPause&)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.has_pause_lanechange = true;
	planner.pause_lanechange_change_point = change_point;
	planner.pause_lanechange_merge_point = merge_point;
	planner.pause_lanechange_merge_end_point = merge_end_point;
	planner.pause_lanechange_change_reason = change_reason;
	planner.pause_lanechange_change_type = change_type;
	planner.pause_lanechange_lateral_offset = lateral_offset;               // offset between the two lanes
	planner.pause_lanechange_change_length = change_length;                // length of the change
	planner.pause_lanechange_merge_length = merge_length;                 // length of the merging zone
	planner.pause_lanechange_merge_allowed = merge_allowed;
	planner.pause_lanechange_has_to_stop = has_to_stop;
	planner.pause_lanechange_recover_type = recover_type;

//	planner.pause_lanechange_old_params = old_params; // for tentacles

	planner.pause_lanechange_obstacles_in_merge_zone = obstacles_in_merge_zone;
	planner.pause_lanechange_merging_suckers = merging_suckers;

	return transit<StPause>();
}

void StLaneChange::restorePauseState()
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	if (planner.has_pause_lanechange) {
		change_point = planner.pause_lanechange_change_point ;
		merge_point = planner.pause_lanechange_merge_point;
		merge_end_point = planner.pause_lanechange_merge_end_point;
		change_reason = planner.pause_lanechange_change_reason;
		change_type = planner.pause_lanechange_change_type;
		lateral_offset = planner.pause_lanechange_lateral_offset;               // offset between the two lanes
		change_length = planner.pause_lanechange_change_length;                // length of the change
		merge_length = planner.pause_lanechange_merge_length;                 // length of the merging zone
		merge_allowed = planner.pause_lanechange_merge_allowed;
		has_to_stop = planner.pause_lanechange_has_to_stop;
		recover_type = planner.pause_lanechange_recover_type;

//		old_params = planner.pause_lanechange_old_params;

		obstacles_in_merge_zone = planner.pause_lanechange_obstacles_in_merge_zone;
		merging_suckers = planner.pause_lanechange_merging_suckers;
	}
}

bool StLaneChange::mergePossible(double desired_speed, bool check_only_forward)
{
	if (change_type == LC_IMPOSSIBLE) return false; // we cannot do the impossible (yet ;) )

	if ( ! merge_point.valid ) {
		mfc->setState(MergeFeasabilityCheck::Merge);
		return true;
	}

	Point_2 mp = merge_point.point();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology;

	if (change_type == LC_LEFT_OPPOSITE) {
		mfc->setEgoGeoConstrain(lateral_offset + SAFETY_DISTANCE, merge_length);
		mfc->setOtherGeoConstrain(SAFETY_DISTANCE, merge_length);
	} else {
		mfc->setEgoGeoConstrain(lateral_offset + merge_length, SAFETY_DISTANCE);
		mfc->setOtherGeoConstrain(merge_length, SAFETY_DISTANCE );
	}

	size_t merge_allowed = 0;
	//	MergeFeasabilityCheck::Entity ego(hypot(mp.x() - planner.robot_pose.x, mp.y() - planner.robot_pose.y), planner.currentPose().v() ); // TODO: use better distance
	MergeFeasabilityCheck::Entity ego = MergeFeasabilityCheck::getEntity(merge_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());
	if (change_type == LC_LEFT_OPPOSITE)
		ego = MergeFeasabilityCheck::getEntity(merge_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());
	else
		ego = MergeFeasabilityCheck::getEntity(merge_end_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());

	double forward_scan_dist = TRIGGER_DIST_MAX_LOOKAHEAD;
	double backward_scan_dist = TRIGGER_DIST_MAX_LOOKAHEAD;
	if (check_only_forward) backward_scan_dist = 0.;
	MergeFeasabilityCheck::Entities others = MergeFeasabilityCheck::getEntities(merge_point.edge, merge_point.offset, ( change_type == LC_LEFT_OPPOSITE ? forward_scan_dist : backward_scan_dist ), ( change_type == LC_LEFT_OPPOSITE ? backward_scan_dist : forward_scan_dist ));

	// Geschwindigkeiten nahe Null auf Null setzen um rauschen zu unterdrücken
	for (MergeFeasabilityCheck::Entities::iterator iter = others.begin(); iter != others.end(); ++iter) {
		TRACE("other distance = "<< iter->distance << " speed = " << iter->speed);
		if ( fabs( iter->speed ) < 0.5 )
			iter->speed = 0.;
	}

	// Geschwindikeiten und Distanzen für Opposide Lane anpassen
	if (change_type == LC_LEFT_OPPOSITE) {
		for (MergeFeasabilityCheck::Entities::iterator iter = others.begin(); iter != others.end(); ++iter) {
			iter->distance *= -1.;
			iter->speed *= -1.;
		}
	}

	// Löschlisten initalisieren
	set< Vehicle* > non_blocking_vehicles;
	for (std::map<Vehicle*, Timestamp>::iterator it = obstacles_in_merge_zone.begin(); it != obstacles_in_merge_zone.end(); ++it)
		non_blocking_vehicles.insert( it->first );
	set< Vehicle* > sucks_no_more;
	for (std::map<Vehicle*, Timestamp>::iterator it = merging_suckers.begin(); it != merging_suckers.end(); ++it)
		sucks_no_more.insert( it->first );

	// Merging Checks
	for (MergeFeasabilityCheck::Entities::const_iterator iter = others.begin(); iter != others.end(); ++iter)
	{
		if ( merging_suckers.find( iter->veh ) != merging_suckers.end() && merging_suckers[ iter->veh ] + VEHICLE_STAND_TILL_IGNORED < Timestamp::getNow() ) {
			topology->debug_distances.push_back( Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 1.0, 0.9, 0.0 ) );
			++merge_allowed;
			sucks_no_more.erase( iter->veh );
			continue;
		}

		MergeFeasabilityCheck::Result r = mfc->test(ego, *iter, desired_speed);

		if (r == MergeFeasabilityCheck::Merge) {
			TRACE("  -> merge allowed.");
			++merge_allowed;
			topology->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 0.0, 1.0, iter->distance < 0.0 ? 1.0 : 0.0));
		} else {
			TRACE("  -> merge NOT allowed.");
			topology->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 1.0, 0.0, iter->distance < 0.0 ? 1.0 : 0.0));

			// blockierende Fahrzeuge in Mergingzone updaten
			if ( iter->veh->speed() < 0.5) {
				if ( obstacles_in_merge_zone.find( iter->veh ) == obstacles_in_merge_zone.end() )
					obstacles_in_merge_zone.insert( std::make_pair( iter->veh, Timestamp().now() ) );
			} else
				non_blocking_vehicles.erase( iter->veh );

			// behindernde Fahrzeuge updaten
			if ( merging_suckers.find( iter->veh ) == merging_suckers.end() )
				merging_suckers.insert( std::make_pair( iter->veh, Timestamp().now() ) );
			sucks_no_more.erase( iter->veh );
		}
	}

	// Nicht blockierende Fahrzeuge aus den Listen löschen
	for (set< Vehicle* >::iterator it = non_blocking_vehicles.begin(); it != non_blocking_vehicles.end(); ++it)
		obstacles_in_merge_zone.erase( *it );
	// Nicht behindernde Fahrzeuge aus den Listen löschen
	for (set< Vehicle* >::iterator it = sucks_no_more.begin(); it != sucks_no_more.end(); ++it)
		merging_suckers.erase( *it );

	// Result zurückliefern
	if ( merge_allowed == others.size() ) {
		mfc->setState(MergeFeasabilityCheck::Merge);
		return true;
	} else {
		mfc->setState(MergeFeasabilityCheck::Stop);
		return false;
	}
}

bool StLaneChange::snailThroughPossible( const Vehicle& base_vehicle, StLaneChangeLaneChangeType type ) const
{
	Topology* topology = context<ChsmPlanner>().topology;
	VehicleManager* vman = topology->vehicle_manager;
	assert(vman);

	if ( type == LC_IMPOSSIBLE || type == LC_NO_CHANGE )
		return false;

	double angle = base_vehicle.edge()->getAngle();
	Vector_2 yaw_v( cos( angle ), sin( angle ) );
	Vector_2 forw_v = yaw_v * (EGO_VEHICLE_LENGTH + 1.0);
	Vector_2 left_v = forw_v.perpendicular( CGAL::LEFT_TURN ) * (EGO_VEHICLE_WIDTH + 2.0);

	// Prüfen ob neben dem Fahrzeug frei ist
	Vehicle veh(base_vehicle);
	veh.move( ( type == LC_RIGHT ? - left_v : left_v ) );
	if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

	// Prüfen ob vor dem Fahrzeug frei ist
	veh.move( ( type == LC_LEFT_OPPOSITE ? forw_v : -forw_v ) );
	if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

	// Prüfen ob hinter dem Fahreug frei ist
	if ( type == LC_LEFT_OPPOSITE ) {
		veh = base_vehicle;
		veh.move( - forw_v );
		if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

		veh.move( - forw_v );
		if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;
	}

	return true;
}

bool StLaneChange::placeIsValid(const GraphPlace& place) const
{
	Topology* topology = context<ChsmPlanner>().topology;

	if ( ! place.valid ) {
		TRACE("  -> GraphPlace not valid");
		return false;
	}

	Point_2 p = place.point();
	double yaw = place.edge->getAngle();
	Vehicle veh_at_end_point( p.x() + cos(yaw)*CENTER_DELTA, p.y() + sin(yaw)*CENTER_DELTA, yaw, EGO_VEHICLE_WIDTH, EGO_VEHICLE_LENGTH );
	if ( topology->vehicle_manager->intersectsWithAnotherVehicle( veh_at_end_point ) ) {
		TRACE("  -> GraphPlace intersects with other vehicle");
		return false;
	}

	return true;
}


/*---------------------------------------------------------------------------
 * StPrepareLaneChange
 *---------------------------------------------------------------------------*/
StPrepareLaneChange::StPrepareLaneChange(my_context ctx) : my_base(ctx), kogmo_base(std::string("StPrepareLaneChange")), prep_start(0)
{
	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	//	RndfEdge* ego_edge = ego_veh.edge;

	// reset pause state
	planner.has_pause_lanechange = false;

	// changePoint berechnen und change_reason setzen
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	mv_veh_dist -= LANE_CHANGE_LENGTH;
	//	if (mv_veh_dist < 0.) mv_veh_dist = 0.;
	sv_veh_dist -= LANE_CHANGE_LENGTH;
	//	if (sv_veh_dist < 0.) sv_veh_dist = 0.;
	double route_lc_dist  = topology->dist_to_next_lanechange() + FRONT_BUMPER_DELTA;
	//	if (route_lc_dist < 0.) route_lc_dist = 0.;
	std::map< double, StLaneChangeLaneChangeReason > dists;
	dists.insert(std::make_pair( mv_veh_dist, LC_CAUSE_OF_MOVING_VEHICLE ) );
	dists.insert(std::make_pair( sv_veh_dist, LC_CAUSE_OF_OBSTACLE ) );
	dists.insert(std::make_pair( route_lc_dist, LC_CAUSE_OF_ROUTE ) );
	lc.change_reason = dists.begin()->second;
	lc.change_point = PlaceOnGraph( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );
	lc.change_point += dists.begin()->first;
	assert( lc.change_point.isValid() );
	RndfEdge* edge = (*lc.change_point.edge)->getEdge();
	assert(edge);
	TRACE("Changing lane from "<< edge->name());

	// Stoppen aktivieren bei stehendem Fahrtzeug voraus
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE)
		lc.has_to_stop = true;

	// changeType setzen
	if (lc.change_reason == LC_CAUSE_OF_ROUTE) {
		double route_lc_left_dist  = topology->dist_to_next_maneuver_start(UC_MANEUVER_LANECHANGE_LEFT);
		double route_lc_right_dist = topology->dist_to_next_maneuver_start(UC_MANEUVER_LANECHANGE_RIGHT);
		if ( route_lc_left_dist < route_lc_right_dist ) {
			TRACE("  -> Wechsel nach links wegen Route");
			lc.change_type = LC_LEFT;
		} else {
			TRACE("  -> Wechsel nach rechts wegen Route");
			lc.change_type = LC_RIGHT;
		}
	} else {
		// Annahme: max zwei Spuren in eine Richtung
		lc.change_type = LC_IMPOSSIBLE;
		if      ( ! edge->getLeftEdges().empty() ) {
			TRACE("  Aktuelle Spur hat eine linke Nachbarspur");
			if ( leftLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_STD_MERGING_LENGTH) ) {
				TRACE("  -> linke Spur eignet sich zum Wechseln");
				lc.change_type = LC_LEFT;
			} else { TRACE("  -> linke Spur eigent sich NICHT zum Wechseln"); }
		}
		if ( ! edge->getRightEdges().empty() && lc.change_type == LC_IMPOSSIBLE ) {
			TRACE("  Aktuelle Spur hat eine rechte Nachbarspur");
			if ( rightLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_STD_MERGING_LENGTH) ) {
				TRACE("  -> rechte Spur eignet sich zum Wechseln");
				lc.change_type = LC_RIGHT;
			} else { TRACE("  -> rechte Spur eigent sich NICHT zum Wechseln"); }
		}
		if ( ! edge->getLeftOppositeEdges().empty() && lc.change_type == LC_IMPOSSIBLE ) {
			TRACE("  Aktuelle Spur hat eine linke Gegenfahrbahn");
			if ( leftOppositeLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_OPPOSITE_MERGING_LENGTH) ) {
				TRACE("  -> linke Gegenfahrbahn eignet sich zum Wechseln");
				lc.change_type = LC_LEFT_OPPOSITE;
			} else { TRACE("  -> linke Gegenfahrbahn eigent sich NICHT zum Wechseln"); }
		}
	}

	// Abbrechen falls Spurwechsel unmöglich ist
	if (lc.change_type == LC_IMPOSSIBLE) {
		TRACE("=> Keine geeignete Spur zum Wechseln gefunden");
		if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
			TRACE("Spurwechsel war wegen stehendem Fahrzeug geplant");
			TRACE("-> Spuren des Fahrzeugs als blockiert markieren");
			Vehicle* veh = topology->get_next_vehicle();
			assert(veh);
			if (planner.params().enable_blockade_detection && veh) veh->blockMatchedEdges();
			TRACE("=> Replanning");
			return;
		} else if ( lc.change_reason == LC_CAUSE_OF_MOVING_VEHICLE ) {
			TRACE("Spurwechsel war wegen fahrendem Fahrzeug geplant -> Drive");
			lc.change_type = LC_NO_CHANGE;
			return;
		} else return;
	}

	// Mergepoint ermitteln
	lc.merge_point = GraphPlace((*lc.change_point.edge)->getEdge(), lc.change_point.offset);
	assert(lc.merge_point.valid);
	lc.merge_length = LANE_CHANGE_STD_MERGING_LENGTH;
	switch (lc.change_type) {
		case LC_LEFT:
			lc.merge_point.goToLeftEdge();
			break;
		case LC_RIGHT:
			lc.merge_point.goToRightEdge();
			break;
		case LC_LEFT_OPPOSITE:
			lc.merge_point.goToLeftOppositeEdge();
			lc.merge_length = LANE_CHANGE_OPPOSITE_MERGING_LENGTH;
			break;
		default: assert(false); break;
	}
//	assert(lc.merge_point.valid);
	if ( ! lc.merge_point.valid )
		lc.change_type = LC_IMPOSSIBLE;


	// MergeEndPoint ermitteln
	if (lc.merge_point.valid) {
		if ( lc.change_type != LC_LEFT_OPPOSITE )
			lc.merge_end_point = searchDistOnLane(lc.merge_point, GraphSearchTraits::FORWARD, lc.merge_length );
		else
			lc.merge_end_point = searchDistOnLane(lc.merge_point, GraphSearchTraits::BACKWARD, lc.merge_length );
	}

	// sampling offset berechnen
	RndfEdge* change_edge = NULL;
	if (lc.change_reason == LC_CAUSE_OF_ROUTE) {
		if (lc.change_type == LC_LEFT)
			change_edge = (topology->next_edge_with_maneuver(UC_MANEUVER_LANECHANGE_LEFT))->getEdge();
		else if (lc.change_type == LC_RIGHT)
			change_edge = (topology->next_edge_with_maneuver(UC_MANEUVER_LANECHANGE_RIGHT))->getEdge();
		assert(change_edge);
		assert(change_edge->isLaneChangeEdge());
	} else {
		TRACE("Looking for change_edge:");
		// TODO ChangeEdge anhand eines Graphplace ermitteln
		switch ( lc.change_type ) {
			case LC_LEFT          : change_edge = *edge->getLeftEdges().begin(); break;
			case LC_RIGHT         : change_edge = *edge->getRightEdges().begin(); break;
			case LC_LEFT_OPPOSITE : change_edge = *edge->getLeftOppositeEdges().begin(); break;
			default: assert(false); break;
		}
		if (change_edge) TRACE("  -> "<< change_edge->name());
	}
	assert(change_edge);

	// change length setzen
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
		lc.change_length = LANE_CHANGE_LENGTH;
	else
		lc.change_length = change_edge->getLength()+1.0;

	TRACE("  change_length: "<< lc.change_length);

	lc.lateral_offset = std::sqrt( squared_distance( lc.change_point.point(), change_edge->getLine() ) );
	if (lc.change_type == LC_RIGHT) lc.lateral_offset *= -1;

	planner.lane_change_data = &lc;
}

StPrepareLaneChange::~StPrepareLaneChange()
{
}

sc::result StPrepareLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Replannig anschmeißen falls hier kein Lanechange möglich ist
	if ( lc.change_type == LC_IMPOSSIBLE ) {
		// Timer starten
		if ( prep_start == Timestamp(0) )
			prep_start = Timestamp::getNow();

		// Prüfen ob die Wartezeit schon um ist
		if ( prep_start + VEHICLE_STAND_TILL_BLOCKADE_TIME < Timestamp::getNow() ) {
			planner.generateCurvePoints(planner.params().max_speed_pass_obstacle);
		} else {
			TRACE("Spurwechsel war wegen stehendem Fahrzeug geplant");
			Vehicle* veh = topology->get_next_vehicle();
			if ( veh == NULL) {
				TRACE("-> Kein Hindernis mehr voraus");
				TRACE("=> Drive");
				return transit<StDrive>();
			} else if ( veh->speed() > 0.5) {
				TRACE("-> Hindernis bewegt sich wieder");
				TRACE("=> Drive");
				return transit<StDrive>();
			} else {
				TRACE("-> Spuren des Fahrzeugs als blockiert markieren");
				TRACE("=> Replanning");
				if (planner.params().enable_blockade_detection && veh) veh->blockMatchedEdges();
				return transit<StReplan>();
			}
		}
	}

	// Wieder in Drive gehen wenn kein Lanechange möglich ist und die Ursache ein
	// fahrendes Fahrzug war
	if ( lc.change_type == LC_NO_CHANGE )
		return transit<StDrive>();

	// Distanz zum Changepoint berechnen
	double dist = difference( lc.change_point, ego_place );
	TRACE("Dist to change_point (P): "<< dist);

	// Prüfen ob sich das Vehicle bewegt hat und das Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		if (veh_dist < 0.) veh_dist = 0.;
		if ( veh_dist > dist + 2.0 )
			return transit<StDrive>();
	}

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked(dist + lc.change_length + 2.0) )
		return transit<StReplan>();

	// Blinker setzen
	if ( dist < TRIGGER_DIST_BLINK )
		planner.vehiclecmd.turnsignal = (lc.change_type == LC_RIGHT ? TURN_SIGNAL_RIGHT : TURN_SIGNAL_LEFT);

	// Timestamp setzen fürs Warten
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE && planner.currentPose().v() < STOP_SPEED_THRESHOLD && prep_start == Timestamp(0)) {
		prep_start.now();
	}

	// Timestamp reseten falls sich das Fahrzeug bewegt
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE && planner.currentPose().v() > STOP_SPEED_THRESHOLD) {
		prep_start = Timestamp(0);
	}

	// Transition: Zurücksetzen falls man schon zu weit vorne ist
	if ( dist < - STD_VEHICLE_LENGTH ) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}

	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_lane_merge_false);
	//lc.merge_allowed = true; // HACK

	if ( ! lc.merge_allowed || (lc.has_to_stop && (prep_start == Timestamp(0) || prep_start + MIN_WAIT_OBSTACLE > Timestamp::getNow()) ) ) {
		// Prüfen ob es möglich wäre sich durchzuschlängeln
		Vehicle* next_veh = topology->get_next_vehicle();
		bool snail_possible = ( next_veh ? lc.snailThroughPossible( *next_veh, lc.change_type ) : false );

//		if (snail_possible && prep_start != Timestamp(0)) {
//			lc.recover_type = LC_RECOVER_TO_ENDPOINT;
//			return transit<StLaneChangeRecover>();
//		}

		// Prüfen ob die Zone schon länger durch ein stehendes Fahrzeug blockiert ist
		// und gegebenenfalls die Lanes blockieren und Replanning durchführen
		bool replan = false;

		for (std::map<Vehicle*, Timestamp>::iterator it = lc.obstacles_in_merge_zone.begin(); it != lc.obstacles_in_merge_zone.end(); ++it) {
			if ( it->second + VEHICLE_STAND_TILL_BLOCKADE_TIME < Timestamp::getNow() && ! snail_possible ) {
				Vehicle* veh = it->first;
				if (planner.params().enable_blockade_detection) veh->blockMatchedEdges();

				// Fahrzeug voraus gegebenenfalls auch als blockierenend markieren
				double mv_veh_dist, sv_veh_dist;
				planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
				double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
				veh_dist -= lc.change_length;
				// if (veh_dist < 0.) veh_dist = 0.;
				if ( veh_dist < dist + 2.0 && next_veh )
					if (planner.params().enable_blockade_detection) next_veh->blockMatchedEdges();

				replan = true;
			}
		}
		if (replan)
			return transit<StReplan>();

		// Generate Curvepoints
		double st_veh_dist, mv_veh_dist;
		planner.getVehicleDistances(st_veh_dist, mv_veh_dist);
		planner.generateCurvePoints(dist, std::min(st_veh_dist, mv_veh_dist), planner.params().max_speed_lane_merge_false);//( stop at change_point );

		return forward_event();
	}

	//
	if ( ! lc.merging_suckers.empty() ) {
//		bool do_recover = true;
//		for (map<Vehicle*, Timestamp>::iterator it = merging_suckers.begin(); it != merging_suckers.end(); ++it) {
//			if ( it->second + VEHICLE_STAND_TILL_IGNORED > Timestamp::getNow() )
//				do_recover = false;
//		}
//		if ( do_recover ) {
			lc.recover_type = LC_RECOVER_TO_ENDPOINT;
			return transit<StLaneChangeRecover>();
//		}
	}

	// Sobald Change Point erreicht wurde Spurwechseln
	if ( dist < 0.2 && dist >= - STD_VEHICLE_LENGTH )
	{
		// Transition: change to left lane
		if (lc.change_type == LC_LEFT)
			return transit<StChangeToLeftLane>();
		// Transition: change to left lane
		if (lc.change_type == LC_RIGHT)
			return transit<StChangeToRightLane>();
		// Transition: change to left lane
		if (lc.change_type == LC_LEFT_OPPOSITE)
			return transit<StChangeToLeftOppositeLane>();
	}

	// Curvepoints erzeugen
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
		std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
//	planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_OBSTACLE );
	else {
		planner.generateCurvePoints(planner.params().max_speed_pass_obstacle);
	}

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToLeftLane
 *---------------------------------------------------------------------------*/
StChangeToLeftLane::StChangeToLeftLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StChangeToLeftLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
//		context<StLaneChange>().storeTentacleParams();
//		context<StLaneChange>().setLaneChangeTentacleParams();
//	}
}

StChangeToLeftLane::~StChangeToLeftLane()
{
//	StLaneChange& lc = context<StLaneChange>();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
//		context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToLeftLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Blinker setzen
	planner.vehiclecmd.turnsignal = TURN_SIGNAL_LEFT;

	// Distanz zum Changepoint berechnen
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (L): "<< dist);

	// TODO Garantie einbauen, das beim Replanning die aktuelle Kante ausgewählt wird

	// Transition: Replan (weil neue Lane erreicht wurde)
	//		if ( fabs(lc.lateral_offset) - ego_veh.distToMatchedEdge() < 0.2  )
	if ( - dist > lc.change_length ) {
		if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
			planner.forced_start_edge = lc.merge_point.edge;
			planner.stop_before_replanning = false;
			return transit<StReplan>();
		} else
			return transit<StDrive>();
	}

	// Prüfen ob sich das Vehicle bewegt hat und das Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}

	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );
	else
		planner.generateCurvePoints(planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToRightLane
 *---------------------------------------------------------------------------*/
StChangeToRightLane::StChangeToRightLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StChangeToRightLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
//		context<StLaneChange>().storeTentacleParams();
//		context<StLaneChange>().setLaneChangeTentacleParams();
//	}
}

StChangeToRightLane::~StChangeToRightLane()
{
//	StLaneChange& lc = context<StLaneChange>();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
//		context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToRightLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Blinker setzen
	planner.vehiclecmd.turnsignal = TURN_SIGNAL_RIGHT;

	// Distanz zum Changepoint berechnen
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (R): "<< dist);

	// Transition: Replan (weil neue Lane erreicht wurde)
	if ( - dist > lc.change_length ) {
		if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
			planner.forced_start_edge = lc.merge_point.edge;
			planner.stop_before_replanning = false;
			return transit<StReplan>();
		} else
			return transit<StDrive>();
	}

	// Prüfen ob sich das Vehicle bewegt hat und das Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}


	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );
	else
		planner.generateCurvePoints(planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToLeftOppositeLane
 *---------------------------------------------------------------------------*/
StChangeToLeftOppositeLane::StChangeToLeftOppositeLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StChangeToLeftOppositeLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
//	context<StLaneChange>().storeTentacleParams();
//	context<StLaneChange>().setLaneChangeTentacleParams();
}

StChangeToLeftOppositeLane::~StChangeToLeftOppositeLane()
{
//	context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToLeftOppositeLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Blinker setzen
	planner.vehiclecmd.turnsignal = TURN_SIGNAL_LEFT;

	// Distanz zum Changepoint berechnen
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (L): "<< dist);

	// Transition: drive on opposite lane (weil neue Lane erreicht wurde)
	if ( - dist > lc.change_length ) {
		TRACE("Andere Spur erreicht");
		planner.addMessage("opposide lane reached");
		return transit<StDriveOnOppositeLane>();
	}

	// Prüfen ob sich das Vehicle bewegt hat und das Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		TRACE("  veh_dist: "<< veh_dist << "  dist: "<< dist);
		if ( veh_dist > dist + 2.0 && ( veh == NULL || veh->speed() > 0.5 ) ) {
			TRACE("Hindernis hat sich weiter bewegt => Drive");
			planner.addMessage("Obstacle moved");
			return transit<StDrive>();
		}
	}


	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
  std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
  //planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDriveOnOppositeLane
 *---------------------------------------------------------------------------*/
StDriveOnOppositeLane::StDriveOnOppositeLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDriveOnOppositeLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
}

StDriveOnOppositeLane::~StDriveOnOppositeLane()
{
}

sc::result StDriveOnOppositeLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Distanz zum Changepoint berechnen
	//	double dist = - difference( ego_place, lc.change_point );
	//	TRACE("Dist to change_point (L): "<< dist);

	// Merging Checks fürs wiedereinfädeln
	// BETTER Richtige Merge Checks
	bool merge_allowed = true;
	//	const map<int, Vehicle>& vehicles = topology->vehicle_manager->vehicle_map;
	//	for (map<int, Vehicle>::const_iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
	double min_dist = LANE_CHANGE_LENGTH + STD_VEHICLE_LENGTH;
	if (topology->dist_to_prev_veh()          < 0.5 ||
			topology->dist_to_next_moving_veh()   < min_dist ||
			topology->dist_to_next_standing_veh() < min_dist )
		merge_allowed = false;

	// Transition: drive back to original lane (weil Fahrzeug überholt wurde)
	if (merge_allowed) {
		lc.change_point = PlaceOnGraph( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );
		return transit<StChangeBackFromOppositeLane>();
	}

	// Merge Check für entgegenkommenden Verkehr
	lc.merge_point = GraphPlace( (*ego_place.edge)->getEdge(), ego_place.offset );
	lc.merge_point.goToLeftOppositeEdge();
	lc.merge_length = LANE_CHANGE_OPPOSITE_MERGING_LENGTH - LANE_CHANGE_LENGTH;
	PlaceOnGraph end_place = ego_place;
	end_place += lc.merge_length;
	lc.merge_end_point = GraphPlace( (*end_place.edge)->getEdge(), end_place.offset );
	lc.merge_end_point.goToLeftOppositeEdge();

	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_obstacle, true);
	//	if ( ! lc.merge_allowed )
	//		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
	if ( lc.merge_allowed )
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( lc.lateral_offset, -20., 1., lc.lateral_offset, MAX_SPEED_PASS_OBSTACLE );
	else
		context<ChsmPlanner>().generateStopTrajectory();

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeBackFromOppositeLane
 *---------------------------------------------------------------------------*/
StChangeBackFromOppositeLane::StChangeBackFromOppositeLane(my_context ctx) : my_base(ctx), kogmo_base(std::string("StChangeBackFromOppositeLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();

//	context<StLaneChange>().storeTentacleParams();
//	context<StLaneChange>().setLaneChangeTentacleParams();

	context<StLaneChange>().merge_length = LANE_CHANGE_LENGTH;
}

StChangeBackFromOppositeLane::~StChangeBackFromOppositeLane()
{
//	context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeBackFromOppositeLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Blinker setzen
	planner.vehiclecmd.turnsignal = TURN_SIGNAL_RIGHT;

	// Distanz zum Changepoint berechnen
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point: "<< dist);

	// Transition: Drive (weil alte Lane wieder erreicht wurde)
	if ( - dist > lc.change_length ) {
		return transit<StDrive>();
	}

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
  std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
	//planner.generateCurvePoints( lc.lateral_offset, dist, lc.change_length, 0., MAX_SPEED_PASS_SWITCH_LANE );

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StAbortLaneChange
 *---------------------------------------------------------------------------*/
StAbortLaneChange::StAbortLaneChange(my_context ctx) : my_base(ctx), kogmo_base(std::string("StAbortLaneChange"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
}

StAbortLaneChange::~StAbortLaneChange()
{
}

sc::result StAbortLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// Blinker setzen
	planner.vehiclecmd.turnsignal = ( lc.change_type == LC_RIGHT ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT );

	// Distanz zum Changepoint berechnen
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point: "<< dist);

	// Prüfen ob sich das Vehicle bewegt hat und ob das erneute Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}

	// Transition:  (weil alte Spur erreicht Lane erreicht wurde oder man nicht mehr weiterfahren kann)
	if (ego_veh.distToMatchedEdge() < 0.5 || ego_veh.speed() < 0.2 ) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}


	// Curvepoints erzeugen
	planner.generateCurvePoints( planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StLaneChangeRecover
 *---------------------------------------------------------------------------*/
StLaneChangeRecover::StLaneChangeRecover(my_context ctx) : my_base(ctx), kogmo_base(std::string("StLaneChangeRecover"))
{
	map_timer.now();
	map_timer += MIN_WAIT_FOR_OBSTACLEMAP;

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
}

StLaneChangeRecover::~StLaneChangeRecover()
{
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StLaneChangeRecover::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return discard_event(); // lanechange state don't get this
}

/*---------------------------------------------------------------------------
 * StLaneChangeRecoverPrepare
 *---------------------------------------------------------------------------*/
StLaneChangeRecoverPrepare::StLaneChangeRecoverPrepare(my_context ctx) : my_base(ctx), kogmo_base(std::string("StLaneChangeRecoverPrepare"))
{
}

StLaneChangeRecoverPrepare::~StLaneChangeRecoverPrepare() {
}

sc::result StLaneChangeRecoverPrepare::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	if (isExpired(context<StLaneChangeRecover>().map_timer)) {
		return transit<StLaneChangeRecoverAStar>();
	} else {
		context<ChsmPlanner>().generateStopTrajectory();
		return forward_event();
	}
}

/*---------------------------------------------------------------------------
 * StLaneChangeRecoverAStar
 *---------------------------------------------------------------------------*/
StLaneChangeRecoverAStar::StLaneChangeRecoverAStar(my_context ctx) : my_base(ctx), kogmo_base(std::string("StLaneChangeRecoverAStar")), failed(false)
{
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();

	GraphPlace end_point;
	if (lc.recover_type == LC_RECOVER_TO_ENDPOINT) {
		// Endpunkt prüfen
		planner.addMessage("Recover to lanechange end point");
		TRACE("Recover to lanechange end point");
		end_point = context<StLaneChange>().merge_end_point;

		if ( end_point.valid && lc.change_type == LC_LEFT_OPPOSITE )
			end_point.goToLeftOppositeEdge();

		// Anfangspunkt prüfen
		if ( ! lc.placeIsValid(end_point) ) {
			planner.addMessage("Recover to lanechange start point");
			end_point = GraphPlace( (*lc.change_point.edge)->getEdge(), lc.change_point.offset );
		}

		if ( ! lc.placeIsValid(end_point) )
			failed = true;
	}
	else if (lc.recover_type == LC_RECOVER_TO_STARTPOINT)
	{
		// Anfangspunkt prüfen
		planner.addMessage("Recover to lanechange start point");
		TRACE("Recover to lanechange end point");
		end_point = GraphPlace( (*lc.change_point.edge)->getEdge(), lc.change_point.offset );

		if ( ! lc.placeIsValid(end_point) ) {
			planner.addMessage("Recover to lanechange end point");
			TRACE("  -> Startpunkt belegt => Probiere Endpunkt");
			context<StLaneChange>().merge_end_point;

			// Endpunkt prüfen
			if ( end_point.valid && lc.change_type == LC_LEFT_OPPOSITE )
				end_point.goToLeftOppositeEdge();
		}
		if ( ! lc.placeIsValid(end_point) ) {
			TRACE("  -> Punkt belegt");
			failed = true;
		}
	} else {
		assert(false);
		failed = true;
	}

	if ( end_point.valid && ! failed ) {
		double x = end_point.point().x();
		double y = end_point.point().y();
		double psi = end_point.edge->getAngle();

		// start plannning
		planner.navigator_control.x = x;
		planner.navigator_control.y = y;
		planner.navigator_control.psi = psi;
		planner.navigator_control.mode = UC_NAVI_PARKING;
	} else {
		planner.addMessage("Recover failed");
		failed = true;
	}


	planner.generateStopTrajectory();
}

StLaneChangeRecoverAStar::~StLaneChangeRecoverAStar() {
	// idle navigator
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.navigator_control.x = planner.navigator_control.y = planner.navigator_control.psi = 0;
	planner.navigator_control.mode = UC_NAVI_IDLE;
}

sc::result StLaneChangeRecoverAStar::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	navigator_feedback_t* navigator_feedback = context<ChsmPlanner>().navigator_feedback_;

	// Transition: Recovery Mode
	if (failed) {
		planner.addMessage("Recover to exit was not successful");
		return transit<StPause>();
	}


	// Transition: if A* is complete, do a replan
	if(navigator_feedback->drive_state == UC_NAVI_DRIVE_STOP_DEST ) {
		return transit<StReplan>();
	}

	// copy curvepoints if data available
	if(navigator_feedback->drive_state != UC_NAVI_DRIVE_UNDEFINED) {
		//memcpy(planner.curvepoints_, &navigator_feedback->curvepoints, sizeof(CurvePoints));  // TODO: substitute CurvePoints
	} else {
		planner.generateStopTrajectory();
	}

	return discard_event(); // intersection state don't get this
}



//=============================================================================
//		Convenience Functions
//=============================================================================

bool leftLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange Startpunkt invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge)->getEdge(), lc_start.offset );
	GraphPlace merge_left( merge_start );

	// Auf die linke Spur wechseln
	merge_left.goToLeftEdge();
	if ( ! merge_left.valid ) {
		TRACE("    Merge Startpunkt invalid");
		return false;
	}

	// Fahrbahnmarkierung überprüfen
	if ( merge_start.edge->getLeftBoundary() == rndf::Lane::DoubleYellow ||
			merge_start.edge->getLeftBoundary() == rndf::Lane::SolidWhite )
	{
		TRACE("    Fahrbahnmarkierung verbietet Lanechange nach links");
		return false;
	}

	// Distanzsuche voraus durchführen
	GraphPlace merge_end = searchDistOnLane(merge_left, GraphSearchTraits::FORWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge Endpunkt invalid");
		return false;
	}

	// Auf die rechte Spur wechseln
	merge_end.goToRightEdge();
	if ( ! merge_end.valid ) {
		TRACE("    LaneChange Endpunkt invalid");
		return false;
	}

	// Von der Startedge aus die Zieledge suchen
	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
		TRACE("    Spuren divergieren. (Kein direkter Weg von "<< merge_start.edge->name() <<" nach "<< merge_end.edge->name() <<" gefunden)");
	}

	//	return result;
	return true;
}

bool rightLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange Startpunkt invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge)->getEdge(), lc_start.offset );
	GraphPlace merge_right( merge_start );

	// Auf die rechte Spur wechseln
	merge_right.goToRightEdge();
	if ( ! merge_right.valid ) {
		TRACE("    Merge Startpunkt invalid");
		return false;
	}

	// Fahrbahnmarkierung überprüfen
	if ( merge_start.edge->getRightBoundary() == rndf::Lane::DoubleYellow ||
			merge_start.edge->getRightBoundary() == rndf::Lane::SolidWhite )
	{
		TRACE("    Fahrbahnmarkierung verbietet Lanechange nach rechts");
		return false;
	}

	// Distanzsuche voraus durchführen
	GraphPlace merge_end = searchDistOnLane(merge_right, GraphSearchTraits::FORWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge Endpunkt invalid");
		return false;
	}

	// Auf die linke Spur wechseln
	merge_end.goToLeftEdge();
	if ( ! merge_end.valid ) {
		TRACE("    LaneChange Endpunkt invalid");
		return false;
	}

	// Von der Startedge aus die Zieledge suchen
	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
		TRACE("    Spuren divergieren. (Kein direkter Weg von "<< merge_start.edge->name() <<" nach "<< merge_end.edge->name() <<" gefunden)");
	}

	//	return result;
	return true;
}

bool leftOppositeLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange Startpunkt invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge)->getEdge(), lc_start.offset );
	GraphPlace merge_left_opp( merge_start );

	// Auf die linke Spur wechseln
	merge_left_opp.goToLeftOppositeEdge();
	if ( ! merge_left_opp.valid ) {
		TRACE("    Merge Startpunkt invalid");
		return false;
	}

	// Fahrbahnmarkierung überprüfen
	if ( merge_start.edge->getLeftBoundary() == rndf::Lane::DoubleYellow ||
			merge_start.edge->getLeftBoundary() == rndf::Lane::SolidWhite )
	{
		TRACE("    Fahrbahnmarkierung verbietet Lanechange nach links auf die Gegenfahrspur");
		return false;
	}

	// Distanzsuche voraus durchführen
	GraphPlace merge_end = searchDistOnLane(merge_left_opp, GraphSearchTraits::BACKWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge Startpunkt invalid");
		return false;
	}

	// Auf die rechte Spur wechseln
	merge_end.goToLeftOppositeEdge();
	if ( ! merge_end.valid ) {
		TRACE("    Merge Startpunkt invalid");
		return false;
	}

	// Von der Startedge aus die Zieledge suchen
	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
		TRACE("    Spuren divergieren. (Kein direkter Weg von "<< merge_start.edge->name() <<" nach "<< merge_end.edge->name() <<" gefunden)");
	}

	return true;
}

} // namespace vlr
