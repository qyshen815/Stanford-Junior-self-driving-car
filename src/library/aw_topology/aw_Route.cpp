/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <iostream>
#include <sstream>
#include <cmath>

#include <aw_lane.h>
#include <vlrException.h>

#include "aw_Route.h"

using namespace std;

namespace vlr {

#define DEG2RAD_FACTOR (M_PI/180)

#define TURNOFF_ANGLE_THRESHOLD1 (10 * DEG2RAD_FACTOR)
#define TURNOFF_ANGLE_THRESHOLD2 (30 * DEG2RAD_FACTOR)
#define TURNOFF_ANGLE_THRESHOLD3 (10 * DEG2RAD_FACTOR)
#define U_TURN_ANGLE_THRESHOLD (150 * DEG2RAD_FACTOR)

#define LOCALIZATION_DISTANCE_THRESHOLD 50
#define POSITIVE_LON_DIST_FACTOR 10
#define NEGATIVE_LON_DIST_FACTOR 10

#define INTERSECTION_SAFETY_ZONE_LENGTH 30

#define BACKWARD_VIEW_AREA -50
#define FORWARD_VIEW_AREA 200


#undef TRACE
#define TRACE(str)
//#define TRACE(str) std::cout << "[Route] "<< str << std::endl


namespace RoutePlanner {

RouteAnnotation::RouteAnnotation(const std::string & /*objname*/, double abs_pos, double x, double y, double min_speed,
        double max_speed) :
             maneuver_(UC_MANEUVER_START_MISSION), area_type_(UC_TRAVEL_AREA),
             start_distance_(INFINITY), start_x_(x), start_y_(y),
             length_(0),
             min_speed_(min_speed), max_speed_(max_speed), absolute_start_position_(abs_pos) {
}

RouteAnnotation::RouteAnnotation(const std::string & /*objname*/, double abs_pos, double x, double y, double min_speed,
        double max_speed, maneuver_t m, area_type_t at, const std::string& way_point_name) :
             maneuver_(m), area_type_(at),
             start_distance_(INFINITY), start_x_(x), start_y_(y),
             way_point_name_(way_point_name), length_(0),
             min_speed_(min_speed), max_speed_(max_speed), absolute_start_position_(abs_pos) {
}

RouteAnnotation::~RouteAnnotation() {
}


void RouteAnnotation::dump() {
	cout << "RouteAnnotation("<< absolute_start_position_ << ", x=" << start_x_ << ", y="<< start_y_ << ", min="
	<< min_speed_ << ", max=" << max_speed_ << ", area=" << areaTypeToString(area_type_) << ", "
	<< maneuverToString(maneuver_) << ")"<< endl;
	std::cout << absolute_start_position_ << "\n";
}

string RouteAnnotation::maneuverToString(maneuver_t maneuver) {
    switch (maneuver) {
    case UC_MANEUVER_TRAVEL:
        return "travel";
    case UC_MANEUVER_STOP_SIGN:
        return "stop sign";
    case UC_MANEUVER_CROSSWALK:
        return "crosswalk";
    case UC_MANEUVER_TRAFFIC_LIGHT:
        return "traffic light";
    case UC_MANEUVER_INT_TURN_RIGHT:
        return "turn right at intersection";
    case UC_MANEUVER_INT_TURN_LEFT:
        return "turn left at intersection";
    case UC_MANEUVER_INT_STRAIGHT:
        return "straight across intersection";
    case UC_MANEUVER_CURVE_RIGHT:
        return "right curve";
    case UC_MANEUVER_CURVE_LEFT:
        return "left curve";
    case UC_MANEUVER_U_TURN:
        return "U turn";
    case UC_MANEUVER_NAVIGATE:
        return "navigate zone";
    case UC_MANEUVER_PARKING:
        return "park";
    case UC_MANEUVER_CHECKPOINT:
        return "checkpoint";
    case UC_MANEUVER_GOAL_REACHED:
        return "stop at goal";
    case UC_MANEUVER_START_MISSION:
        return "START MISSION";
    default:
        return "INVALID MANEUVER";
    }
}

string RouteAnnotation::areaTypeToString(area_type_t at) {
	switch (at) {
		case UC_TRAVEL_AREA:
			return "travel";
		case UC_PARKING_ZONE:
			return "parking zone";
		case UC_ZONE:
			return "zone";
		default:
			return "INVALID AREA TYPE";
	}
}

AnnotatedRouteEdge::AnnotatedRouteEdge(RndfEdge * edge, double startPosition) :
   was_reached( false ), edge(edge), startPosition(startPosition), fromCheckpointEdge(false), toCheckpointEdge(false) {
	//edge->setAnnotatedEdge(this);
}

AnnotatedRouteEdge::AnnotatedRouteEdge(const AnnotatedRouteEdge& orig) :
   was_reached( false ), edge(orig.edge), startPosition(orig.startPosition), fromCheckpointEdge(orig.fromCheckpointEdge),
	toCheckpointEdge(orig.toCheckpointEdge) {
	for (std::list< RouteAnnotation* >::const_iterator it = orig.annotations.begin(); it != orig.annotations.end(); it++) {
		annotations.push_back(new RouteAnnotation( *(*it) ));
	}
	//edge->setAnnotatedEdge(this);
}

AnnotatedRouteEdge::~AnnotatedRouteEdge() {
	for (AnnotationList::iterator it = annotations.begin(); it != annotations.end(); ++it) {
		delete *it;
	}
}

string AnnotatedRouteEdge::getNextObjectName() {
std::stringstream s;
s << "A_" << edge->name() << "_" << (long unsigned int)(annotations.size()+1);
//char name[KOGMO_RTDB_OBJMETA_NAME_MAXLEN];
//	snprintf(name, KOGMO_RTDB_OBJMETA_NAME_MAXLEN, "A_%s_%lu", edge->name().c_str(), (long unsigned int)(annotations.size()+1));
	return s.str(); //string(name);
}

double AnnotatedRouteEdge::annotateLength(double prev_start_pos) {
	for (AnnotationList::reverse_iterator rit = annotations.rbegin(); rit != annotations.rend(); ++rit) {
		RouteAnnotation * annotation = *rit;
		if (annotation->maneuver() != UC_MANEUVER_CHECKPOINT && annotation->maneuver() != UC_MANEUVER_START_MISSION) {
			annotation->length(max(0., prev_start_pos - annotation->absoluteStartPosition()));
			prev_start_pos = annotation->absoluteStartPosition();
		}
	}
	return prev_start_pos;
}

void AnnotatedRouteEdge::dump() {
	cout << "RouteEdge("<< edge->name() << ", length="<< edge->getLength() << ", travelTime="<< edge->getTravelTime()
	<< (edge->isLaneEdge() ? ", lane" : "")<< (edge->isZoneEdge() ? ", zone" : "")
	<< (edge->isCircleEdge() ? ", circle" : "");
	if (edge->hasLaneWidth())
		cout << ", width="<< (edge->getLaneWidth());
	cout << ", left="<< rndf::Lane::boundaryToRndfString(edge->getLeftBoundary()) << ", right="
	<< rndf::Lane::boundaryToRndfString(edge->getRightBoundary()) << ", startpos="<< startPosition << ")"<< endl;
	for (AnnotationList::iterator it = annotations.begin(); it != annotations.end(); ++it) {
		(*it)->dump();
	}
}

bool AnnotatedRouteEdge::hasAnnotation(maneuver_t anno) {
	for (AnnotationList::iterator it = annotations.begin(); it != annotations.end(); ++it) {
		if ((*it)->maneuver() == anno) {
			return true;
		}
	}
	return false;
}

void AnnotatedRouteEdge::deleteAnnotation(maneuver_t anno) {
	for (AnnotationList::iterator it = annotations.begin(); it != annotations.end(); ++it) {
		if ((*it)->maneuver() == anno) {
			annotations.erase(it);
			break;
		}
	}
}

Route::Route() :
	routeLength(0), minSpeed(-1), maxSpeed(-1), currentManeuver(UC_MANEUVER_START_MISSION), currentArea(UC_TRAVEL_AREA) {
}

Route::~Route() {
// do NOT delete the annotedrouteedges here! that will break code in StReplan.cpp
}

void Route::addEdges(EdgeList * edgeList)
{
	int last = edgeList->size() - 1;
	int n = 0;
	TRACE("Adding Edges to Route: ");
	for (EdgeList::iterator eit = edgeList->begin(); eit != edgeList->end(); ++eit, ++n) {
		RndfEdge * edge = *eit;
		TRACE(" - "<< edge->name());

		route.push_back(new AnnotatedRouteEdge(edge, routeLength));

		//printf( "anno_edge@%x\n", &(route.back()) );
		if (n == 0) {
			route.back()->setFromCheckpointEdge();
		}
		if (n == last) {
			route.back()->setToCheckpointEdge();
		}
		routeLength += edge->getLength();
	}
}

void Route::addEdge(RndfEdge* edge) {
	route.push_back(new AnnotatedRouteEdge(edge, routeLength));
	route.back()->setFromCheckpointEdge();
	route.back()->setToCheckpointEdge();
	routeLength += edge->getLength();
}

double* Route::computeIntersectionAngles(RndfEdge* edge) {
	RndfVertex * vertex1 = edge->fromVertex();
	RndfVertex * vertex2 = edge->toVertex();
	double dx1 = vertex2->x() - vertex1->x();
	double dy1 = vertex2->y() - vertex1->y();
	double angle1 = atan2(dy1, dx1);
	assert(vertex2->getNumberOfEdges()!=0);
	double * angle = new double[vertex2->getNumberOfEdges()];
	int n = 0;
	for (RndfVertex::EdgeIterator eit = vertex2->beginEdges(); eit != vertex2->endEdges(); ++eit, ++n) {
		RndfEdge * edge3 = static_cast<RndfEdge *>(*eit);
		RndfVertex * vertex3 = edge3->toVertex();
		RndfEdge * nextEdge;
		if (vertex3->getNumberOfEdges() > 0) {
			nextEdge = static_cast<RndfEdge *>(*vertex3->beginEdges());
		} else {
			nextEdge = edge3;
		}
		while (vertex3->getNumberOfEdges() == 1 && nextEdge != NULL && (!nextEdge->isLaneEdge() || nextEdge->isVirtualEdge())) {
			vertex3 = nextEdge->toVertex();
			nextEdge = static_cast<RndfEdge *>(*vertex3->beginEdges());
		}
		RndfVertex* vertex4 = nextEdge->toVertex();
		double dx4 = vertex4->x() - vertex3->x();
		double dy4 = vertex4->y() - vertex3->y();
		double angle4 = atan2(dy4, dx4);
		angle[n] = angle4 - angle1;
		//angle[n] = acos((dx1 * dx4 + dy1 * dy4) / sqrt((sqr(dx1)+sqr(dy1)) * (sqr(dx4)+sqr(dy4))));
		if (angle[n] > M_PI) {angle[n] = angle[n] - 2 * M_PI;}
		if (angle[n] < - M_PI) {angle[n] += 2 * M_PI;}
		//cout << edge->name() << " -> " << nextEdge->name() << " " << angle1 << " " << angle4 << " " << angle[n] << endl;
		//cout << "  " << dx1 << ":" << dy1 << "  " << dx4 << ":" << dy4 << endl;
	}
	return angle;
}

maneuver_t Route::addVertexAnnotations(AnnotatedRouteEdge* routeEdge, RndfVertex* toVertex,
		RndfEdge * nextEdge, bool isCheckpoint, double currentPosition, area_type_t areaType,
		double minSpeed, double maxSpeed, bool firstVertex) {
  if(!routeEdge || !toVertex) {
    throw Exception("addVertexAnnotations: Zero pointer as input argument.");
  }
  if(!routeEdge->getEdge()) {
    throw Exception("addVertexAnnotations: Invalid edge.");
  }

	string name = toVertex->isVirtualVertex() ? "virtual_vertex" : toVertex->name();
	double x = toVertex->x();
	double y = toVertex->y();

  // add crosswalk annotation
  if (toVertex->isCrosswalkVertex()) {
    routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_CROSSWALK, areaType, name));
  }

  // add traffic light annotation
  if (toVertex->isTrafficLightVertex()) {
    routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_TRAFFIC_LIGHT, areaType, name));
  }

  // add Stop Annotation
  if (toVertex->isStopVertex()) {
    routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_STOP_SIGN, areaType, name));
  }

	// add Checkpoint Annotation
	if (isCheckpoint) {
		if (toVertex->isParkingSpotVertex()) {
			routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_PARKING, areaType, name));
		}
		routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_CHECKPOINT, areaType, name));
	}

	// Add Goal Annotation
	if (nextEdge == NULL && !firstVertex) {
		routeEdge->addAnnotation(new RouteAnnotation(routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, UC_MANEUVER_GOAL_REACHED, areaType, name));
	}

	// add intersection and curve annotation
	if (nextEdge) {
		// Über alle Folgelanes iterieren und Krümmungen ermitteln
		// angel[which] gibt anschließen die Krümmung der Folgelane an
		int followingLanes = toVertex->getNumberOfEdges();
		assert(followingLanes >= toVertex->exitCount());
		double * angle = computeIntersectionAngles(routeEdge->getEdge());
		maneuver_t turn = UC_MANEUVER_START_MISSION;
		int which = -1;
		int n = 0;
		int leftmost = 0;
		int rightmost = 0;
		//
		for (RndfVertex::EdgeIterator eit = toVertex->beginEdges(); eit != toVertex->endEdges(); ++eit, ++n) {
			if (*eit == nextEdge) {
				which = n;
			}
			if (angle[n] > angle[leftmost]) {
				leftmost = n;
			}
			if (angle[n] < angle[rightmost]) {
				rightmost = n;
			}
		}
		//cout << "leftmost=" << angle[leftmost] << " rightmost=" << angle[rightmost] << " current=" << angle[which] << endl;

		// Mission Start abfangen
		//		if (turn != UC_MANEUVER_START_MISSION) {
		//			routeEdge->addAnnotation(new RouteAnnotation( routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, turn, areaType, name));
		//			delete[] angle;
		//			return turn;
		//		}

		//cout << "  Testing if Edge is KTurn Edge:" << endl;

		// KTurn setzen
		if (routeEdge->getEdge()->isUTurnEdge()) {
			assert(areaType == UC_TRAVEL_AREA); // U Turns only possible in travel areas
			turn = UC_MANEUVER_U_TURN;
			//cout << "  -> True" << endl;
		} else {
			//cout << "  Testing if Edge is Intersection Edge:" << endl;

			// Intersection und Kurven Annotations setzen
			if (routeEdge->getEdge()->getIntersection() != NULL) {
				//cout << "  -> True" << endl;
				turn = UC_MANEUVER_INT_STRAIGHT;
				if (which == rightmost && routeEdge->getEdge()->isVirtualEdge()) { // non-virtual edges are always straight
					if ((followingLanes <= 2&& angle[which] < - TURNOFF_ANGLE_THRESHOLD2) ||
							(followingLanes >  2&& angle[which] < - TURNOFF_ANGLE_THRESHOLD3))
					{
						turn = UC_MANEUVER_INT_TURN_RIGHT;
					}
				}
				if (which == leftmost && routeEdge->getEdge()->isVirtualEdge()) { // non-virtual edges are always straight
					if ((followingLanes <= 2&& angle[which] > TURNOFF_ANGLE_THRESHOLD2) ||
							(followingLanes >  2&& angle[which] > TURNOFF_ANGLE_THRESHOLD3))
					{
						turn = UC_MANEUVER_INT_TURN_LEFT;
					}
				}
				//			cout << "Turn set to "<< turn << endl;
			}
		}
		//else
		//			cout << "  -> False" << endl;

		/*
		if (followingLanes == 1&& !toVertex->isStopVertex()) {
			if (angle[0] < - TURNOFF_ANGLE_THRESHOLD1) {
				turn = UC_MANEUVER_CURVE_RIGHT;
			}
			if (angle[0] > TURNOFF_ANGLE_THRESHOLD1) {
				turn = UC_MANEUVER_CURVE_LEFT;
			}
		} else {
			turn = UC_MANEUVER_INT_STRAIGHT;
			if (which == rightmost) {
				if ((followingLanes <= 2&& angle[which] < - TURNOFF_ANGLE_THRESHOLD2) ||
					(followingLanes >  2&& angle[which] < - TURNOFF_ANGLE_THRESHOLD3)) {
					turn = UC_MANEUVER_INT_TURN_RIGHT;
				}
			}
			if (which == leftmost) {
				if ((followingLanes <= 2&& angle[which] > TURNOFF_ANGLE_THRESHOLD2) ||
					(followingLanes >  2&& angle[which] > TURNOFF_ANGLE_THRESHOLD3)) {
					turn = UC_MANEUVER_INT_TURN_LEFT;
				}
			}
		}
		 */

		if (turn != UC_MANEUVER_START_MISSION) {
			routeEdge->addAnnotation(new RouteAnnotation( routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, turn, areaType, name));
		}
		delete[] angle;
		return turn;
	}
	return UC_MANEUVER_START_MISSION;
}


void Route::annotateRoute()
{
	double currentPosition = 0;
	std::list<double> safetyPositions;
	area_type_t areaType = UC_TRAVEL_AREA;
	maneuver_t drivingManeuver = UC_MANEUVER_START_MISSION;

	//	cout << "vertex annotations"<< endl;
	currentPosition = 0;
	bool turnManeuver = false;
	RouteEdgeList::iterator firstRouteEdgeIt = route.begin();
	std::list<double>::iterator safetyAreaIt = safetyPositions.begin();
	double minSpeed = (*firstRouteEdgeIt)->getEdge()->getMinSpeed();
	double maxSpeed = (*firstRouteEdgeIt)->getEdge()->getMaxSpeed();
	addVertexAnnotations(*firstRouteEdgeIt, (*firstRouteEdgeIt)->getEdge()->fromVertex(), NULL, route.front()->isFromCheckpointEdge(), currentPosition, areaType, minSpeed, maxSpeed, true);
	maxSpeed = -1;
	minSpeed = -1;
	RndfEdge * edge = NULL;
	RndfEdge * next_edge = NULL;
	RndfEdge * prev_edge = NULL;
	AnnotatedRouteEdge* routeEdge = NULL;
	AnnotatedRouteEdge* next_routeEdge = NULL;
	AnnotatedRouteEdge* prev_routeEdge = NULL;

	for (RouteEdgeList::iterator it = route.begin(); it != route.end(); ++it) {
		routeEdge = *it;
		edge = routeEdge->getEdge();

		// get next route edge already
		RouteEdgeList::iterator nextit = it;
		++nextit;
		if(nextit!=route.end()) {
			next_routeEdge = *nextit;
			next_edge = next_routeEdge->getEdge();
		}
		else {
			next_routeEdge = NULL;
			next_edge = NULL;
		}

		maneuver_t nextDrivingManeuver = UC_MANEUVER_TRAVEL;
		area_type_t nextAreaType = UC_TRAVEL_AREA;
//		cout << "[Route] Annotating  " << edge->name() << " ("<< (edge->isLaneEdge() ? " lane " : "")<< (edge->isLaneChangeEdge() ? " lane_change " : "")<< (edge->isLCEdge() ? " fucking_lane_change " : "")<< (edge->isLaneChange ? " m_lane_change " : "")<< (edge->isVirtualEdge() ? " virtual " : "")<< (edge->isZoneEdge() ? " zone " : "") <<")" << endl;
		if (edge->isLaneEdge()) {
			nextDrivingManeuver = UC_MANEUVER_TRAVEL;
			nextAreaType = UC_TRAVEL_AREA;
		}
		else if (edge->isLaneChangeEdge()) {
			if (edge->isLeftLaneChangeEdge()) {
				nextDrivingManeuver = UC_MANEUVER_LANECHANGE_LEFT;
			} else if (edge->isRightLaneChangeEdge()) {
				nextDrivingManeuver = UC_MANEUVER_LANECHANGE_RIGHT;
			} else {
				assert(false);
				nextDrivingManeuver = UC_MANEUVER_TRAVEL; // backup
			}
			nextAreaType = UC_TRAVEL_AREA;
		}
		else if(edge->isZoneEdge()) {
			// the first edge is annotated as entry
			if(edge->isVirtualEdge()) {
				if(prev_edge && prev_edge->isLaneEdge())
					nextDrivingManeuver = UC_MANEUVER_ZONE_ENTRY;
				else if(next_edge && next_edge->isLaneEdge())
					nextDrivingManeuver = UC_MANEUVER_ZONE_EXIT;
				else {
					cout << edge->name() << endl;
					if (prev_edge) cout << "prev" << prev_edge->name() << endl;
					if (next_edge) cout << "next" << next_edge->name() << endl;
					assert(false);
				}
				nextAreaType = UC_TRAVEL_AREA;
			}
			else {
				nextDrivingManeuver = UC_MANEUVER_NAVIGATE;
				nextAreaType = UC_ZONE;
			}
		}
		else if (edge->isVirtualEdge() ||
				edge->fromVertex()->isStopVertex() ||
				turnManeuver) {
			if(it==route.begin())
				nextDrivingManeuver = UC_MANEUVER_TRAVEL;
			else
				nextDrivingManeuver = drivingManeuver;
			nextAreaType = areaType;
		} else {
			assert(false);
		}
		turnManeuver = false;

		//		if (nextDrivingManeuver != drivingManeuver ||
		//		    nextAreaType != areaType ||
		//		    edge->getMinSpeed() != minSpeed ||
		//		    edge->getMaxSpeed() != maxSpeed) {
		minSpeed = edge->getMinSpeed();
		maxSpeed = edge->getMaxSpeed();
		areaType = nextAreaType;
		drivingManeuver = nextDrivingManeuver;
		double x = edge->fromVertex()->x();
		double y = edge->fromVertex()->y();
		string name = edge->fromVertex()->isVirtualVertex() ? "VIRTUAL_VERTEX" : edge->fromVertex()->name();
		routeEdge->addAnnotation(new RouteAnnotation( routeEdge->getNextObjectName(), currentPosition, x, y, minSpeed, maxSpeed, nextDrivingManeuver, nextAreaType, name));
		//		}
		currentPosition += edge->getLength();

		// Edge Annotation ermitteln und Edge annotieren
		RndfVertex * toVertex = edge->toVertex();
		maneuver_t vertexManeuver = addVertexAnnotations(routeEdge, toVertex, next_edge,
				routeEdge->isToCheckpointEdge(), currentPosition, areaType, minSpeed, maxSpeed, false);
		if (vertexManeuver != UC_MANEUVER_START_MISSION) {
			drivingManeuver = vertexManeuver;
			turnManeuver = true;
		}

		// remember previous edges
		prev_edge = edge ;
		prev_routeEdge = routeEdge;
	}

	double prevStartPos = routeLength;
	for (RouteEdgeList::reverse_iterator rit = route.rbegin(); rit != route.rend(); ++rit) {
		AnnotatedRouteEdge* routeEdge = *rit;
		prevStartPos = routeEdge->annotateLength(prevStartPos);
	}
}

void Route::dump() {
	for (RouteEdgeList::iterator it = route.begin(); it != route.end(); ++it) {
		(*it)->dump();
	}
}

void Route::init() {
  AnnotatedRouteEdge* anedge = *(route.begin());
	veh_x = anedge->getEdge()->fromVertex()->x();
	veh_y = anedge->getEdge()->toVertex()->y();
	veh_route = 0;
}

} // namespace RoutePlanner

} // namespace vlr
