/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_ROUTE_H
#define AW_ROUTE_H

#include <list>
#include <deque>
#include <string>
#include <iostream>

#include <aw_roadNetwork.h>
#include <aw_Maneuver.h>
#include <aw_RndfGraph.h>

namespace vlr {

#define UC_RNDF_LANE_SAMPLING_INTERVAL 1.
#define UC_RNDF_LANE_PREVIEW 200

namespace RoutePlanner {

class RouteAnnotation  {
public:
    RouteAnnotation( const std::string & objname, double abs_pos, double x, double y, double min_speed, double max_speed);
    RouteAnnotation( const std::string & objname, double abs_pos, double x, double y, double min_speed, double max_speed, maneuver_t m, area_type_t at, const std::string& way_point_name);
	virtual ~RouteAnnotation();

	inline double absoluteStartPosition() const { return absolute_start_position_; }

    inline maneuver_t maneuver() const {return maneuver_;}
	inline void maneuver(maneuver_t m) {maneuver_ = m;}

	inline area_type_t areaType() const {return area_type_;}
	inline void areaType(area_type_t at) {area_type_ = at;}

	inline void length(double l) {length_ = l;}

	inline void updatePosition(double vehicle_pos) {
	    start_distance_ = absolute_start_position_ - vehicle_pos;
	}

	void dump();
	static std::string maneuverToString(maneuver_t maneuver);
	static std::string areaTypeToString(area_type_t areaType);

private:
    maneuver_t maneuver_;
    area_type_t area_type_;     //parking, safety or travel (road) area
    double start_distance_;     // distance from now on (time when this description becomes valid);  not the euclidean distance between the car and the starting pos
    double start_x_, start_y_;  // start position of this maneuver (absolute coordinates)
    std::string way_point_name_; // RNDF Waypoint name (eg "1.1.3") TODO: really needed?!?
    double length_;
    double min_speed_, max_speed_;  // speed limits

    double absolute_start_position_;
};

class AnnotatedRouteEdge {
public:
	typedef std::list<RouteAnnotation*> AnnotationList;

	AnnotatedRouteEdge(RndfEdge * edge, double startPosition);
	AnnotatedRouteEdge( const AnnotatedRouteEdge& );
	virtual ~AnnotatedRouteEdge();

	double getStartPosition() const { return startPosition; }
	bool isFromCheckpointEdge() const { return fromCheckpointEdge; }
	bool isToCheckpointEdge() const { return toCheckpointEdge; }

	void addAnnotation(RouteAnnotation * annotation) {
		if( annotation ) annotations.push_back(annotation);
		else printf( "zero annotation occured!\n" );
		//std::cout << "Add Edge Annotation:  "<< edge->name() <<" -> " << annotation->getManeuver().maneuver << std::endl;
	}
	std::string getNextObjectName();
	double annotateLength(double prevStartPos);
	void updateRtdb(double vehiclePosition);
	void setFromCheckpointEdge() { fromCheckpointEdge = true; }
	void setToCheckpointEdge() { toCheckpointEdge = true; }

	const AnnotationList& getAnnotations(void) {return annotations;}
	bool hasAnnotation(maneuver_t anno);
	void deleteAnnotation(maneuver_t anno);

	RndfEdge* getEdge() { return edge; }
	const RndfEdge* getEdge() const { return edge; }

	void dump();

	bool was_reached; // was the from-Node of this edge reached already? (used for memorizing which checkpoints had been reached btw road_planner crashes
protected:
	RndfEdge * edge;
public:
	double startPosition;
protected:
	AnnotationList annotations;
	bool fromCheckpointEdge, toCheckpointEdge;
};

class Route {
public:
	typedef std::deque<AnnotatedRouteEdge*> RouteEdgeList;
	typedef RndfGraph::EdgeList EdgeList;

	Route();
	virtual ~Route();

	void addEdges( EdgeList * edgeList );
	void addEdge( RndfEdge * edge ); //!< for single-checkpoint missions
	void annotateRoute();
	void init();

	double * computeIntersectionAngles(RndfEdge * edge);
	maneuver_t addVertexAnnotations(AnnotatedRouteEdge* routeEdge, RndfVertex * vertex, RndfEdge * nextEdge, bool isCheckpoint, double currentPosition, area_type_t areaType, double minSpeed, double maxSpeed, bool firstVertex);

	//RouteEdgeList::iterator getCurrentEdgeIt() { return currentEdge; }

	void dump();

	RouteEdgeList route;

protected:
	double veh_x, veh_y, veh_route;
	double routeLength;
	double minSpeed, maxSpeed;
	maneuver_t currentManeuver;
	area_type_t currentArea;
};

} // namespace vlr

} // namspace RoutePlanner

#endif
