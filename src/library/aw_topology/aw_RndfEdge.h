/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDFEDGE_H
#define AW_RNDFEDGE_H

#include <string>
#include <iostream>
#include <aw_lane.h>
#include <aw_Graph.h>
#include <aw_CGAL.h>

namespace vlr {

// forward declarations
struct Vehicle;

namespace RoutePlanner {
using CGAL_Geometry::Point_2;
using CGAL_Geometry::Vector_2;
using CGAL_Geometry::Segment_2;
using CGAL_Geometry::Line_2;

class RndfGraphBuilder;
class RndfVertex;
class RndfIntersection;
class RndfEdge;

typedef set<RndfEdge*> TRndfEdgeSet;


class RndfEdge : public Edge<RndfVertex> {
public:
	friend class RndfGraph;
	friend class RndfGraphBuilder;

	RndfEdge(RndfVertex* v1, RndfVertex* v2, std::string name, bool isLane, bool isVirtual, bool isLaneChange, double minSpeed, double maxSpeed, bool isOffroad, double weight, int id);

	// create a plain edge with new vertices, no name, no annotaions, no lat-long...
	// I use this as a quick hack and only for doing graph matching
	RndfEdge( double from_x, double from_y, double to_x, double to_y );

	virtual ~RndfEdge();

	virtual RndfVertex* fromVertex() { return from; }
	virtual RndfVertex const * fromVertex() const { return from; }
	virtual RndfVertex* toVertex() { return to; }
	virtual RndfVertex const * toVertex() const { return to; }
	std::string name() const { return name_; }
	void copy_attributes_from( RndfEdge& );
	bool isBlockedEdge() const { return isBlocked; }
	bool isLaneEdge() const { return isLane; }
	bool isZoneEdge() const { return isZone; }
	bool isCircleEdge() const { return isCircle; }
	bool isVirtualEdge() const { return isVirtual; }
	bool isLaneChangeEdge() const { return RndfEdge::isLaneChange; }
	bool isLeftLaneChangeEdge() const { return isLeftLaneChange; }
	bool isRightLaneChangeEdge() const { return isRightLaneChange; }
	bool isLeftOppositeLaneChangeEdge() const { return isLeftOppositeLaneChange; }
	bool isUTurnEdge() const { return isUTurn; }
	bool isStopLaneEdge() const { return isStopLane; }
	bool isPriorityLaneEdge() const { return !isStopLane; }
	bool isIntersectionEdge() const { return (intersection_ != NULL); }
	bool isOffroadEdge() const { return isOffroad; }
	bool hasLaneWidth() const { return hasWidth; }
	double getWidth() const { return width; }
	double getLength() const { return length; }
	double getTravelTime() const { return travelTime; }
	double getMinSpeed() const { return minSpeed; }
	double getMaxSpeed() const { return maxSpeed; }
	double getLaneWidth() const { return width; }
	rndf::Lane::eBoundaryTypes getLeftBoundary() const { return leftBoundary; }
	rndf::Lane::eBoundaryTypes getRightBoundary() const { return rightBoundary; }
	RndfIntersection* getIntersection() const { return intersection_; }

	void setBlocked(bool v = true) { isBlocked = v; }
	void setZone() { isZone = true; }
	void setUTurn(bool v = true) { isUTurn = v; }
	void setStopLane(bool v = true) { isStopLane = v; }
	void setLeftLaneChange(bool v = true) { isLeftLaneChange = v; }
	void setRightLaneChange(bool v = true) { isRightLaneChange = v; }
	void setLeftoppositeLaneChange(bool v = true) { isLeftOppositeLaneChange = v; }
	void setOffroad(bool v = true) { isOffroad = v; }

	void setWidth(double width);
	void setLeftBoundary(rndf::Lane::eBoundaryTypes boundary) { leftBoundary = boundary; }
	void setRightBoundary(rndf::Lane::eBoundaryTypes boundary) { rightBoundary = boundary; }

	void setVirtual() { isVirtual = true; }

	virtual void computePosition(double offset, double & x, double & y);
	virtual void computePosition(double offset, double & x, double & y, double & yawangle);
	virtual void computeDistance(double x, double y, double & latDist, double & lonDist, double & lonOffset);

	std::map<int, Vehicle*> vehicles_on_edge;

	bool hasLeftEdge(RndfEdge* edge)			{ return leftEdges.find(edge) != leftEdges.end(); }
	bool hasRightEdge(RndfEdge* edge)			{ return rightEdges.find(edge) != rightEdges.end(); }
	bool hasLeftOppositeEdge(RndfEdge* edge)	{ return leftOppositeEdges.find(edge) != leftOppositeEdges.end(); }
	bool hasCrossingEdge(RndfEdge* edge)		{ return crossingEdges.find(edge) != crossingEdges.end(); }

	const TRndfEdgeSet& getLeftEdges() const			{ return leftEdges; }
	const TRndfEdgeSet& getRightEdges() const			{ return rightEdges; }
	const TRndfEdgeSet& getLeftOppositeEdges() const	{ return leftOppositeEdges; }
	const TRndfEdgeSet& getCrossingEdges() const 		{ return crossingEdges; }

	Point_2 point(double offset) const;
	Point_2 fromVertexPoint() const;
	Point_2 toVertexPoint() const;
	Vector_2 getVector() const;
	Segment_2 getSegment() const;
	Line_2 getLine() const;
	double getAngle() const { return CGAL_Geometry::angle( getVector() ); }

	bool is_crossing_virtual_segment;
public:
	virtual void computeLength();
	virtual void computeTravelTime();

	std::string name_;

	bool isLane;
	bool isCircle;
	bool isVirtual;
	bool isZone;
	bool isUTurn;
	bool isLaneChange;
	bool isLeftLaneChange;
	bool isRightLaneChange;
	bool isLeftOppositeLaneChange;
	bool isStopLane;
	bool isBlocked;
	bool isOffroad;
	bool hasWidth;

	double minSpeed, maxSpeed;   //!< [m/s]
	double length;               //!< [m]
	double travelTime;           //!< [s]
	double width;                //!< [m]

	rndf::Lane::eBoundaryTypes leftBoundary;   // TODO: make independent of rndf library...
	rndf::Lane::eBoundaryTypes rightBoundary;

	RndfIntersection* intersection_;

	//! set of left adjacent eges in the same direction
	TRndfEdgeSet leftEdges;
	//! set of right adjacent edges in the same direction
	TRndfEdgeSet rightEdges;
	//! set of left adjacent edges in the opposite direction
	TRndfEdgeSet leftOppositeEdges;

	//! set of crossing edges
	TRndfEdgeSet crossingEdges;
};

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------
std::ostream& operator << (std::ostream& ostrm, const RndfEdge& obj);

}

} // namespace vlr

#endif // AW_RNDFEDGE_H
