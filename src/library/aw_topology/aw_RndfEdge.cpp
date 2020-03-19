/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <math.h>
#include <sla.h>

#include "aw_RndfVertex.h"
#include "aw_RndfEdge.h"
#include "aw_RndfIntersection.h"

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfEdge::RndfEdge(RndfVertex * v1, RndfVertex * v2, string name, bool isLane, bool isVirtual, bool isLaneChange, double minSpeed,
		double maxSpeed, bool isOffroad, double weight, int id) :
      Edge<RndfVertex>(v1, v2, weight, id), is_crossing_virtual_segment( false ), name_(name),
      isLane(isLane), isCircle(false), isVirtual(isVirtual), isZone(false), isUTurn(false),
      isLaneChange(isLaneChange), isLeftLaneChange(false), isRightLaneChange(false),
      isLeftOppositeLaneChange(false), isStopLane(false), isBlocked(false), isOffroad(isOffroad),
      hasWidth(false), minSpeed(minSpeed), maxSpeed(maxSpeed), length(0), width(4.5),
			leftBoundary(rndf::Lane::UnknownBoundary), rightBoundary(rndf::Lane::UnknownBoundary), intersection_(NULL)
{
	computeLength();
	computeTravelTime();
	if (from) {from->addOutEdge(this);}
	if (to) {to->addInEdge(this);}
}

RndfEdge::RndfEdge(double from_x, double from_y, double to_x, double to_y) :
  Edge<RndfVertex>(new RndfVertex( 0, 0, from_x, from_y ), new RndfVertex( 0, 0, to_x, to_y ), 1., 0), is_crossing_virtual_segment( false ),
	name_("plane"), isLane(true), isCircle(false), isVirtual(false), isZone(false), isUTurn(false),
	isLaneChange(false), isLeftLaneChange(false), isRightLaneChange(false), isLeftOppositeLaneChange(false), isStopLane(false), isBlocked(false), isOffroad(false),
	hasWidth(false), minSpeed(0), maxSpeed(0), length(0), width(4.5),
	leftBoundary(rndf::Lane::UnknownBoundary), rightBoundary(rndf::Lane::UnknownBoundary), intersection_(NULL)
{
  computeLength();
  computeTravelTime();
  if (from) from->addOutEdge(this);
  if (to) to->addInEdge(this);
}

  void RndfEdge::copy_attributes_from( RndfEdge& old_edge )
  {

    isLane = old_edge.isLane;
    isCircle = old_edge.isCircle;

    isZone = old_edge.isZone;
    isUTurn = old_edge.isUTurn;
    isLaneChange = old_edge.isLaneChange;
    isLeftLaneChange = old_edge.isLeftLaneChange;
    isRightLaneChange = old_edge.isRightLaneChange;
    isLeftOppositeLaneChange = old_edge.isLeftOppositeLaneChange;
    isStopLane = old_edge.isStopLane;
    isBlocked = old_edge.isBlocked;
    hasWidth = old_edge.hasWidth;

  }

RndfEdge::~RndfEdge()
{
	// remove links
	if (intersection_) {intersection_->removeEdge(this);}
	for (TRndfEdgeSet::iterator it = leftEdges.begin(); it != leftEdges.end(); ++it)
		(*it)->rightEdges.erase(this);
	for (TRndfEdgeSet::iterator it = rightEdges.begin(); it != rightEdges.end(); ++it)
		(*it)->leftEdges.erase(this);
	for (TRndfEdgeSet::iterator it = leftOppositeEdges.begin(); it != leftOppositeEdges.end(); ++it)
		(*it)->leftOppositeEdges.erase(this);
	for (TRndfEdgeSet::iterator it = crossingEdges.begin(); it != crossingEdges.end(); ++it)
		(*it)->crossingEdges.erase(this);
}


void RndfEdge::computeLength() {
	RndfVertex * from = this->from;
	RndfVertex * to = this->to;
	length = sqrt(sla::sqr(from->x() - to->x()) + sla::sqr(from->y() - to->y()));
}

void RndfEdge::computeTravelTime() {
	travelTime = length / maxSpeed;
}

void RndfEdge::setWidth(double width) {
	this->width = width;
	hasWidth = true;
}

void RndfEdge::computePosition(double offset, double & x, double & y) {
	double factor = offset/length; // check: must be in [0,1] !
	x = factor * toVertex()->x() + (1-factor) * fromVertex()->x();
	y = factor * toVertex()->y() + (1-factor) * fromVertex()->y();
}

void RndfEdge::computePosition(double offset, double & x, double & y, double & yawangle) {
	computePosition(offset, x, y);
	yawangle = atan2(toVertex()->y() - fromVertex()->y(), toVertex()->x() - fromVertex()->x());
}

void RndfEdge::computeDistance(double x, double y, double & latDist, double & lonDist, double & lonOffset) {
	// better: precompute and store dx, dy, c
	double dx = toVertex()->y() - fromVertex()->y();
	double dy = fromVertex()->x() - toVertex()->x();
	double normalLength = sqrt(sla::sqr(dx) + sla::sqr(dy));
	dx /= normalLength;
	dy /= normalLength;
	double c = -toVertex()->x() * dx - toVertex()->y() * dy;
	latDist = x * dx + y * dy + c;

	// orthogonal projection onto lane's center line
	double opx = x - dx * latDist;
	double opy = y - dy * latDist;
	assert(fabs(opx * dx + opy * dy + c) < .0001);
	double xdiff = fromVertex()->x() - opx;
	double ydiff = fromVertex()->y() - opy;
	bool forward;
	if (fabs(xdiff) > fabs(ydiff)) {
		if ( (xdiff<0 && dy<0) || (xdiff>0 && dy>0) ) {
			forward = true;
		} else {
			forward = false;
		}
	} else {
		if ( (ydiff<0 && dx>0) || (ydiff>0 && dx<0) ) {
			forward = true;
		} else {
			forward = false;
		}
	}
	lonOffset = sqrt(sla::sqr(xdiff) + sla::sqr(ydiff));
	if (forward) {
		lonDist = max(lonOffset - length, 0.);
	} else {
		lonOffset = - lonOffset;
		lonDist = lonOffset;
	}
}


Point_2 RndfEdge::point(double offset) const
{
	Point_2 p1( from->x(), from->y() );
	Point_2 p2( to->x(), to->y() );
	if (offset > length) offset = length;
	if (offset < 0.) offset = 0.;
	return p1 + ( p2 - p1 ) * offset / length;
}

Point_2 RndfEdge::fromVertexPoint() const
{
	return from->point();
}

Point_2 RndfEdge::toVertexPoint() const
{
	return to->point();
}

Vector_2 RndfEdge::getVector() const
{
	return Vector_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

Segment_2 RndfEdge::getSegment() const
{
	return Segment_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

Line_2 RndfEdge::getLine() const
{
	return Line_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------

std::ostream& operator << (std::ostream& ostrm, const RndfEdge& obj)
{
	ostrm << "RndfEdge ["<< obj.name() <<"]";
	if (obj.isLaneEdge()) ostrm << " isLaneEdge";
	if (obj.isVirtualEdge()) ostrm << " isVirtualEdge";
	if (obj.toVertex() && obj.toVertex()->isStopVertex())  ostrm << " hasStopLine";
	return ostrm;
}

}

} // namespace vlr
