/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_RndfVertex.h"
#include "aw_RndfEdge.h"

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfVertex::RndfVertex(double lat, double lon, double x, double y, string name, int id) :
  Vertex<RndfEdge>(id), latitude_(lat), longitude_(lon), x_(x), y_(y), name_(name), isStop(false), isExit(false),
      isCheckpoint(false), isPerimeterPoint(false), isParkingSpot(false), isVirtual(false), isBlocked(false), exit_count_(0),
      checkpoint_id_(-1)/*, m_intersection(NULL)*/ {
}

RndfVertex::~RndfVertex() {
};

bool RndfVertex::hasIntersectionOutEdge(const RndfIntersection* isec) const
{
	for (TEdgeSet::const_iterator i=m_outEdges.begin(); i!=m_outEdges.end(); ++i) {
		if ((*i)->getIntersection() == isec) {
			return true;
		}
	}
	return false;
}
bool RndfVertex::hasIntersectionOutEdge() const
{
	for (TEdgeSet::const_iterator i=m_outEdges.begin(); i!=m_outEdges.end(); ++i) {
		if ((*i)->getIntersection() != 0) {
			return true;
		}
	}
	return false;
}

} // namespace RoutePlanner

} // namespace vlr
