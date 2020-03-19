/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDFINTERSECTION_H
#define AW_RNDFINTERSECTION_H

#include <string>
#include <set>
#include <boost/utility.hpp>
#include <aw_CGAL.h>

namespace vlr {

namespace RoutePlanner {
class RndfGraphBuilder;
class RndfEdge;
typedef std::set<RndfEdge*> TRndfEdgeSet;


class RndfIntersection : boost::noncopyable
{
  struct point {
    double x;
    double y;
  };

public:
	RndfIntersection(int id);
	~RndfIntersection();

	bool addEdge(RndfEdge* edge);
	void removeEdge(RndfEdge* edge);
	size_t numEdges() const;
	bool hasEdge(RndfEdge* edge) const;

	const TRndfEdgeSet& getEdges() const;

	int getId() const {return id; }

	CGAL_Geometry::Point_2 center();

	double getRadius();

	void paint_segment_with_offset( RndfEdge* scanning_edge, double offset );

	std::vector<point> get_boundary_lines();
protected:
	int id;
private:
	TRndfEdgeSet edges;
	CGAL_Geometry::Vector_2 sum;
	double sum_length;
	unsigned int center_count;
	CGAL_Geometry::Point_2 center_;
	double radius;
	bool radius_calculated;
};


} // namespace RoutePlanner

} // namespace vlr

#endif // AW_RNDFINTERSECTION_H
