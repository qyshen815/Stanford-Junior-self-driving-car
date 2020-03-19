/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <assert.h>
#include "aw_RndfIntersection.h"
#include "aw_RndfEdge.h"
#include "aw_RndfVertex.h"

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfIntersection::RndfIntersection(int id) :
	id(id),
	sum(CGAL_Geometry::Vector_2(0,0)),
	sum_length(0),
	center_count(0),
	center_(CGAL_Geometry::Point_2(0,0)),
	radius(0),
	radius_calculated(false) {
}

RndfIntersection::~RndfIntersection() {
}

bool RndfIntersection::addEdge(RndfEdge* edge) {
	assert(edge);
	pair<TRndfEdgeSet::iterator,bool> p = edges.insert(edge);
	if (p.second) {
		sum = sum + CGAL_Geometry::Vector_2(edge->fromVertex()->x(), edge->fromVertex()->y())*edge->getLength() + CGAL_Geometry::Vector_2(edge->toVertex()->x(), edge->toVertex()->y())*edge->getLength();
		sum_length += edge->getLength();
		radius_calculated = false;
	}
	return p.second;
}

void RndfIntersection::removeEdge(RndfEdge* edge)
{
	if (edges.find(edge) == edges.end()) return;
	assert(edge->fromVertex() && edge->toVertex());
	sum = sum - CGAL_Geometry::Vector_2(edge->fromVertex()->x(), edge->fromVertex()->y())*edge->getLength() - CGAL_Geometry::Vector_2(edge->toVertex()->x(), edge->toVertex()->y())*edge->getLength();
	sum_length -= edge->getLength();
	radius_calculated = false;
	edges.erase(edge);
}

size_t RndfIntersection::numEdges() const {
	return edges.size();
}

bool RndfIntersection::hasEdge(RndfEdge* edge) const {
	return (edges.find(edge) != edges.end());
}

const TRndfEdgeSet& RndfIntersection::getEdges() const {
	return edges;
}

CGAL_Geometry::Point_2 RndfIntersection::center()
{
	if (center_count != edges.size()) {
		center_ = CGAL_Geometry::Point_2(0,0) + (sum / (sum_length*2.0));
		center_count = edges.size();
	}
	return center_;
}

double RndfIntersection::getRadius()
{
	if (!radius_calculated) {
		center(); // ensure that center is up-to-date
		radius = 0;
		for (TRndfEdgeSet::const_iterator iter = edges.begin(); iter != edges.end(); ++iter) {
			assert((*iter));
			assert((*iter)->toVertex());
			double r = hypot((*iter)->toVertex()->x() - center_.x(), (*iter)->toVertex()->y() - center_.y());
			if (r > radius) {
				radius = r;
			}
		}
		radius_calculated = true;
	}
	return radius;
}

void RndfIntersection::paint_segment_with_offset( RndfEdge* scanning_edge, double offset )
{
  std::cout << "Hi\n";

  assert( scanning_edge );
  // one intersection entry found!

  // scan forw
  while( true )
    {
      std::cout << "Di\n";
      //	  if( scanning_edge->toVertex()->getOutEdges().size() != 1 ) break;

      double x1 = scanning_edge->fromVertex()->x();
      double y1 = scanning_edge->fromVertex()->y();

      double x2 = scanning_edge->toVertex()->x();
      double y2 = scanning_edge->toVertex()->y();

      double dx = x2 - x1;
      double dy = y2 - y1;

      double norm = sqrt( dx*dx + dy*dy );

      double n1 = -dy / norm;
      double n2 =  dx / norm;

      std::cout << "ILINE " << x1 + n1 * offset << " " << y1 + n2 * offset << "\n";
      std::cout << "ILINE " << x2 + n1 * offset << " " << y2 + n2 * offset << "\n  \n  \n";


      scanning_edge = *( scanning_edge->toVertex()->getOutEdges().begin() );

      if( edges.find( scanning_edge ) == edges.end() )
	break;

    }
}

std::vector<RndfIntersection::point> RndfIntersection::get_boundary_lines()
{
  std::vector<RndfIntersection::point> result;
  set<RndfEdge*> entries;
  for( set<RndfEdge*>::iterator it = edges.begin(); it != edges.end(); it++ )
    {
      RndfEdge* scanning_edge = *it;
      // scan backw
      while( true )
	{
	  std::cout << "hop\n";
	  if( edges.find( scanning_edge ) == edges.end() ) break;
	  assert( scanning_edge->fromVertex()->getInEdges().size() == 1 );
	  RndfEdge* prev_scanning_edge = *(scanning_edge->fromVertex()->getInEdges().begin());
	  //	  if( scanning_edge->fromVertex()->getOutEdges().size() != 1 ) break;
	  scanning_edge = prev_scanning_edge;
	}
      entries.insert( scanning_edge );
    }

  std::cout << "# ILINE found " << entries.size() << " entries\n"<< "\n  \n  \n";
  for( set<RndfEdge*>::iterator it = entries.begin(); it != entries.end(); it++ )
    {
      std::cout << "Ho!\n";

      RndfEdge* leftest = 0;
      RndfEdge* rightest = 0;

      double a_min = 999999;
      double a_max = -999999;
      for( set<RndfEdge*>::iterator out_it = (*it)->toVertex()->getOutEdges().begin(); out_it != (*it)->toVertex()->getOutEdges().end(); out_it++ )
	{
	  RndfEdge* preview_edge = *out_it;
	  for( int i=0; i<2; i++ )
	    {
	      if( preview_edge->toVertex()->getOutEdges().size() == 1 )
		{
		  RndfEdge* pre_preview_edge = preview_edge;
		  pre_preview_edge = *(preview_edge->toVertex()->getOutEdges().begin());
		  if( edges.find( pre_preview_edge ) != edges.end() )
		    preview_edge = pre_preview_edge;
		}
	    }
	  double dx = preview_edge->toVertex()->x() -  (*out_it)->fromVertex()->x();
	  double dy = preview_edge->toVertex()->y() -  (*out_it)->fromVertex()->y();
	  double a = atan2( dy, dx );
	  if( a <= a_min )
	    {
	      a_min = a;
	      rightest = *out_it;
	    }
	  if( a >= a_max )
	    {
	      a_max = a;
	      leftest = *out_it;
	    }
	}
      std::cout << "a_min a_max " << a_min << " " << a_max << "\n";
#warning add tolerance here!

      //      paint_segment_with_offset( leftest, 0 );

      //      if( (*it)->getLeftBoundary() == rndf::lane::SolidWhite )

      // einer zurueck!
      RndfEdge* approaching_edge = *it;
      if( approaching_edge->fromVertex()->getInEdges().size() == 1 )
	approaching_edge = *(approaching_edge->fromVertex()->getInEdges().begin());

      if( approaching_edge->getLeftOppositeEdges().size() == 0 )
	{
	  paint_segment_with_offset( leftest,   0.4 * (*it)->getWidth() );
	  paint_segment_with_offset( rightest, -0.4 * (*it)->getWidth() );      	}
      else
	paint_segment_with_offset( rightest, -0.4 * (*it)->getWidth() );
      //      if( approaching_edge->getRightEdges().size() == 0 )


    }
  return result;
}


} //namespace RoutePlanner

} // namespace vlr
