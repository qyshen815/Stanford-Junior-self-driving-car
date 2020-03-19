/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <limits>

//#include <CGAL/squared_distance_2.h>

#include "aw_graph_tools.hpp"
#include "aw_VehicleManager.hpp"
#include "aw_Topology.hpp"
#include "aw_RndfIntersection.h"

#include <aw_CGAL.h>

namespace vlr {

using namespace std;
using namespace GraphTools;
using namespace RoutePlanner;

PlaceOnGraph::PlaceOnGraph(): graph( NULL ), /*edge(  Topology::complete_mission_graph.end()  0 ), */offset( 0. ), valid(false)
{
}

PlaceOnGraph::PlaceOnGraph( Route::RouteEdgeList::const_iterator edge_, double offset_, RoutePlanner::Route::RouteEdgeList& edges )
: graph( &edges ), edge( edge_ ), offset( offset_ ), valid(true)
{
	// edge = std::find( Topology::complete_mission_graph.begin(), Topology::complete_mission_graph.end(), (*edge_) );
}

PlaceOnGraph::PlaceOnGraph( const PlaceOnGraph& original ):
	graph( original.graph ),
	edge( original.edge ),
	offset( original.offset ),
	valid( original.valid )
{
	assert(valid);
}

PlaceOnGraph& PlaceOnGraph::operator=(const PlaceOnGraph& original)
{
	if (this != &original) {
		valid = original.valid;
		edge = original.edge;
		offset = original.offset;
		graph = original.graph;
//		assert(valid);
	}
	return *this;
}

/* seems not to be needed anymore - uses the potentially dangerous function Topology::plain_edge_to_route
PlaceOnGraph::PlaceOnGraph( Vehicle& vehicle, RoutePlanner::Route::RouteEdgeList& edges ): graph( &edges )
{
  assert( vehicle.edge->getAnnotatedEdge() && "This vehicle is not on the mission graph!" );

  edge = Topology::plain_edge_to_route( *graph, vehicle.edge );

  //  edge = std::find( Topology::complete_mission_graph.begin(), Topology::complete_mission_graph.end(), (*edge) );

  // double-gemoubbled
  assert( edge != graph->end() );

  offset = vehicle.distFromStart();

}
 */

void PlaceOnGraph::operator+=( double meter )
{
	assert(valid);
	assert( (*edge) );

	if ( meter < 0. ) {
		*this -= ( - meter );
		return;
	}

	while( true )
	{
		double current_edge_length = (*edge)->getEdge()->getLength();
		if( ( current_edge_length - meter - offset) > 0 )
		{
			offset += meter;
			// std::cout << "operator+: return1" << std::endl;
			return;
		}
		else
		{
			meter -= ( current_edge_length - offset );
			offset = 0;

			// std::cout << "operator+: next round" << std::endl;
			if( *edge == graph->back() )
			{
				offset = current_edge_length;
				// std::cout << "operator+: return2" << std::endl;
				return;
			}
			edge++;
		}
	}
}

void PlaceOnGraph::operator-=( double meter )
{
	assert(valid);

	if ( meter < 0. ) {
		*this += ( - meter );
		return;
	}

	assert( (*edge) );
	while( true )
	{
		if( ( offset - meter ) > 0 )
		{
			offset -= meter;
			//	    std::cout << "operator-: return1" << std::endl;
			return;
		}
		else
		{
			meter -= ( offset );

			if( *edge == graph->front() )
			{
				offset = 0;
				// std::cout << "operator-: return2" << std::endl;
				return;
			}

			offset = (*edge)->getEdge()->getLength();
			edge--;
		}
	}
}


Point PlaceOnGraph::make_point() const
{
	assert(valid);
	double scale = offset / (*edge)->getEdge()->getLength();
	double x = (1.-scale) * (*edge)->getEdge()->fromVertex()->x() + scale * (*edge)->getEdge()->toVertex()->x();
	double y = (1.-scale) * (*edge)->getEdge()->fromVertex()->y() + scale * (*edge)->getEdge()->toVertex()->y();

	return Point( x, y );
}

Point_2 PlaceOnGraph::point() const
{
	return (*edge)->getEdge()->point( offset );
}


double GraphTools::difference( const PlaceOnGraph& second, const PlaceOnGraph& first )
{
	assert(first.isValid());
	assert(second.isValid());

	if( ( (*first.edge)==0 || (*first.edge)->getEdge() ) == 0 ) return -1;
	if( ( (*second.edge)==0 || (*second.edge)->getEdge() ) == 0 ) return -1;

	//assert( first.Topology::complete_mission_graph.front() == second.Topology::complete_mission_graph.front() && first.Topology::complete_mission_graph.back() == second.Topology::complete_mission_graph.back() );
	//    std::cout << "difference says hello agagin" << std::endl;

	assert( first.graph == second.graph );

	double diff = -first.offset;
	Route::RouteEdgeList::const_iterator edge = first.edge;

	if( *(edge) == *(second.edge) )
	{
		diff = second.offset - first.offset;
		if (diff > 0.) return diff;
		else return -1.;
	}

	while( true )
	{
		diff += (*edge)->getEdge()->getLength();
		// std::cout << "difference: advancing edge " << diff << std::endl;
		edge++;
		if( *edge == first.graph->back() ) return -1;
		if( *edge == *(second.edge) ) return diff + second.offset;
	}
}

// return the place with the next obstacle.
Vehicle* PlaceOnGraph::next_obstacle(double& d, PlaceOnGraph& place, double max_scan_distance) {
  assert(valid);
  d = -offset;
  bool is_on_current = true;

  for (RoutePlanner::Route::RouteEdgeList::const_iterator it = edge;; it++) {
    //std::cout << "d == " << d << "\n";
    if (d > max_scan_distance) {
      place = PlaceOnGraph();
      return NULL;
    }

    if ((*it)->getEdge()->vehicles_on_edge.size() > 0) {
      // this is the edge
      double smallest_offset = std::numeric_limits<double>::infinity();

      Vehicle* veh = NULL;
      // check all vehicles on edge, smallest offset wins
      for (std::map<int, Vehicle*>::iterator vit = (*it)->getEdge()->vehicles_on_edge.begin(); vit != (*it)->getEdge()->vehicles_on_edge.end(); ++vit) {
        double v_dist_from_start = vit->second->distFromStart();
        if(!vit->second) {
          printf("VEHICLE/EDGE MATCH: Invalid vehicle entry (%i)\n", int(vit->first));
        }
        if (vit->second->edge() != (*it)->getEdge()) {
          // Match becomes incosistent when multiple fits are possible (e.g. lane change edges)
          // That doesn't hurt but isn't nice either, maybe sometime...
//          printf("MATCH INCONSISTENT FOR VEHICLE %i / EDGES: %s-%s <-> %s-%s\n", vit->second->id(),
//              vit->second->edge()->fromVertex()->name().c_str(), vit->second->edge()->toVertex()->name().c_str(),
//              (*it)->getEdge()->fromVertex()->name().c_str(), (*it)->getEdge()->toVertex()->name().c_str());
//          v_dist_from_start = vit->second->edges[(*it)->getEdge()];
          map< RoutePlanner::RndfEdge*, double >::const_iterator dist_it = vit->second->edges().find((*it)->getEdge());
          if(dist_it == vit->second->edges().end()) {
            printf("VEHICLE/EDGE MATCH: Invalid entry int dist map (0x%llx)\n", (unsigned long long int)(*it)->getEdge());
          }
          v_dist_from_start = vit->second->edges()[(*it)->getEdge()];
        }
        if (v_dist_from_start < smallest_offset) {
          bool take_it = false;
          if (!is_on_current)
            take_it = true;
          else if (v_dist_from_start > offset) take_it = true;

          if (take_it) {
            smallest_offset = v_dist_from_start;
            //cout << setprecision(14) << smallest_offset << " <= " << (*it)->getEdge()->getLength() << " +- " << 2.0*std::numeric_limits<double>::epsilon() << " = " << ((*it)->getEdge()->getLength()-smallest_offset) << endl;
            //assert(((*it)->getEdge()->getLength() - smallest_offset) > -2.0*std::numeric_limits<double>::epsilon() );
            veh = vit->second;
          }
        }
      }

      if (smallest_offset > (*it)->getEdge()->getLength()) {
        // only the case if we found vehicles on our segment, but they are all behind us
        is_on_current = false;
        d += (*it)->getEdge()->getLength();
        if ((*it) == graph->back()) break;
        continue;
      }

      d += smallest_offset;
      place = PlaceOnGraph(it, smallest_offset, *graph);
      return veh;
    }
    is_on_current = false;
    d += (*it)->getEdge()->getLength();

    if ((*it) == graph->back()) break;
  }

  place = PlaceOnGraph();
  return NULL;
}

// return the place with the previous obstacle.
Vehicle* PlaceOnGraph::prev_obstacle( double& d, PlaceOnGraph& place, double max_scan_distance )
{
	assert(valid);
	d = -(*edge)->getEdge()->getLength() + offset;
	bool is_on_current = true;

	for( RoutePlanner::Route::RouteEdgeList::const_iterator it = edge; ; --it )
	{

		if( d > max_scan_distance ) {
			place = PlaceOnGraph();
			return 0;
		}

		d += (*it)->getEdge()->getLength();

		if( (*it)->getEdge()->vehicles_on_edge.size() > 0 )
		{
			// this is the edge
			double biggest_offset = -1;

			Vehicle* veh = NULL;
			// check all vehicles on edge, biggest offset wins
			for( std::map<int, Vehicle*>::iterator vit = (*it)->getEdge()->vehicles_on_edge.begin();
			vit != (*it)->getEdge()->vehicles_on_edge.end();
			vit++ )
			{
				double v_dist_from_start = vit->second->distFromStart();
				if (vit->second->edge() != (*it)->getEdge()) {
					v_dist_from_start = vit->second->edges()[(*it)->getEdge()];
				}
				if( v_dist_from_start > biggest_offset )
				{
					bool take_it = false;
					if( !is_on_current ) take_it = true;
					else
						if( v_dist_from_start < offset ) take_it = true;

					if( take_it )
					{
						biggest_offset = v_dist_from_start;
						//cout << setprecision(14) << smallest_offset << " <= " << (*it)->getEdge()->getLength() << " +- " << 2.0*std::numeric_limits<double>::epsilon() << " = " << ((*it)->getEdge()->getLength()-smallest_offset) << endl;
						//assert(((*it)->getEdge()->getLength() - biggest_offset) > -2.0*std::numeric_limits<double>::epsilon() );
						veh = vit->second;
					}
				}
			}


			if( biggest_offset < 0 )
			{
				// only the case if we found vehicles on our segment, but they are all in front of us

				is_on_current = false;
				if( (*it) == graph->front() ) break;
				continue;
			}

			d -= biggest_offset;
			place = PlaceOnGraph( it, biggest_offset, *graph );
			return veh;
		}
		is_on_current = false;

		if( (*it) == graph->front() ) break;
	}

	place = PlaceOnGraph();
	return 0;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::next_manuever( double& d, PlaceOnGraph& place, double max_scan_distance, const maneuver_t maneuver )
{
	vector<maneuver_t> maneuvers(1);
	maneuvers.push_back(maneuver);
	return next_manuever(d, place, max_scan_distance, maneuvers);
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::next_manuever( double& d, PlaceOnGraph& place, double max_scan_distance, const vector<maneuver_t>& maneuvers )
{
	assert(valid);
	d = -offset;
	bool is_on_current = true;

	for( RoutePlanner::Route::RouteEdgeList::const_iterator it = edge; ; it++ )
	{
		//std::cout << "d == " << d << "\n";
		if( d > max_scan_distance ) {
			place = PlaceOnGraph();
			return 0;
		}

		d += (*it)->getEdge()->getLength();
		AnnotatedRouteEdge::AnnotationList annos = (*it)->getAnnotations();
		for (AnnotatedRouteEdge::AnnotationList::iterator ait=annos.begin(); ait != annos.end(); ++ait) {
			for (vector<maneuver_t>::const_iterator m_it=maneuvers.begin(); m_it != maneuvers.end(); ++m_it) {
				if ((*ait)->maneuver() == *m_it) {
					place = PlaceOnGraph( it, 0, *graph );
					return (*it);
				}
			}
		}
		is_on_current = false;
		if( (*it) == graph->back() ) break;
	}

	place = PlaceOnGraph();
	return 0;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::next_area( double& d, PlaceOnGraph& place, double max_scan_distance, const area_type_t area )
{
	assert(valid);
	d = -offset;
	bool is_on_current = true;

	for( RoutePlanner::Route::RouteEdgeList::const_iterator it = edge; ; it++ )
	{
		//std::cout << "d == " << d << "\n";
		if( d > max_scan_distance ) {
			place = PlaceOnGraph();
			return 0;
		}
		AnnotatedRouteEdge::AnnotationList annos = (*it)->getAnnotations();
		for (AnnotatedRouteEdge::AnnotationList::iterator ait=annos.begin(); ait != annos.end(); ++ait) {
			if ((*ait)->areaType() == area) {
			  if(d<0) d=0;
				place = PlaceOnGraph( it, 0, *graph );
				return (*it);
			}
		}
		d += (*it)->getEdge()->getLength();
		is_on_current = false;
		if( (*it) == graph->back() ) break;
	}

	place = PlaceOnGraph();
	return 0;
}


RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_fwd_to_intersection( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec )
{
	assert(valid);

	// Prüfen ob man sich schon auf der Intersection befindet
	if ((*edge)->getEdge()->getIntersection()) {
		if ( isec == NULL || (*edge)->getEdge()->getIntersection()->getId() == isec->getId() ) {
			d = 0.;
			return *edge;
		} else {
			d =std::numeric_limits<double>::infinity();
			return NULL;
		}
	}

	// Suche Init
	d = -offset;
	offset = 0.;

	// Über Edges iterieren und nach Intersection suchen
	for(; ; ++edge )
	{
		// Auf Intersection testen
		if ((*edge)->getEdge()->getIntersection()) {
			if ( isec == NULL || (*edge)->getEdge()->getIntersection()->getId() == isec->getId())
				return *edge;
			else {
				d =std::numeric_limits<double>::infinity();
				return NULL;
			}
		}

		// Distanz aufsummieren
		d += (*edge)->getEdge()->getLength();

		// Max Scandistanz prüfen
		//std::cout << "d == " << d << "\n";
		if( d > max_scan_distance ) {
			d = std::numeric_limits<double>::infinity();
			return NULL;
		}

		if (*edge == graph->back()) break;
	}

	d = std::numeric_limits<double>::infinity();
	return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_bwd_to_intersection( double& d,
                              double /*max_scan_distance*/, const RoutePlanner::RndfIntersection* /*isec*/ )
{
	assert(false); // TODO Bei Bedarf impln
	// Auf Abbruch Bedingung aufpassen
	d = std::numeric_limits<double>::infinity();
	return NULL;
}


RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_fwd_to_stopline(double& d, double max_scan_distance,
    const RoutePlanner::RndfIntersection* isec) {
  assert(valid);

  // Suche Init
  d = -offset;

  // iterate over edges and find next stop line
  for (;; ++edge) {
    // sum up edge lengths
    d += (*edge)->getEdge()->getLength();

    // test for stop line and intersection
    RndfVertex* to_vertex = (*edge)->getEdge()->toVertex();
    assert( to_vertex );
    if (to_vertex->isStopVertex() && (isec == NULL || to_vertex->hasIntersectionOutEdge(isec))) {return *edge;}

    // check max scan distance
    if (d > max_scan_distance) {
      d = std::numeric_limits<double>::infinity();
      return NULL;
    }

    if (*edge == graph->back()) break;
    offset = 0.;
  }

  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_bwd_to_stopline( double& d,
                  double /*max_scan_distance*/, const RoutePlanner::RndfIntersection* /*isec*/ )
{
	assert(false); // TODO Bei Bedarf impln
	// Auf Abbruch Bedingung aufpassen
	d = std::numeric_limits<double>::infinity();
	return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_fwd_to_traffic_light(double& d, double max_scan_distance,
    const RoutePlanner::RndfIntersection* isec) {
  assert(valid);

  // Suche Init
  d = -offset;

  // iterate over edges and find next stop line
  for (;; ++edge) {
    // sum up edge lengths
    d += (*edge)->getEdge()->getLength();

    // test for stop line and intersection
    RndfVertex* to_vertex = (*edge)->getEdge()->toVertex();
//    printf("x: %f, y: %f, name: %s, %s\n", to_vertex->latitude(), to_vertex->longitude(), to_vertex->name().c_str(), (to_vertex->isTrafficLightVertex() ? "TL" : "-"));
    assert( to_vertex );
    if (to_vertex->isTrafficLightVertex() && (isec == NULL || to_vertex->hasIntersectionOutEdge(isec))) {return *edge;}

    // check max scan distance
    if (d > max_scan_distance) {
      d = std::numeric_limits<double>::infinity();
      return NULL;
    }

    if (*edge == graph->back()) break;
    offset = 0.;
  }

  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_bwd_to_traffic_light( double& d,
                  double /*max_scan_distance*/, const RoutePlanner::RndfIntersection* /*isec*/ )
{
  assert(false); // TODO Bei Bedarf impln
  // Auf Abbruch Bedingung aufpassen
  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_fwd_to_crosswalk(double& d, double max_scan_distance) {

  // Suche Init
  d = -offset;

  // iterate over edges and find next stop line
  for (;; ++edge) {
    // sum up edge lengths
    d += (*edge)->getEdge()->getLength();

    // test for crosswalk
    RndfVertex* to_vertex = (*edge)->getEdge()->toVertex();
//    printf("x: %f, y: %f, name: %s, %s\n", to_vertex->latitude(), to_vertex->longitude(), to_vertex->name().c_str(), (to_vertex->isTrafficLightVertex() ? "TL" : "-"));
    assert( to_vertex );
    if (to_vertex->isCrosswalkVertex()) {return *edge;}

    // check max scan distance
    if (d > max_scan_distance) {
      d = std::numeric_limits<double>::infinity();
      return NULL;
    }

    if (*edge == graph->back()) break;
    offset = 0.;
  }

  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_bwd_to_crosswalk( double& d, double /*max_scan_distance*/)
{
  assert(false); // TODO Bei Bedarf impln
  // Auf Abbruch Bedingung aufpassen
  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_fwd_to_traffic_light(double& d, double max_scan_distance) {

  // Suche Init
  d = -offset;

  // iterate over edges and find next stop line
  for (;; ++edge) {
    // sum up edge lengths
    d += (*edge)->getEdge()->getLength();

    // test for crosswalk
    RndfVertex* to_vertex = (*edge)->getEdge()->toVertex();
//    printf("x: %f, y: %f, name: %s, %s\n", to_vertex->latitude(), to_vertex->longitude(), to_vertex->name().c_str(), (to_vertex->isTrafficLightVertex() ? "TL" : "-"));
    assert( to_vertex );
    if (to_vertex->isTrafficLightVertex()) {return *edge;}

    // check max scan distance
    if (d > max_scan_distance) {
      d = std::numeric_limits<double>::infinity();
      return NULL;
    }

    if (*edge == graph->back()) break;
    offset = 0.;
  }

  d = std::numeric_limits<double>::infinity();
  return NULL;
}

RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::go_bwd_to_traffic_light( double& d, double /*max_scan_distance*/)
{
  assert(false); // TODO Bei Bedarf impln
  // Auf Abbruch Bedingung aufpassen
  d = std::numeric_limits<double>::infinity();
  return NULL;
}


RoutePlanner::AnnotatedRouteEdge* PlaceOnGraph::next_zone_exit( double& d, PlaceOnGraph& place, double max_scan_distance )
{
	assert(valid);
	d = -offset;
	bool is_on_current = true;

	for( RoutePlanner::Route::RouteEdgeList::const_iterator it = edge; ; it++ )
	{
		//std::cout << "d == " << d << "\n";
		if( d > max_scan_distance ) {
			place = PlaceOnGraph();
			return 0;
		}

		d += (*it)->getEdge()->getLength();
		// Assumption: a perimiter point cannot be an exit and entry point at the same time
		if ((*it)->getEdge()->fromVertex()->isPerimeterPointVertex() && (*it)->getEdge()->fromVertex()->isExitVertex()) {
			place = PlaceOnGraph( it, 0, *graph );
			return (*it);
		}
		is_on_current = false;
		if( (*it) == graph->back() ) break;
	}

	place = PlaceOnGraph();
	return 0;
}

// this is completely unoptimized code
std::set<RndfEdge*> GraphTools::getSurroundingEdges(RndfEdge* edge, RndfGraph* complete_graph, double search_distance)
{
	using namespace CGAL_Geometry;

	assert(edge);
	assert(complete_graph);

	std::set<RndfEdge*> edges;
	Segment_2 edgeSeg = Segment_2(
			Point_2(edge->fromVertex()->x(), edge->fromVertex()->y()),
			Point_2(edge->toVertex()->x(), edge->toVertex()->y()));
	double sqr_search_dist = search_distance * search_distance;

	for (RndfGraph::EdgeMap::const_iterator iter=complete_graph->edgeMap.begin(); iter != complete_graph->edgeMap.end(); ++iter) {
		if (iter->second == edge) continue;
		Segment_2 iterSeg = Segment_2(
					Point_2((iter->second)->fromVertex()->x(), (iter->second)->fromVertex()->y()),
					Point_2((iter->second)->toVertex()->x(), (iter->second)->toVertex()->y()));

		double sqr_dist = squared_distance(edgeSeg, iterSeg);
		if (sqr_dist <= sqr_search_dist) {
			edges.insert(iter->second);
		}
	}

	return edges;
}

std::map<int, RoutePlanner::RndfEdge*> GraphTools::getSiblingEdges(RoutePlanner::RndfEdge* /*edge*/, double /*max_search_distance*/)
{
	std::map<int, RoutePlanner::RndfEdge*> edges;

	return edges;
}

} // namespace vlr
