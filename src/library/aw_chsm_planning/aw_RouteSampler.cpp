/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <aw_CGAL.h>
#include "aw_RouteSampler.hpp"

using namespace std;

namespace vlr {

using namespace RoutePlanner;

#undef TRACE
#define TRACE(str) cout << "[RouteSampler][" << __FUNCTION__ << "] " << str << endl;

RouteSampler::RouteSampler(Topology* top) : top_(top), graph_(NULL), route_(NULL), edges_(NULL),
                                            lastOffsets_() {
  assert(top);
  graph_ = top_->complete_graph;
  assert(graph_);
  route_ = &top_->route;
  edges_ = &route_->route;

    // temporary curve point will be initialized later
  cp_.s = 0;
  cp_.theta = 0;
  cp_.kappa = 0;
  cp_.kappa_prime = 0;
}

RouteSampler::~RouteSampler() {
}

// TODO: Implement sampleLane()
//
//bool
//RouteSampler::samplePoints( CurvePoints* curve, double back_sampl_dist, double length, int point_anz, double start_lateral_offset, bool make_step, double dist_to_step, double gap_length, double end_lateral_offset ) {
//  return RouteSampler::samplePoints(top_->current_edge_it, top_->ego_vehicle.dist_from_start, edges_, curve, back_sampl_dist, length, point_anz, start_lateral_offset, make_step, dist_to_step, gap_length, end_lateral_offset);
//}
//
//bool RouteSampler::samplePoints(Route::RouteEdgeList::iterator edge_it, double dist_from_start, Route::RouteEdgeList* edges, CurvePoints* curve, double back_sampl_dist, double length, int point_anz, double start_lateral_offset, bool make_step, double dist_to_step, double gap_length, double end_lateral_offset )
//{
//  //assert(length >= 0.0);
//  length = std::max( length, -back_sampl_dist - 0.1 );
//  // Ergebnis struct ini
//  curve->numberValidPoints = 0;
//  assert(point_anz <= C2_CURVEPOINTS_POINTSMAX);
//  if (point_anz > C2_CURVEPOINTS_POINTSMAX) point_anz = C2_CURVEPOINTS_POINTSMAX;
//
//  if (edges->end() == edge_it) {
//    TRACE("iterator at end");
//    return false;
//  }
//
//  // Um back_sampl_dist zurück gehen
//  double offset = dist_from_start + back_sampl_dist;
//  double delta_to_start_offset = back_sampl_dist;
//  //TRACE("Akt. Edge Off: "<< top->ego_vehicle.dist_from_start <<" + "<<back_sampl_dist << " = "<< offset);
//  RndfEdge* edge = (*edge_it)->getEdge();
//  assert(edge);
//  if (!edge) {
//    TRACE("edge is null (1)");
//    return false;
//  }
//  while ((offset < 0) && (edge_it != edges->begin())) {
//    --edge_it;
//    edge = (*edge_it)->getEdge();
//    assert(edge);
//    if (!edge) {
//      TRACE("edge is null (2)");
//      return false;
//    }
//    offset += edge->getLength();
//    //TRACE("Backsampling: Gehe eine Kante zurück -> offset + edge.length("<<  edge->getLength() <<") = "<< offset);
//  }
//  if (offset <= 0.) {
//    length += offset;
//    delta_to_start_offset += offset;
//    offset = 0.;
//  }
//  if (length <= 0.) {
//    length = C2_CURVEPOINTS_LEASTDISTANCE * C2_CURVEPOINTS_POINTSMAX;
//  }
////    if (edge_it == edges->begin())
////      TRACE("Kein weiteres Backsampling möglich da die aktuelle Missionskante keine Vorgänger hat.");
//
//  double delta = length / (point_anz-1);
////  if (dist_to_step >= 0.) delta = ( length - gap_length ) / (point_anz);
//
//  // Schauen ob man im letzen Samplingdurchgang die Kante gesampelt hat,
//  // um gegebenenfalls das Offset so anzugelichen, damit die Samplingpunkte
//  // auf die gleichen Punkte fallen
//  /*
//    if (lastOffsets.find(edge) != lastOffsets.end()) {
//    TRACE("Kante wurde im letzen Durchgang schonmal gesampelt. Berechne Offset Korrektur");
//    double old_offset = lastOffsets[edge];
//    double diff = offset - old_offset ;
//    TRACE("Offset: "<< offset <<"  LastOffset: "<< old_offset <<"  Delta: "<<delta <<"  Differenz: "<< diff <<"  Korrektur:"<< diff/delta - static_cast<int>(diff/delta));
//    offset -= diff/delta - static_cast<int>(diff/delta);
//    TRACE(" -> Offset = "<<offset);
//    if (offset < 0) {
//    offset += delta;
//    TRACE(" -> Offset = "<<offset<<" (Vorzeichen-Korrektur)");
//    }
//    } else {
//    TRACE("Kante wurde im letzen Durchgang nicht gesampelt.");
//    }
//   */
//  assert(offset >= -0.01);
//  //  lastOffsets.clear();
//
//
//  // Neuen Zwischenpunkt berechnen
//  bool mission_end = false;
//  bool stepped = false;//dist_from_start + gap_length > delta_to_start_offset;
////  bool passed_lane_change_edge = false;
//  RndfEdge* change_edge = NULL;
//  double sampled_length = 0.;
//  RndfEdge* lastEdge = NULL;
//  for (int i=0; i<point_anz && !mission_end; ++i)
//  {
//    // Nächste Edge holen wenn Offset die Länge überschreitet
//    while (offset > edge->getLength()) {
//      offset -= edge->getLength();
//
//      // Nächste Kante ermitteln
//      ++edge_it;
//
//      // Missionsende prüfen
//      if (edge_it == edges->end()) {
//        TRACE("Keine weiteren Kante in der Route. Akt Offset = "<< offset);
//        mission_end = true;
//
//        if (curve->numberValidPoints % 2 == 1) curve->numberValidPoints--;
//
//        RndfVertex* ev =  edge->toVertex();
//        curve->curvepoints[ curve->numberValidPoints ].x = ev->x();
//        curve->curvepoints[ curve->numberValidPoints ].y = ev->y();
//        curve->curvepoints[ curve->numberValidPoints ].theta = edge->getAngle(); // curvesmother will set this
//        curve->curvepoints[ curve->numberValidPoints ].kappa = 0; // curvesmother will set this
//        break;
//      }
//
//      lastEdge = edge;
//      edge = (*edge_it)->getEdge();
//      assert(edge);
//      if ( ! edge ) {
//        TRACE("Fehlende RndfKante. Akt Offset = "<< offset)
//        mission_end = true;
//        break;
//      }
//
//      assert( lastEdge == NULL || lastEdge->toVertex()->hasOutEdge( edge ) );
//      if ( ! (lastEdge == NULL || lastEdge->toVertex()->hasOutEdge( edge ) ) ) {
//        TRACE("Routen Edges sind nicht direkt verbunden!!");
//        TRACE("Fehlende Verbindung von "<< lastEdge->name() << " nach "<< edge->name());
//        mission_end = true;
//        break;
//      }
//
//      //      if (lastOffsets.find(edge) == lastOffsets.end()) lastOffsets[edge] = offset;
//    }
//    if (mission_end) break;
//    assert(offset >= -0.01);
//
//
//    // Vertexes holen
//    RndfVertex* sv =  edge->fromVertex();
//    RndfVertex* ev =  edge->toVertex();
//
//    assert(sv && ev);
//    if (!sv || !ev) {
//      TRACE("Fehlende Vertexes zur Edge.");
//      break;
//    }
//
//    double x_norm = -( ev->y() - sv->y() );
//    double y_norm =  ( ev->x() - sv->x() );
//
//    double norm = sqrt( x_norm * x_norm + y_norm * y_norm );
//    x_norm /= norm;
//    y_norm /= norm;
//
//    // Zwischenpunkt berechnen
//    double w = offset / edge->getLength();
//    double x = sv->x() * (1-w) + ev->x() * (w);
//    double y = sv->y() * (1-w) + ev->y() * (w);
//
//    // lateral offset berechen
//    double lateral_offset = start_lateral_offset;
//    if (make_step) {
//      double act_dist_to_step = dist_to_step - (sampled_length + delta_to_start_offset);
//      if (fabs( act_dist_to_step ) < 0.01 && ! stepped) {
//        offset         += act_dist_to_step + gap_length - delta;
//        sampled_length += act_dist_to_step + gap_length - delta;
//        assert(offset >= 0.);
//        change_edge = edge;
//        stepped = true;
//      } else if (act_dist_to_step >= 0. && act_dist_to_step < 2*delta) {
//        offset         += act_dist_to_step - delta;
//        sampled_length += act_dist_to_step - delta;
//        assert(offset >= -delta -0.01);
//      } else if ( act_dist_to_step <= -gap_length+0.1 && stepped ) { // && (! passed_lane_change_edge || old_edge->isLaneChangeEdge()) ) {
//        lateral_offset = end_lateral_offset;
//        // stepped = false;
//      }
//    }
//
//    x +=  lateral_offset * x_norm;
//    y +=  lateral_offset * y_norm;
//
//    // Curvepoint setzen
//    curve->curvepoints[ curve->numberValidPoints ].x = x;
//    curve->curvepoints[ curve->numberValidPoints ].y = y;
//    curve->curvepoints[ curve->numberValidPoints ].theta = edge->getAngle(); // curvesmother will set this
//    curve->curvepoints[ curve->numberValidPoints ].kappa = 0; // curvesmother will set this
//    ++curve->numberValidPoints;
//
//    // Laneoffset merken
//    //    if (lastOffsets.find(edge) == lastOffsets.end()) lastOffsets[edge] = offset;
//
////    if (old_edge != edge) edge = old_edge;
//    sampled_length += delta;
//    offset         += delta;
//  }
//
//  using namespace CGAL_Geometry;
//  for (int i=0; i < curve->numberValidPoints; ++i) {
//    assert(!isdegenerated(curve->curvepoints[i].x));
//    assert(!isdegenerated(curve->curvepoints[i].y));
//    assert(!isdegenerated(curve->curvepoints[i].theta));
//    assert(!isdegenerated(curve->curvepoints[i].kappa));
//  }
//  double lx = curve->curvepoints[0].x;
//  double ly = curve->curvepoints[0].y;
//  for (int i=1; i < curve->numberValidPoints; ++i) {
//    assert(curve->curvepoints[i].x != lx);
//    assert(curve->curvepoints[i].y != ly);
//    lx = curve->curvepoints[i].x;
//    ly = curve->curvepoints[i].y;
//  }
//
//  return true;
//}

void RouteSampler::vertexToCurvePoint(RndfVertex* v) {
  if(!v) {return;}
  cp_.x = v->x();
  cp_.y = v->y();
}

bool RouteSampler::sampleMission(std::vector<CurvePoint>& waypoints, std::vector<bool>& points_to_ignore) {
  if (!edges_) {
    TRACE("Got zero pointer to edge list.");
    return false;
  }

  if (edges_->begin() == edges_->end()) {
    TRACE("empty mission");
    return false;
  }

  waypoints.clear();
  Route::RouteEdgeList::const_iterator eit = edges_->begin();
  for(; eit != edges_->end(); eit++) {
       vertexToCurvePoint((*eit)->getEdge()->fromVertex());
        waypoints.push_back(cp_);
        points_to_ignore.push_back((*eit)->getEdge()->isVirtualEdge());
     }
  vertexToCurvePoint( (*(--edges_->end()))->getEdge()->toVertex());
  waypoints.push_back(cp_);
//  points_to_ignore.push_back((*eit)->getEdge()->fromVertex()->isVirtualVertex());
  return true;
}
} // namespace vlr
