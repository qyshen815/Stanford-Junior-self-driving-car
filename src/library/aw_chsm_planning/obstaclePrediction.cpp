//#include <vector>
//#include <trajectory_points_interface.h>

#include <aw_Topology.hpp>
#include <aw_ChsmPlanner.hpp>
#include <intersectionPath.hpp>
#include <obstaclePrediction.h>

using namespace std;

namespace vlr {
#ifdef HAVE_PROBT
ObstaclePredictor::ObstaclePredictor(IntersectionManager* intersection_manager, Topology* topology, VehicleManager* vehicle_manager,
    pthread_mutex_t& intersection_predictor_mutex) :
  intersection_predictor_mutex_(intersection_predictor_mutex) {
  intersection_manager_ = intersection_manager;
  topology_ = topology;
  vehicle_manager_ = vehicle_manager;
  intersection_ = intersection_manager->getIntersection();
  setIntersectionVariables();
}

void ObstaclePredictor::predict() {
  //		bool all_way_stop_intersection = isAllWayStopIntersection();
  //		if (all_way_stop_intersection) {
  //			std::cout << "Next intersection is an all-way-stop intersection\n";
  //		}
  //		else {
  //			std::cout << "Next intersection is NOT an all-way-stop intersection\n";
  //		}
}

//	bool isAllWayStopIntersection() {
//		return 1;
//	}

void ObstaclePredictor::setIntersectionVariables() {

  const TRndfEdgeSet& intersection_edges = intersection_->getEdges();
  //TRndfEdgeSet intersection_edges = intersection->getEdges();
  //for (TRndfEdgeSet::iterator it = intersection_edges.begin(); it != intersection_edges.end();) {
  //cout << "edge " << (*it)->name() << endl;
  //TRndfEdgeSet::iterator erase_element = it++;
  //if ((*erase_element)->isLaneChangeEdge()) {
  //cout << "erased edge " << (*erase_element)->name() << endl;
  //intersection_edges.erase(erase_element++);
  //}
  //}

  // Find entrance and exit vertices, and find out if the intersection is all-way-stop

  is_all_way_stop_intersection_ = true;

  for (TRndfEdgeSet::const_iterator edge_it = intersection_edges.begin(); edge_it != intersection_edges.end(); ++edge_it) {

    //if (!(*edge_it)->isLaneChangeEdge()) {

    RndfVertex* connectedVertex;

    // Find entrance vertices
    if ((*edge_it)->isVirtualEdge()) {
      findEntranceVerticesOfVirtualEdge(*edge_it, &entrance_vertices_, true);
    }
    else {
      connectedVertex = (*edge_it)->fromVertex();
      entrance_vertices_.insert(connectedVertex);
      // Check if the intersection is all-way-stop
      if (!connectedVertex->isStopVertex()) {
        is_all_way_stop_intersection_ = false;
      }
    }

    // Find exit vertices
    if ((*edge_it)->isVirtualEdge()) {
      findExitVerticesOfVirtualEdge(*edge_it, &exit_vertices_);
    }
    else {
      connectedVertex = (*edge_it)->toVertex();
      exit_vertices_.insert(connectedVertex);
    }

    //}

  }

  // Set segment IDs
  nr_possible_segments_ = 0;
  nr_possible_entrance_segments_ = 0;
  createSegmentsIds();

  // Store angles between entrance lanes and exit lanes
  for (TRndfVertexSet::const_iterator entrance_vertex = entrance_vertices_.begin(); entrance_vertex != entrance_vertices_.end(); ++entrance_vertex) {

    double extreme_angle_left = 0.0;
    double extreme_angle_right = 0.0;
    RndfVertex* extreme_vertex_left = (*exit_vertices_.begin());
    RndfVertex* extreme_vertex_right = (*exit_vertices_.begin());

    for (TRndfVertexSet::const_iterator exit_vertex = exit_vertices_.begin(); exit_vertex != exit_vertices_.end(); ++exit_vertex) {

      VertexVertex entrance_exit;
      entrance_exit.entrance_vertex_ = *entrance_vertex;
      entrance_exit.exit_vertex_ = *exit_vertex;
      AngleExtreme* angle_extreme = new AngleExtreme;
      angle_extreme->angle_ = angleBetweenEdges(*((*exit_vertex)->getOutEdges().begin()), *((*entrance_vertex)->getInEdges().begin()));
      ;
      angle_extreme->is_extreme_left_ = false;
      angle_extreme->is_extreme_right_ = false;
      entrance_exit_angle_map_.insert(VertexVertexAngleMap::value_type(entrance_exit, *angle_extreme));

      if ((vertex_segment_map_[*entrance_vertex] != vertex_segment_map_[*exit_vertex]) && (angle_extreme->angle_ > extreme_angle_left)) {
        extreme_angle_left = angle_extreme->angle_;
        extreme_vertex_left = *exit_vertex;
      }

      if ((vertex_segment_map_[*entrance_vertex] != vertex_segment_map_[*exit_vertex]) && (angle_extreme->angle_ < extreme_angle_right)) {
        extreme_angle_right = angle_extreme->angle_;
        extreme_vertex_right = *exit_vertex;
      }

    }

    VertexVertex entrance_exit;
    entrance_exit.entrance_vertex_ = *entrance_vertex;

    entrance_exit.exit_vertex_ = extreme_vertex_left;
    entrance_exit_angle_map_[entrance_exit].is_extreme_left_ = true;
//!    cout << "Entrance vertex " << entrance_exit.entrance_vertex_->name() << " has extreme left exit vertex " << entrance_exit.exit_vertex_->name() << endl;

    entrance_exit.exit_vertex_ = extreme_vertex_right;
    entrance_exit_angle_map_[entrance_exit].is_extreme_right_ = true;
    //!    cout << "Entrance vertex " << entrance_exit.entrance_vertex_->name() << " has extreme right exit vertex " << entrance_exit.exit_vertex_->name() << endl;

  }

  // Display results
//!  for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {
//    cout << "Entrance vertex " << (*vertex_it)->name() << endl;
//  }
//  for (TRndfVertexSet::const_iterator vertex_it = exit_vertices_.begin(); vertex_it != exit_vertices_.end(); ++vertex_it) {
//    cout << "Exit vertex " << (*vertex_it)->name() << endl;
//  }
//  for (TRndfEdgeSet::const_iterator edge_it = intersection_edges.begin(); edge_it != intersection_edges.end(); ++edge_it) {
//    cout << "Edge " << (*edge_it)->name() << endl;
//  }
//  if (is_all_way_stop_intersection_) {
//    cout << "Next intersection is an all-way stop intersection with " << entrance_vertices_.size() << " entrance lanes and " << exit_vertices_.size()
//        << " exit lanes" << endl;
//  }
//  else {
//    cout << "Next intersection is NOT an all-way stop intersection" << endl;
//  }

  if (is_all_way_stop_intersection_) {

    // Find all possible paths in intersection
    PathSet path_set;
    for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {
      IntersectionPath* path = new IntersectionPath(*vertex_it, NULL);
      path_set.insert(path);
      createPath(path, &path_set, exit_vertices_);
    }
    for (PathSet::const_iterator path_it = path_set.begin(); path_it != path_set.end(); ++path_it) {
      VertexVertex entrance_vertex_exit_vertex;
      entrance_vertex_exit_vertex.entrance_vertex_ = (*path_it)->entrance_vertex_;
      entrance_vertex_exit_vertex.exit_vertex_ = (*path_it)->exit_vertex_;
      intersection_paths_.insert(VertexVertexPathMap::value_type(entrance_vertex_exit_vertex, *path_it));
    }

//!    // Display results
//    cout << "Nr paths: " << intersection_paths_.size() << endl;
//    for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths_.begin(); path_map_it != intersection_paths_.end(); ++path_map_it) {
//      cout << "Path: " << endl;
//      cout << "Entrance vertex " << path_map_it->second->entrance_vertex_->name() << endl;
//      cout << "Exit vertex " << path_map_it->second->exit_vertex_->name() << endl;
//    }

    // Mapping between exit vertices and entrance vertices
    findVerticesLeadingToExitVertices();
    findReachableVerticesFromEntranceVertex();

    //////////////////////
    // Bayesian Network //
    //////////////////////

    nr_possible_entrance_lanes_ = entrance_vertices_.size();
    nr_possible_exit_lanes_ = exit_vertices_.size();
    nr_possible_paths_ = intersection_paths_.size();
    nr_possible_turn_signals_ = 3;
    nr_possible_confusion_states_ = 2;

    // HERECHANGEPROBAS These probabilities should be learned, we set them manually for now
    proba_wrong_lane_ = 0.05;
    proba_wrong_segment_ = 0.005;
    //proba_max_wrong_path_ = proba_wrong_lane_ + proba_wrong_segment_;
    //proba_choose_incorrect_turn_signal_consistent_ = 0.05;
    //proba_choose_no_turn_signal_consistent_ = 1.0 - proba_choose_incorrect_turn_signal_consistent_;
    //proba_choose_incorrect_turn_signal_inconsistent_ = 0.02;
    //proba_choose_no_turn_signal_inconsistent_ = 1.0 - proba_choose_incorrect_turn_signal_inconsistent_;
    //proba_choose_incorrect_turn_signal_int_ = proba_choose_incorrect_turn_signal_consistent_;
    //proba_choose_no_turn_signal_int_ = proba_choose_no_turn_signal_consistent_;

    proba_min_choose_correct_turn_signal_ = 0.01;
    proba_choose_incorrect_turn_signal_extreme_ = 0.005;

    proba_max_choose_correct_turn_signal_multiple_consistent_extreme_ = 0.6;
    proba_max_choose_correct_turn_signal_unique_consistent_extreme_ = 0.4;

    proba_choose_correct_turn_signal_consistent_middle_ = 0.005;
    proba_choose_incorrect_turn_signal_consistent_middle_ = 0.005;

    proba_choose_correct_turn_signal_inconsistent_ = 0.8;
    proba_choose_incorrect_turn_signal_inconsistent_ = 0.005;

    //proba_choose_correct_turn_signal_inconsistent_lane_ = 0.01;
    //proba_choose_incorrect_turn_signal_inconsistent_lane_ = 0.01;

    proba_max_choose_correct_turn_signal_int_ = 0.4;
    proba_choose_incorrect_turn_signal_int_ = 0.01;

    // Mapping between indexes, paths, vertices and turn signals
    uint32_t index_exit = 0;
    for (TRndfVertexSet::const_iterator vertex_it = exit_vertices_.begin(); vertex_it != exit_vertices_.end(); ++vertex_it) {
      index_exit_map_[index_exit] = *vertex_it;
      exit_index_map_[*vertex_it] = index_exit;
      index_exit++;
    }
    uint32_t index_entrance = 0;
    for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {
      index_entrance_map_[index_entrance] = *vertex_it;
      entrance_index_map_[*vertex_it] = index_entrance;
      index_entrance++;
    }
    uint32_t index_path = 0;
    for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths_.begin(); path_map_it != intersection_paths_.end(); ++path_map_it) {
      index_path_map_[index_path] = path_map_it->first;
      path_index_map_[path_map_it->first] = index_path;
      index_path++;
      //cout << "entrance " << path_map_it->first.entrance_vertex_->name() << ", exit " << path_map_it->first.exit_vertex_->name() << endl;
    }

    ///////////////////////////////////////////////
    // Bayesian Network: variables specification //
    ///////////////////////////////////////////////

    plIntegerType entrance_type(0, nr_possible_entrance_lanes_ - 1);
    bn_entrance_ = plSymbol("Entrance_lane", entrance_type);

    plIntegerType segment_type(0, nr_possible_entrance_segments_ - 1);
    bn_segment_ = plSymbol("Entrance_segment", segment_type);

    plIntegerType exit_type(0, nr_possible_exit_lanes_ - 1);
    bn_exit_ = plSymbol("Exit_lane", exit_type);

    plIntegerType path_type(0, nr_possible_paths_ - 1);
    bn_path_ = plSymbol("Path", path_type);

    plIntegerType turn_signal_type(0, nr_possible_turn_signals_ - 1); // 0=none, 1=left, 2=right
    bn_turn_signal_ = plSymbol("Turn_signal", turn_signal_type);

    bn_turn_signal_int_ = plSymbol("Turn_signal_int", turn_signal_type);

    bn_wrong_lane_ = plSymbol("Wrong_lane", PL_BINARY_TYPE);
    bn_wrong_segment_ = plSymbol("Wrong_segment", PL_BINARY_TYPE);

    bn_coherence_entrance_ = plSymbol("Coherence_entrance", PL_BINARY_TYPE);
    bn_coherence_turn_signal_ = plSymbol("Coherence_turn_signal", PL_BINARY_TYPE);
    bn_coherence_turn_signal_int_ = plSymbol("Coherence_turn_signal_int", PL_BINARY_TYPE);
    bn_coherence_path_ = plSymbol("Coherence_path", PL_BINARY_TYPE);

    bn_soft_evidence_entrance_ = plSymbol("Soft_evidence_entrance", entrance_type);
    bn_soft_evidence_turn_signal_ = plSymbol("Soft_evidence_turn_signal", turn_signal_type);
    bn_soft_evidence_turn_signal_int_ = plSymbol("Soft_evidence_turn_signal_int", turn_signal_type);
    bn_soft_evidence_path_ = plSymbol("Soft_evidence_path", path_type);

    /////////////////////////////////////////////////////
    // Bayesian Network: parametric form specification //
    /////////////////////////////////////////////////////

    // Specification of P(exit)
    plUniform* P_exit = new plUniform(bn_exit_);
    P_exit_ = *P_exit;

    // Specification of P(wrong_lane)
    plProbValue table_wrong_lane[] = { 1.0 - proba_wrong_lane_, proba_wrong_lane_ };
    plProbTable* P_wrong_lane = new plProbTable(bn_wrong_lane_, table_wrong_lane);
    P_wrong_lane_ = *P_wrong_lane;

    // Specification of P(wrong_segment)
    plProbValue table_wrong_segment[] = { 1.0 - proba_wrong_segment_, proba_wrong_segment_ };
    plProbTable* P_wrong_segment = new plProbTable(bn_wrong_segment_, table_wrong_segment);
    P_wrong_segment_ = *P_wrong_segment;

    // Specification of P(segment)
    plProbValue table_segment_knowing_wrong_segment_exit[nr_possible_entrance_segments_ * nr_possible_confusion_states_ * nr_possible_exit_lanes_];

    for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {

      RndfVertex* exit_vertex = index_exit_map_[ex];
      uint32_t exit_segment = vertex_segment_map_[exit_vertex];

      for (uint32_t s = 0; s < nr_possible_entrance_segments_; s++) {

        // Find how many entrance segments take you to the segment that the exit is part of
        uint32_t nr_entrance_segments_leading_to_exit_segment = 0;
        for (uint32_t seg = 0; seg < nr_possible_entrance_segments_; seg++) {
          bool found = false;
          TRndfVertexSet* segment_vertices = segment_vertices_map_[seg];
          for (TRndfVertexSet::const_iterator segment_vertex_it = segment_vertices->begin(); segment_vertex_it != segment_vertices->end(); ++segment_vertex_it) {
            if (entrance_vertices_.find(*segment_vertex_it) != entrance_vertices_.end()) {
              for (TRndfVertexSet::const_iterator exit_vertex_it = entrance_vertex_exit_vertices_map_[*segment_vertex_it]->begin(); exit_vertex_it
                  != entrance_vertex_exit_vertices_map_[*segment_vertex_it]->end(); ++exit_vertex_it) {
                if (vertex_segment_map_[*exit_vertex_it] == exit_segment) {
                  found = true;
                }
              }
            }
          }
          if (found) {
            nr_entrance_segments_leading_to_exit_segment++;
          }
        }
        uint32_t nr_entrance_segments_not_leading_to_exit_segment = nr_possible_entrance_segments_ - nr_entrance_segments_leading_to_exit_segment;
        cout << nr_entrance_segments_leading_to_exit_segment << " segments take you to segment with exit " << exit_vertex->name() << ", and "
            << nr_entrance_segments_not_leading_to_exit_segment << " don't" << endl;

        // Does this entrance segment take us to this exit segment?
        bool segment_leads_to_exit = false;
        TRndfVertexSet* segment_vertices = segment_vertices_map_[s];
        for (TRndfVertexSet::const_iterator segment_vertex_it = segment_vertices->begin(); segment_vertex_it != segment_vertices->end(); ++segment_vertex_it) {
          if (entrance_vertices_.find(*segment_vertex_it) != entrance_vertices_.end()) {
            for (TRndfVertexSet::const_iterator exit_vertex_it = entrance_vertex_exit_vertices_map_[*segment_vertex_it]->begin(); exit_vertex_it
                != entrance_vertex_exit_vertices_map_[*segment_vertex_it]->end(); ++exit_vertex_it) {
              if (vertex_segment_map_[*exit_vertex_it] == exit_segment) {
                segment_leads_to_exit = true;
                //cout << "segment with entrance vertex " << (*entrance_vertex_it)->name() << " takes you to exit " << exit_vertex->name() << endl;
              }
            }
          }
          //if (exit_vertex_entrance_vertices_map_[exit_vertex]->find(*entrance_vertex_it) != exit_vertex_entrance_vertices_map_[exit_vertex]->end()) {
          //segment_leads_to_exit = true;
          ////cout << "segment with entrance vertex " << (*entrance_vertex_it)->name() << " takes you to exit " << exit_vertex->name() << endl;
          //}
        }

        uint32_t ws = 0;
        // If this segment takes you to this exit
        if (segment_leads_to_exit) {
          // When not wrong_segment
          ws = 0;
          table_segment_knowing_wrong_segment_exit[subToIndex(s, nr_possible_entrance_segments_, ws, ex, nr_possible_exit_lanes_)] = 1.0
              / nr_entrance_segments_leading_to_exit_segment;
          // When wrong_segment
          ws = 1;
          table_segment_knowing_wrong_segment_exit[subToIndex(s, nr_possible_entrance_segments_, ws, ex, nr_possible_exit_lanes_)] = 0.0;
        }
        else {
          // When not wrong_segment
          ws = 0;
          table_segment_knowing_wrong_segment_exit[subToIndex(s, nr_possible_entrance_segments_, ws, ex, nr_possible_exit_lanes_)] = 0.0;
          // When wrong_segment
          ws = 1;
          table_segment_knowing_wrong_segment_exit[subToIndex(s, nr_possible_entrance_segments_, ws, ex, nr_possible_exit_lanes_)] = 1.0
              / nr_entrance_segments_not_leading_to_exit_segment;
        }

      }

    }

    plDistributionTable* P_segment = new plDistributionTable(bn_segment_, bn_wrong_segment_ ^ bn_exit_, table_segment_knowing_wrong_segment_exit);
    P_segment_ = *P_segment;

    // Specification of P(entrance)
    plProbValue table_entrance_knowing_wrong_lane_exit_segment[nr_possible_entrance_lanes_ * nr_possible_confusion_states_ * nr_possible_exit_lanes_
        * nr_possible_entrance_segments_];

    for (uint32_t s = 0; s < nr_possible_entrance_segments_; s++) {

      for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {

        RndfVertex* exit_vertex = index_exit_map_[ex];

        // Find how many entrance lanes in the segment take you to the exit
        uint32_t nr_entrances_total = 0;
        uint32_t nr_entrances_leading_to_exit = 0;
        uint32_t nr_entrances_not_leading_to_exit;
        for (TRndfVertexSet::const_iterator seg_vertex_it = segment_vertices_map_[s]->begin(); seg_vertex_it != segment_vertices_map_[s]->end(); ++seg_vertex_it) {
          if (entrance_vertices_.find(*seg_vertex_it) != entrance_vertices_.end()) {
            nr_entrances_total++;
            if (exit_vertex_entrance_vertices_map_[exit_vertex]->find(*seg_vertex_it) != exit_vertex_entrance_vertices_map_[exit_vertex]->end()) {
              nr_entrances_leading_to_exit++;
            }
          }
        }
        nr_entrances_not_leading_to_exit = nr_entrances_total - nr_entrances_leading_to_exit;
        //cout << "segment with vertex " << (*(segment_vertices_map_[s]->begin()))->name() << ", exit " << exit_vertex->name() << ", nr_entrances_total " << nr_entrances_total << ", nr_entrances_leading_to_exit " << nr_entrances_leading_to_exit << endl;

        for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {

          RndfVertex* entrance_vertex = index_entrance_map_[en];

          // Proba is 0 for entrance lanes in other segments
          if (vertex_segment_map_[entrance_vertex] != s) {
            for (uint32_t wl = 0; wl < nr_possible_confusion_states_; wl++) {
              table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s,
                  nr_possible_entrance_segments_)] = 0.0;
              //cout << "Zero proba for entrance " << entrance_vertex->name() << " when wrong_lane is " << j << ", exit is " << index_exit_map_[k]->name() << " and segment is " << l << endl;
            }
          }

          // Lanes in the segment
          else {

            uint32_t wl = 0;

            // Find how many entrance lanes in the segment take you to the exit
            //uint32_t nr_entrances_leading_to_exit = 0;
            //for (TRndfVertexSet::const_iterator entrance_vertex_it = exit_vertex_entrance_vertices_map_[exit_vertex]->begin(); entrance_vertex_it != exit_vertex_entrance_vertices_map_[exit_vertex]->end(); ++entrance_vertex_it) {
            //if (vertex_segment_map_[*entrance_vertex_it] == s) {
            //nr_entrances_leading_to_exit++;
            //}
            //}

            // If this entrance takes you to this exit
            if (exit_vertex_entrance_vertices_map_[exit_vertex]->find(entrance_vertex) != exit_vertex_entrance_vertices_map_[exit_vertex]->end()) {
              // When not wrong_lane
              wl = 0;
              table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s,
                  nr_possible_entrance_segments_)] = 1.0 / nr_entrances_leading_to_exit;
              //cout << "proba=" << table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s, nr_possible_entrance_segments_)] << endl;
              // When wrong_lane
              wl = 1;
              table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s,
                  nr_possible_entrance_segments_)] = 0.0;
              //cout << "nr_entrances_leading_to_exit=" << nr_entrances_leading_to_exit << endl;
              //cout << "Zero proba for entrance " << entrance_vertex->name() << " when wrong_lane is " << 1 << ", exit is " << exit_vertex->name() << " and segment is " << l << endl;
            }
            else {
              // When not wrong_lane
              wl = 0;
              table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s,
                  nr_possible_entrance_segments_)] = 0.0;
              // When wrong_lane
              wl = 1;
              table_entrance_knowing_wrong_lane_exit_segment[subToIndex(en, nr_possible_entrance_lanes_, wl, ex, nr_possible_exit_lanes_, s,
                  nr_possible_entrance_segments_)] = 1.0 / nr_entrances_not_leading_to_exit;
            }

          }
        }
      }
    }

    plDistributionTable* P_entrance = new plDistributionTable(bn_entrance_, bn_wrong_lane_ ^ bn_exit_ ^ bn_segment_,
        table_entrance_knowing_wrong_lane_exit_segment);
    P_entrance_ = *P_entrance;

    // Specification of P(path) and P(turn_signal_int)
    // depend on the vehicle state, defined later

    // Specification of P(turn_signal)

    plProbValue table_turn_signal_knowing_entrance_exit[nr_possible_turn_signals_ * nr_possible_entrance_lanes_ * nr_possible_exit_lanes_];

    for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {

      RndfVertex* entrance_vertex = index_entrance_map_[en];

      for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {

        RndfVertex* exit_vertex = index_exit_map_[ex];

        uint32_t correct_turn_signal = 1;
        uint32_t incorrect_turn_signal = 2;
        uint32_t no_turn_signal = 0;

        VertexVertex entrance_vertex_exit_vertex;
        entrance_vertex_exit_vertex.entrance_vertex_ = entrance_vertex;
        entrance_vertex_exit_vertex.exit_vertex_ = exit_vertex;

        // When entrance and exit are inconsistent (exit cannot be reached from entrance)
        if (exit_vertex_entrance_vertices_map_[exit_vertex]->find(entrance_vertex) == exit_vertex_entrance_vertices_map_[exit_vertex]->end()) {

          // Find if exit is reachable from another vertex in the segment
          bool segment_is_inconsistent = true;
          //bool dummy_same_segment = false;
          //TRndfEdgeSet dummy_checked_edges;
          uint32_t side_other_vertex = 0;
          TRndfVertexSet* other_vertices_in_segment = segment_vertices_map_[vertex_segment_map_[entrance_vertex]];
          for (TRndfVertexSet::const_iterator vertex_it = other_vertices_in_segment->begin(); vertex_it != other_vertices_in_segment->end(); ++vertex_it) {
            if (entrance_vertices_.find(*vertex_it) != entrance_vertices_.end()) {
              if (entrance_vertex_exit_vertices_map_[*vertex_it]->find(exit_vertex) != entrance_vertex_exit_vertices_map_[*vertex_it]->end()) {
                segment_is_inconsistent = false;
                VertexVertex vertex_vertex;
                vertex_vertex.entrance_vertex_ = entrance_vertex;
                vertex_vertex.exit_vertex_ = *vertex_it;
                side_other_vertex = side_vertices_in_same_segment_[vertex_vertex];
                //dummy_same_segment = areInSameSegment(*(entrance_vertex->getInEdges().begin()), *((*vertex_it)->getInEdges().begin()), &dummy_checked_edges, &side_other_vertex);
              }
            }
          }

          // If segment is inconsistent, determine correct turn signal from angle
          if (segment_is_inconsistent) {
            cout << "exit " << exit_vertex->name() << " cannot be reached from any entrance lane in the same segment as " << entrance_vertex->name() << endl;
            double angle = entrance_exit_angle_map_[entrance_vertex_exit_vertex].angle_;
            if (angle > 0.0) {
              correct_turn_signal = 1;
              incorrect_turn_signal = 2;
            }
            else {
              correct_turn_signal = 2;
              incorrect_turn_signal = 1;
            }
          }
          // If lane (and not segment) is inconsistent, determine correct turn angle from which lane would make exit reachable
          else {
            if (side_other_vertex == 1) {
              correct_turn_signal = 1;
              incorrect_turn_signal = 2;
            }
            else if (side_other_vertex == 2) {
              correct_turn_signal = 2;
              incorrect_turn_signal = 1;
            }
            else {
              cout << "Problem in sides" << endl;
            }
            cout << "entrance " << entrance_vertex->name() << " and exit " << exit_vertex->name()
                << " are inconsistent, but the exit can be reached from a lane " << correct_turn_signal << endl;
          }

          double proba_choose_correct_turn_signal = proba_choose_correct_turn_signal_inconsistent_;
          double proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_inconsistent_;

          table_turn_signal_knowing_entrance_exit[subToIndex(correct_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              = proba_choose_correct_turn_signal;
          table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              = proba_choose_incorrect_turn_signal;
          //if (table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] > proba_choose_correct_turn_signal) {
          //table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] = proba_choose_correct_turn_signal;
          //}
          table_turn_signal_knowing_entrance_exit[subToIndex(no_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] = 1.0
              - table_turn_signal_knowing_entrance_exit[subToIndex(correct_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              - table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)];
          cout << "inconsistent entrance " << entrance_vertex->name() << " exit " << exit_vertex->name() << " : none="
              << table_turn_signal_knowing_entrance_exit[subToIndex(0, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", left="
              << table_turn_signal_knowing_entrance_exit[subToIndex(1, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", right="
              << table_turn_signal_knowing_entrance_exit[subToIndex(2, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << endl;

        }

        // When entrance and exit are consistent
        else {

          //cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << endl;

          double angle = entrance_exit_angle_map_[entrance_vertex_exit_vertex].angle_;
          double ratio = abs(angle) / (0.5 * M_PI);
          if (ratio > 1.0) {
            ratio = 1.0;
          }

          bool is_extreme;
          if (angle > 0.0) {
            correct_turn_signal = 1;
            incorrect_turn_signal = 2;
            is_extreme = (vertex_segment_map_[entrance_vertex] == vertex_segment_map_[exit_vertex]
                || entrance_exit_angle_map_[entrance_vertex_exit_vertex].is_extreme_left_);
          }
          else {
            correct_turn_signal = 2;
            incorrect_turn_signal = 1;
            is_extreme = (vertex_segment_map_[entrance_vertex] == vertex_segment_map_[exit_vertex]
                || entrance_exit_angle_map_[entrance_vertex_exit_vertex].is_extreme_right_);
          }

          bool is_unique;
          if (entrance_vertex_exit_vertices_map_[entrance_vertex]->size() > 1) {
            is_unique = false;
          }
          else {
            is_unique = true;
          }

          double proba_choose_correct_turn_signal;
          double proba_choose_incorrect_turn_signal;

          if (is_extreme && is_unique) {
            proba_choose_correct_turn_signal = proba_min_choose_correct_turn_signal_ + (proba_max_choose_correct_turn_signal_unique_consistent_extreme_
                - proba_min_choose_correct_turn_signal_) * ratio;
            proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_extreme_;
          }
          else if (is_extreme && !is_unique) {
            proba_choose_correct_turn_signal = proba_min_choose_correct_turn_signal_ + (proba_max_choose_correct_turn_signal_multiple_consistent_extreme_
                - proba_min_choose_correct_turn_signal_) * ratio;
            proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_extreme_;
          }
          else if (!is_extreme && is_unique) {
            proba_choose_correct_turn_signal = proba_choose_correct_turn_signal_consistent_middle_;
            proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_consistent_middle_;
          }
          else {
            proba_choose_correct_turn_signal = proba_choose_correct_turn_signal_consistent_middle_;
            proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_consistent_middle_;
          }

          table_turn_signal_knowing_entrance_exit[subToIndex(correct_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              = proba_choose_correct_turn_signal;
          table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              = proba_choose_incorrect_turn_signal;
          //if (table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] > proba_choose_correct_turn_signal) {
          //table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] = proba_choose_correct_turn_signal;
          //}
          table_turn_signal_knowing_entrance_exit[subToIndex(no_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] = 1.0
              - table_turn_signal_knowing_entrance_exit[subToIndex(correct_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)]
              - table_turn_signal_knowing_entrance_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)];
          cout << "consistent entrance " << entrance_vertex->name() << " exit " << exit_vertex->name() << ", probas: none="
              << table_turn_signal_knowing_entrance_exit[subToIndex(0, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", left="
              << table_turn_signal_knowing_entrance_exit[subToIndex(1, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", right="
              << table_turn_signal_knowing_entrance_exit[subToIndex(2, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << endl;

          // Debug
          //if (en == 1 && ex == 4) {
          //cout << "Right turn: angle=" << abs(angle/M_PI) << ", proba none=" << table_turn_signal_knowing_entrance_exit[subToIndex(no_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", proba right=" << table_turn_signal_knowing_entrance_exit[subToIndex(2, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", proba left=" << table_turn_signal_knowing_entrance_exit[subToIndex(1, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << endl;
          //}
          //if (en == 2 && ex == 2) {
          //cout << "Go straight: angle=" << abs(angle/M_PI) << ", proba none=" << table_turn_signal_knowing_entrance_exit[subToIndex(no_turn_signal, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", proba left=" << table_turn_signal_knowing_entrance_exit[subToIndex(1, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << ", proba right=" << table_turn_signal_knowing_entrance_exit[subToIndex(2, nr_possible_turn_signals_, en, ex, nr_possible_exit_lanes_)] << endl;
          //}

        }

      }
    }

    plDistributionTable* P_turn_signal = new plDistributionTable(bn_turn_signal_, bn_entrance_ ^ bn_exit_, table_turn_signal_knowing_entrance_exit);
    P_turn_signal_ = *P_turn_signal;

    // Specification of P_coherence_entrance_

    plProbValue table_coherence_entrance_knowing_entrance_soft_evidence_entrance[2 * nr_possible_entrance_lanes_ * nr_possible_entrance_lanes_];

    for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {
      for (uint32_t s_en = 0; s_en < nr_possible_entrance_lanes_; s_en++) {
        uint32_t c_en;
        if (en == s_en) {
          c_en = 0;
          table_coherence_entrance_knowing_entrance_soft_evidence_entrance[subToIndex(c_en, 2, s_en, en, nr_possible_entrance_lanes_)] = 0.0;
          c_en = 1;
          table_coherence_entrance_knowing_entrance_soft_evidence_entrance[subToIndex(c_en, 2, s_en, en, nr_possible_entrance_lanes_)] = 1.0;
        }
        else {
          c_en = 0;
          table_coherence_entrance_knowing_entrance_soft_evidence_entrance[subToIndex(c_en, 2, s_en, en, nr_possible_entrance_lanes_)] = 1.0;
          c_en = 1;
          table_coherence_entrance_knowing_entrance_soft_evidence_entrance[subToIndex(c_en, 2, s_en, en, nr_possible_entrance_lanes_)] = 0.0;
        }
      }
    }

    plDistributionTable* P_coherence_entrance = new plDistributionTable(bn_coherence_entrance_, bn_entrance_ ^ bn_soft_evidence_entrance_,
        table_coherence_entrance_knowing_entrance_soft_evidence_entrance);
    P_coherence_entrance_ = *P_coherence_entrance;

    // Specification of P_coherence_turn_signal_

    plProbValue table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal[2 * nr_possible_turn_signals_ * nr_possible_turn_signals_];

    for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
      for (uint32_t s_t = 0; s_t < nr_possible_turn_signals_; s_t++) {
        uint32_t c_t;
        if (t == s_t) {
          c_t = 0;
          table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal[subToIndex(c_t, 2, s_t, t, nr_possible_turn_signals_)] = 0.0;
          c_t = 1;
          table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal[subToIndex(c_t, 2, s_t, t, nr_possible_turn_signals_)] = 1.0;
        }
        else {
          c_t = 0;
          table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal[subToIndex(c_t, 2, s_t, t, nr_possible_turn_signals_)] = 1.0;
          c_t = 1;
          table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal[subToIndex(c_t, 2, s_t, t, nr_possible_turn_signals_)] = 0.0;
        }
      }
    }

    plDistributionTable* P_coherence_turn_signal = new plDistributionTable(bn_coherence_turn_signal_, bn_turn_signal_ ^ bn_soft_evidence_turn_signal_,
        table_coherence_turn_signal_knowing_turn_signal_soft_evidence_turn_signal);
    P_coherence_turn_signal_ = *P_coherence_turn_signal;

    // Specification of P_coherence_path_

    plProbValue table_coherence_path_knowing_path_soft_evidence_path[2 * nr_possible_paths_ * nr_possible_paths_];

    for (uint32_t p = 0; p < nr_possible_paths_; p++) {
      for (uint32_t s_p = 0; s_p < nr_possible_paths_; s_p++) {
        uint32_t c_p;
        if (p == s_p) {
          c_p = 0;
          table_coherence_path_knowing_path_soft_evidence_path[subToIndex(c_p, 2, s_p, p, nr_possible_paths_)] = 0.0;
          c_p = 1;
          table_coherence_path_knowing_path_soft_evidence_path[subToIndex(c_p, 2, s_p, p, nr_possible_paths_)] = 1.0;
        }
        else {
          c_p = 0;
          table_coherence_path_knowing_path_soft_evidence_path[subToIndex(c_p, 2, s_p, p, nr_possible_paths_)] = 1.0;
          c_p = 1;
          table_coherence_path_knowing_path_soft_evidence_path[subToIndex(c_p, 2, s_p, p, nr_possible_paths_)] = 0.0;
        }
      }
    }

    plDistributionTable* P_coherence_path = new plDistributionTable(bn_coherence_path_, bn_path_ ^ bn_soft_evidence_path_,
        table_coherence_path_knowing_path_soft_evidence_path);
    P_coherence_path_ = *P_coherence_path;

    // Specification of P_coherence_turn_signal_int_

    plProbValue
        table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int[2 * nr_possible_turn_signals_ * nr_possible_turn_signals_];

    for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
      for (uint32_t s_t = 0; s_t < nr_possible_turn_signals_; s_t++) {
        uint32_t c_ti;
        if (t == s_t) {
          c_ti = 0;
          table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int[subToIndex(c_ti, 2, s_t, t, nr_possible_turn_signals_)] = 0.0;
          c_ti = 1;
          table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int[subToIndex(c_ti, 2, s_t, t, nr_possible_turn_signals_)] = 1.0;
        }
        else {
          c_ti = 0;
          table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int[subToIndex(c_ti, 2, s_t, t, nr_possible_turn_signals_)] = 1.0;
          c_ti = 1;
          table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int[subToIndex(c_ti, 2, s_t, t, nr_possible_turn_signals_)] = 0.0;
        }
      }
    }

    plDistributionTable* P_coherence_turn_signal_int = new plDistributionTable(bn_coherence_turn_signal_int_, bn_turn_signal_int_
        ^ bn_soft_evidence_turn_signal_int_, table_coherence_turn_signal_int_knowing_turn_signal_int_soft_evidence_turn_signal_int);
    P_coherence_turn_signal_int_ = *P_coherence_turn_signal_int;

  }

}

// Update relevant vehicles
void ObstaclePredictor::determineRelevantVehicles() {
  //cout << "size " << vehicle_manager_->vehicle_map.size() << endl;

  for (VehicleMap::iterator vehicle_map_it = vehicle_manager_->vehicle_map.begin(); vehicle_map_it != vehicle_manager_->vehicle_map.end(); ++vehicle_map_it) {

    //if (vehicle_map_it->first == 10 || vehicle_map_it->first == 11) {
    //cout << "Vehicle " << vehicle_map_it->first << " is matched to edge " << vehicle_map_it->second.edge()->name() << endl;
    //}

    if (vehicle_map_it->second.isOnIntersection(intersection_)) {
      relevant_vehicles_.insert(vehicle_map_it->first);
      relevant_vehicles_in_intersection_.insert(vehicle_map_it->first);
      cout << "Vehicle " << vehicle_map_it->first << " is in intersection" << endl;
      //veh_id_path_proba_map_old_ = veh_id_path_proba_map_;
      //veh_id_turn_signal_int_proba_map_old_ = veh_id_turn_signal_int_proba_map_;
    }
    else {
      bool is_heading_for_next_intersection = false;
      uint32_t count = 0;
      vehicleHeadingToNextIntersection(vehicle_map_it->second, vehicle_map_it->second.edge(), &is_heading_for_next_intersection, &count);
      if (is_heading_for_next_intersection) {
        relevant_vehicles_.insert(vehicle_map_it->first);
        relevant_vehicles_approaching_intersection_.insert(vehicle_map_it->first);
        cout << "Vehicle " << vehicle_map_it->first << " is approaching intersection" << endl;
        //veh_id_entrance_vertex_proba_map_old_ = veh_id_entrance_vertex_proba_map_;
        //veh_id_turn_signal_proba_map_old_ = veh_id_turn_signal_proba_map_;
      }
      else {
        ////non_relevant_vehicles_.insert(vehicle_map_it->first);
        //relevant_vehicles_.erase(vehicle_map_it->first);
        //relevant_vehicles_in_intersection_.erase(vehicle_map_it->first);
        //relevant_vehicles_approaching_intersection_.erase(vehicle_map_it->first);
        cout << "Vehicle " << vehicle_map_it->first << " is not relevant" << endl;
      }
    }
  }

  // Erase vehicles that are not relevant from class variables

  //cout << "FOUND=" << (relevant_vehicles_.find(0)!=relevant_vehicles_.end()) << endl;
  for (VehIdVertexProbaMap::iterator it = veh_id_entrance_vertex_proba_map_.begin(); it != veh_id_entrance_vertex_proba_map_.end();) {
    VehIdVertexProbaMap::iterator erase_element = it++;
    if (vehicle_manager_->vehicle_map.find(erase_element->first.veh_id_) == vehicle_manager_->vehicle_map.end()) {
      cout << "Erase stored information on vehicle " << erase_element->first.veh_id_ << endl;
      veh_id_entrance_vertex_proba_map_.erase(erase_element);
    }
  }
  for (VehIdPathProbaMap::iterator it = veh_id_path_proba_map_.begin(); it != veh_id_path_proba_map_.end();) {
    VehIdPathProbaMap::iterator erase_element = it++;
    if (vehicle_manager_->vehicle_map.find(erase_element->first.veh_id_) == vehicle_manager_->vehicle_map.end()) {
      veh_id_path_proba_map_.erase(erase_element);
    }
  }
  for (VehIdTurnSignalProbaMap::iterator it = veh_id_turn_signal_proba_map_.begin(); it != veh_id_turn_signal_proba_map_.end();) {
    VehIdTurnSignalProbaMap::iterator erase_element = it++;
    if (vehicle_manager_->vehicle_map.find(erase_element->first.veh_id_) == vehicle_manager_->vehicle_map.end()) {
      veh_id_turn_signal_proba_map_.erase(erase_element++);
    }
  }
  for (VehIdTurnSignalProbaMap::iterator it = veh_id_turn_signal_int_proba_map_.begin(); it != veh_id_turn_signal_int_proba_map_.end();) {
    VehIdTurnSignalProbaMap::iterator erase_element = it++;
    if (vehicle_manager_->vehicle_map.find(erase_element->first.veh_id_) == vehicle_manager_->vehicle_map.end()) {
      veh_id_turn_signal_int_proba_map_.erase(erase_element++);
    }
  }
  for (VehIdVertexProbaMap::iterator it = veh_id_exit_vertex_proba_map_.begin(); it != veh_id_exit_vertex_proba_map_.end();) {
    VehIdVertexProbaMap::iterator erase_element = it++;
    if (vehicle_manager_->vehicle_map.find(erase_element->first.veh_id_) == vehicle_manager_->vehicle_map.end()) {
      veh_id_exit_vertex_proba_map_.erase(erase_element++);
    }
  }
}
void ObstaclePredictor::updateVehiclesInIntersection() {
  /////////////////////////////
  // Vehicle in intersection //
  /////////////////////////////

  for (VehicleIdSet::iterator veh_id_it = relevant_vehicles_in_intersection_.begin(); veh_id_it != relevant_vehicles_in_intersection_.end(); ++veh_id_it) {

    PathEdgeProbaMap path_edge_proba_map;

    double smallest_dist = 0.0;
    double dist = 0.0;
    //double largest_proba = 0.0;
    double proba = 0.0;
    for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths_.begin(); path_map_it != intersection_paths_.end(); ++path_map_it) {
      // Calculate distance to each edge of the path and keep track of the closest edge
      IntersectionPath* path = path_map_it->second;
      struct EdgeProba edge_proba;
      edge_proba.edge_ = *(path_map_it->second->edges().begin());
      edge_proba.dist_ = vehicle_manager_->vehicle_map[*veh_id_it].distToEdge(edge_proba.edge_);
      edge_proba.proba_ = computeProbaVehicleOnEdge(&(vehicle_manager_->vehicle_map[*veh_id_it]), edge_proba.edge_);
      path_edge_proba_map.insert(PathEdgeProbaMap::value_type(path, edge_proba));
      smallest_dist = edge_proba.dist_;
      //largest_proba = edge_proba.proba_;
      for (TRndfEdgeSet::iterator edge_it = path_map_it->second->edges().begin(); edge_it != path_map_it->second->edges().end(); ++edge_it) {
        dist = vehicle_manager_->vehicle_map[*veh_id_it].distToEdge(*edge_it);
        proba = computeProbaVehicleOnEdge(&(vehicle_manager_->vehicle_map[*veh_id_it]), (*edge_it));
        if (dist < smallest_dist) {
          path_edge_proba_map[path].edge_ = *edge_it;
          path_edge_proba_map[path].proba_ = proba;
          smallest_dist = dist;
        }
        //if (proba > largest_proba) {
        //path_edge_proba_map[path].edge_ = *edge_it;
        //path_edge_proba_map[path].proba_ = proba;
        //largest_proba = proba;
        //}
      }
      //cout << "Before normalization, vehicle " << *veh_id_it << " is on path " << path_map_it->first << " with proba " << path_map_it->second.proba_ << endl;
    }

    // Normalize probabilities
    double sum_of_probas_paths = 0.0;
    for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
      sum_of_probas_paths += path_map_it->second.proba_;
    }
    if (sum_of_probas_paths > 0.0) {
      for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
        path_map_it->second.proba_ = path_map_it->second.proba_ / sum_of_probas_paths;
      }
    }

    // Class variable path
    for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
      VehIdPath veh_id_path;
      veh_id_path.veh_id_ = *veh_id_it;
      veh_id_path.path_ = path_map_it->first;
      veh_id_path_proba_map_[veh_id_path] = path_map_it->second.proba_;
      if (*veh_id_it == 8) {
        //HERECOUTcout << "Vehicle " << *veh_id_it << " is on path leading from vertex " << path_map_it->first->entrance_vertex_->name() << " to vertex " << path_map_it->first->exit_vertex_->name() << " with proba " << path_map_it->second.proba_ << endl;
      }
    }

    // Class variable turn signal int: for now, initialize manually
    TurnSignalProbaMap turn_signal_int_proba_map;
    turn_signal_int_proba_map[0] = 0.33334;
    turn_signal_int_proba_map[1] = 0.33333;
    turn_signal_int_proba_map[2] = 0.33333;
    //if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 0) {
    //turn_signal_int_proba_map[0] = 1.0;
    //turn_signal_int_proba_map[1] = 0.0;
    //turn_signal_int_proba_map[2] = 0.0;
    //}
    //else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 1) {
    //turn_signal_int_proba_map[0] = 0.0;
    //turn_signal_int_proba_map[1] = 1.0;
    //turn_signal_int_proba_map[2] = 0.0;
    //}
    //else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 2) {
    //turn_signal_int_proba_map[0] = 0.0;
    //turn_signal_int_proba_map[1] = 0.0;
    //turn_signal_int_proba_map[2] = 1.0;
    //}
    //else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 127) {
    //turn_signal_int_proba_map[0] = 0.33334;
    //turn_signal_int_proba_map[1] = 0.33333;
    //turn_signal_int_proba_map[2] = 0.33333;
    //}
    for (TurnSignalProbaMap::iterator turn_signal_int_map_it = turn_signal_int_proba_map.begin(); turn_signal_int_map_it != turn_signal_int_proba_map.end(); ++turn_signal_int_map_it) {
      VehIdTurnSignal veh_id_turn_signal_int;
      veh_id_turn_signal_int.veh_id_ = *veh_id_it;
      veh_id_turn_signal_int.turn_signal_ = turn_signal_int_map_it->first;
      veh_id_turn_signal_int_proba_map_[veh_id_turn_signal_int] = turn_signal_int_map_it->second;
    }

    // Specification of P(path)

    plProbValue table_path_knowing_exit[nr_possible_paths_ * nr_possible_entrance_lanes_ * nr_possible_exit_lanes_];
    //for (uint32_t i = 0; i < nr_possible_paths_ * nr_possible_entrance_lanes_* nr_possible_exit_lanes_; i++) {
    //table_path_knowing_exit[i] = 0.0;
    //}

    for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {
      RndfVertex* entrance_vertex = index_entrance_map_[en];
      double distance_to_entrance = distanceBetweenPositions(entrance_vertex->x(), entrance_vertex->y(),
          vehicle_manager_->vehicle_map[*veh_id_it].xMatchedFrom(), vehicle_manager_->vehicle_map[*veh_id_it].yMatchedFrom());

      for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {
        RndfVertex* exit_vertex = index_exit_map_[ex];
        double distance_to_exit = distanceBetweenPositions(exit_vertex->x(), exit_vertex->y(), vehicle_manager_->vehicle_map[*veh_id_it].xMatchedFrom(),
            vehicle_manager_->vehicle_map[*veh_id_it].yMatchedFrom());

        double distance_entrance_exit = distanceBetweenPositions(entrance_vertex->x(), entrance_vertex->y(), exit_vertex->x(), exit_vertex->y());
        //// Compute weights for correct entrance and correct exit
        //double weight_exit = 1.0;//distance_to_entrance / (distance_to_entrance + distance_to_exit);//attention, remettre à 1.0 ?
        ////double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), vehicle_manager_->vehicle_map[*veh_id_it].yawMatchedFrom());
        ////double weight_entrance = computeWeightEntrance(distance_to_entrance, distance_to_exit, angle, vehicle_manager_->vehicle_map[*veh_id_it].speed());
        ////cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << ", distance_to_entrance=" << distance_to_entrance << ", distance_to_exit=" << distance_to_exit << ", angle=" << angle << ", velocity=" << vehicle_manager_->vehicle_map[*veh_id_it].speed() << ", weight_entrance=" << weight_entrance << endl;

        //double sum_probas_path = 0.0;
        //for (uint32_t p = 0; p < nr_possible_paths_; p++) {
        //IntersectionPath* path = intersection_paths_[index_path_map_[p]];

        //double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), path_edge_proba_map[path].edge_->getAngle());

        //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] = 0.0;

        //// if the entrance of the path is the entrance
        //if (path->entrance_vertex_ == entrance_vertex) {
        //double weight_entrance = computeWeightEntrance(distance_to_entrance, distance_to_exit, angle, vehicle_manager_->vehicle_map[*veh_id_it].speed());
        //cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << ", distance_to_entrance=" << distance_to_entrance << ", distance_to_exit=" << distance_to_exit << ", angle=" << angle << ", velocity=" << vehicle_manager_->vehicle_map[*veh_id_it].speed() << ", weight_entrance=" << weight_entrance << endl;
        //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_entrance;
        //}

        //// if the exit of the path is the exit
        //if (path->exit_vertex_ == exit_vertex) {
        //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_exit;
        //}

        //sum_probas_path += table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)];

        //}

        // Compute weights for correct entrance and correct exit
        double weight_exit = gaussian_func(distance_to_exit, 0.0, distance_entrance_exit / 2.0);//1.0 - distance_to_exit / distance_entrance_exit;//distance_to_entrance / (distance_to_entrance + distance_to_exit);
        //if (weight_exit < 0.0) {
        //weight_exit = 0.0;
        //}
        //double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), vehicle_manager_->vehicle_map[*veh_id_it].yawMatchedFrom());
        double weight_entrance = gaussian_func(distance_to_entrance, 0.0, distance_entrance_exit / 2.0);//1.0 - distance_to_entrance / distance_entrance_exit;//distance_to_exit / (distance_to_entrance + distance_to_exit);
        //if (weight_entrance < 0.0) {
        //weight_entrance = 0.0;
        //}
        //cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << ", distance_to_entrance=" << distance_to_entrance << ", distance_to_exit=" << distance_to_exit << ", angle=" << angle << ", velocity=" << vehicle_manager_->vehicle_map[*veh_id_it].speed() << ", weight_entrance=" << weight_entrance << endl;

        double sum_probas_path = 0.0;
        for (uint32_t p = 0; p < nr_possible_paths_; p++) {
          IntersectionPath* path = intersection_paths_[index_path_map_[p]];

          table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] = 0.0;

          double dist_entrance = distanceBetweenPositions(path->entrance_vertex_->x(), path->entrance_vertex_->y(),
              vehicle_manager_->vehicle_map[*veh_id_it].xMatchedFrom(), vehicle_manager_->vehicle_map[*veh_id_it].yMatchedFrom());
          double dist_exit = distanceBetweenPositions(path->exit_vertex_->x(), path->exit_vertex_->y(),
              vehicle_manager_->vehicle_map[*veh_id_it].xMatchedFrom(), vehicle_manager_->vehicle_map[*veh_id_it].yMatchedFrom());
          double dist_entrance_exit = distanceBetweenPositions(path->entrance_vertex_->x(), path->entrance_vertex_->y(), path->exit_vertex_->x(),
              path->exit_vertex_->y());

          //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] = 0.0;

          //// if the entrance of the path is the entrance
          //if (path->entrance_vertex_ == entrance_vertex) {
          //double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), path_edge_proba_map[path].edge_->getAngle());
          //double sigma_angle = M_PI / 6.0;
          //double weight_angle = gaussian_func(angle, 0.0, sigma_angle);
          //double weight_path;
          //if (path->exit_vertex_ == exit_vertex) {
          //weight_path = 1.0;
          //}
          //else {
          //weight_path = dist_exit / (dist_entrance + dist_exit);
          ////if (weight_path > 1.0) {
          ////weight_path = 1.0;
          ////}
          //}
          //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_entrance * weight_path;// * weight_angle;
          //}
          ////cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << ", entrance path " << path->entrance_vertex_->name() << ", exit path, " << path->exit_vertex_->name() << ", weight_entrance=" << table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] << endl;

          //// if the exit of the path is the exit
          //if (path->exit_vertex_ == exit_vertex) {
          //double weight_path;
          //if (path->entrance_vertex_ == entrance_vertex) {
          //weight_path = 1.0;
          //}
          //else {
          //weight_path = dist_entrance / (dist_entrance + dist_exit);
          ////if (weight_path > 1.0) {
          ////weight_path = 1.0;
          ////}
          //}
          //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_exit * weight_path;
          //}

          //cout << "entrance " << entrance_vertex->name() << ", exit " << exit_vertex->name() << ", entrance path " << path->entrance_vertex_->name() << ", exit path, " << path->exit_vertex_->name() << ", weight_entrance + exit=" << table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] << endl;

          // if the entrance of the path is the entrance
          if (path->entrance_vertex_ == entrance_vertex) {
            double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), path_edge_proba_map[path].edge_->getAngle());
            double sigma_angle = M_PI / 4.0;// attention, 4.0 ok
            double weight_angle = gaussian_func(angle, 0.0, sigma_angle);
            double weight_path = gaussian_func(dist_entrance, 0.0, dist_entrance_exit / 2.0);//1.0 - dist_entrance / dist_entrance_exit;//dist_exit / dist_entrance_exit;
            if (path->exit_vertex_ == exit_vertex) {
              table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += 1.0;
            }
            else {
              table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_entrance * weight_path * weight_angle;
            }
          }

          // if the exit of the path is the exit
          if (path->exit_vertex_ == exit_vertex) {
            double weight_path = gaussian_func(dist_exit, 0.0, dist_entrance_exit / 2.0);//1.0 - dist_exit / dist_entrance_exit;//dist_entrance / dist_entrance_exit;
            if (path->entrance_vertex_ == entrance_vertex) {
              table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += 1.0;
            }
            else {
              table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_exit * weight_path;
            }
          }

          //// if the entrance of the path is the entrance
          //if (path->entrance_vertex_ == entrance_vertex) {
          //double angle = CGAL_Geometry::deltaAngle((*(exit_vertex->getOutEdges().begin()))->getAngle(), path_edge_proba_map[path].edge_->getAngle());
          //double sigma_angle = M_PI / 6.0;
          //double weight_angle = gaussian_func(angle, 0.0, sigma_angle);
          //double weight_path = dist_exit / dist_entrance_exit;
          //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_entrance * weight_path;// * weight_angle;
          //}

          //// if the exit of the path is the exit
          //if (path->exit_vertex_ == exit_vertex) {
          //double weight_path = dist_entrance / dist_entrance_exit;
          //table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] += weight_exit * weight_path;
          //}

          sum_probas_path += table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)];

        }

        // Normalize
        for (uint32_t p = 0; p < nr_possible_paths_; p++) {
          table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] = table_path_knowing_exit[subToIndex(p,
              nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] / sum_probas_path;
        }

      }

    }

    plDistributionTable* P_path = new plDistributionTable(bn_path_, bn_entrance_ ^ bn_exit_, table_path_knowing_exit);
    P_path_ = *P_path;

    // Specification of P(turn_signal_int)

    plProbValue table_turn_signal_int_knowing_exit[nr_possible_turn_signals_ * nr_possible_exit_lanes_];

    for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {

      RndfVertex* exit_vertex = index_exit_map_[ex];

      uint32_t correct_turn_signal;
      uint32_t incorrect_turn_signal;
      uint32_t no_turn_signal = 0;

      //const RndfVertex* v1 = ->fromVertex();
      //const RndfVertex* v2 = (*(exit_vertex->getOutEdges().begin()))->toVertex();
      //Vector_2 vec(Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ));
      double angle = (*(exit_vertex->getOutEdges().begin()))->getAngle() - vehicle_manager_->vehicle_map[*veh_id_it].yawMatchedFrom();

      //double angle = vehicle_manager_->vehicle_map[*veh_id_it].angleToEdge(*(exit_vertex->getOutEdges().begin()));
      if (angle > M_PI) {
        angle = angle - 2.0 * M_PI;
      }
      else if (angle < -M_PI) {
        angle = angle + 2.0 * M_PI;
      }
      //cout << "in intersection exit " << exit_vertex->name() << " : angle=" << angle*180.0/M_PI << endl;

      double ratio = abs(angle) / (0.5 * M_PI);
      if (ratio > 1.0) {
        ratio = 1.0;
      }

      if (angle > 0.0) {
        correct_turn_signal = 1;
        incorrect_turn_signal = 2;

      }
      else {
        correct_turn_signal = 2;
        incorrect_turn_signal = 1;
      }

      double proba_choose_correct_turn_signal = proba_min_choose_correct_turn_signal_ + (proba_max_choose_correct_turn_signal_int_
          - proba_min_choose_correct_turn_signal_) * ratio;
      double proba_choose_incorrect_turn_signal = proba_choose_incorrect_turn_signal_int_;

      table_turn_signal_int_knowing_exit[subToIndex(correct_turn_signal, nr_possible_turn_signals_, ex)] = proba_choose_correct_turn_signal;
      table_turn_signal_int_knowing_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, ex)] = proba_choose_incorrect_turn_signal;
      //if (table_turn_signal_int_knowing_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, ex)] > proba_choose_correct_turn_signal_int) {
      //table_turn_signal_int_knowing_exit[subToIndex(incorrect_turn_signal, nr_possible_turn_signals_, ex)] = proba_choose_correct_turn_signal_int;
      //}
      table_turn_signal_int_knowing_exit[subToIndex(no_turn_signal, nr_possible_turn_signals_, ex)] = 1.0 - table_turn_signal_int_knowing_exit[subToIndex(
          correct_turn_signal, nr_possible_turn_signals_, ex)] - table_turn_signal_int_knowing_exit[subToIndex(incorrect_turn_signal,
          nr_possible_turn_signals_, ex)];
      //cout << "in intersection exit " << exit_vertex->name() << " : none=" << table_turn_signal_int_knowing_exit[subToIndex(0, nr_possible_turn_signals_, ex)] << ", left=" << table_turn_signal_int_knowing_exit[subToIndex(1, nr_possible_turn_signals_, ex)] << ", right=" << table_turn_signal_int_knowing_exit[subToIndex(2, nr_possible_turn_signals_, ex)] << endl;

    }

    plDistributionTable* P_turn_signal_int = new plDistributionTable(bn_turn_signal_int_, bn_exit_, table_turn_signal_int_knowing_exit);
    P_turn_signal_int_ = *P_turn_signal_int;

    //// Temporary: Find possible destinations
    //VertexProbaMap exit_vertex_proba_map;
    //computeDistributionExitVertices(&exit_vertex_proba_map, path_edge_proba_map);

    //// Class variable exit vertices
    //for (VertexProbaMap::iterator vertex_map_it = exit_vertex_proba_map.begin(); vertex_map_it != exit_vertex_proba_map.end(); ++vertex_map_it) {
    //VehIdVertex veh_id_vertex;
    //veh_id_vertex.veh_id_ = *veh_id_it;
    //veh_id_vertex.vertex_ = vertex_map_it->first;
    //veh_id_exit_vertex_proba_map_[veh_id_vertex] = vertex_map_it->second;
    //}

  }
}

void ObstaclePredictor::updateVehiclesApproachingIntersection() {
  //////////////////////////////////////
  // Vehicle approaching intersection //
  //////////////////////////////////////

  for (VehicleIdSet::iterator veh_id_it = relevant_vehicles_approaching_intersection_.begin(); veh_id_it != relevant_vehicles_approaching_intersection_.end(); ++veh_id_it) {

    //veh_id_entrance_vertices_proba_map_.clear();
    //veh_id_turn_signal_proba_map_.clear();

    // Calculate probability of the vehicle being on an edge
    EdgeProbaMap edge_proba_map;

    RndfEdge* matched_edge = vehicle_manager_->vehicle_map[*veh_id_it].edge();

    if (!matched_edge->isIntersectionEdge()) {
      // Insert matched edge
      insertAdjacentEdges(&(vehicle_manager_->vehicle_map[*veh_id_it]), matched_edge, &edge_proba_map, 0);
      // Insert left adjacent edges
      insertAdjacentEdges(&(vehicle_manager_->vehicle_map[*veh_id_it]), matched_edge, &edge_proba_map, 1);
      // Insert right adjacent edges
      insertAdjacentEdges(&(vehicle_manager_->vehicle_map[*veh_id_it]), matched_edge, &edge_proba_map, 2);
    }
    else {
      insertAdjacentEdgesInIntersection(&(vehicle_manager_->vehicle_map[*veh_id_it]), matched_edge, &edge_proba_map);
    }

    // findEdgesLeadingToVertex and findReachableVerticesFromEdges
    EdgeVerticesMap reachable_vertices_from_edges;
    findReachableVerticesFromEdges(edge_proba_map, &reachable_vertices_from_edges);
    VertexEdgesMap edges_leading_to_vertices;
    findEdgesLeadingToVertices(reachable_vertices_from_edges, &edges_leading_to_vertices);

    // Normalize probas of edges
    double sum_of_probas_edges = 0.0;
    for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
      sum_of_probas_edges += edge_map_it->second;
      //cout << "Before normalization, vehicle " << *veh_id_it << " is on edge " << edge_map_it->first->name() << " with proba " << edge_map_it->second << endl;
    }
    if (sum_of_probas_edges > 0.0) {
      for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
        edge_map_it->second = edge_map_it->second / sum_of_probas_edges;
        //cout << "Vehicle " << *veh_id_it << " is on edge " << edge_map_it->first->name() << " with proba " << edge_map_it->second << endl;
      }
    }

    // Calculate probabilities for each entrance vertex (soft evidence -> Jeffrey's rule)
    VertexProbaMap entrance_vertex_proba_map;
    computeDistributionEntranceVertices(&entrance_vertex_proba_map, edge_proba_map, edges_leading_to_vertices);

    // Class variable entrance vertices
    for (VertexProbaMap::iterator vertex_map_it = entrance_vertex_proba_map.begin(); vertex_map_it != entrance_vertex_proba_map.end(); ++vertex_map_it) {
      VehIdVertex veh_id_vertex;
      veh_id_vertex.veh_id_ = *veh_id_it;
      veh_id_vertex.vertex_ = vertex_map_it->first;
      veh_id_entrance_vertex_proba_map_[veh_id_vertex] = vertex_map_it->second;
      if (*veh_id_it == 8) {
        //HERECOUTcout << "Vehicle " << *veh_id_it << " is heading for entrance vertex " << vertex_map_it->first->name() << " with probability " << vertex_map_it->second << endl;
      }
    }

    //HERECHANGETS Class variable turn signal: for now, initialize manually
    TurnSignalProbaMap turn_signal_proba_map;
    //cout << "turn signal=" << vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() << endl;
    if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 0) {
      turn_signal_proba_map[0] = 0.9;//1.0;//
      turn_signal_proba_map[1] = 0.05;//0.0;//
      turn_signal_proba_map[2] = 0.05;//0.0;//
    }
    else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 1) {
      turn_signal_proba_map[0] = 0.05;//0.0;//
      turn_signal_proba_map[1] = 0.9;//1.0;//
      turn_signal_proba_map[2] = 0.05;//0.0;//
    }
    else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 2) {
      turn_signal_proba_map[0] = 0.05;//0.0;//
      turn_signal_proba_map[1] = 0.05;//0.0;//
      turn_signal_proba_map[2] = 0.9;//1.0;//
    }
    else if (vehicle_manager_->vehicle_map[*veh_id_it].turnSignal() == 127) {
      turn_signal_proba_map[0] = 0.33334;
      turn_signal_proba_map[1] = 0.33333;
      turn_signal_proba_map[2] = 0.33333;
    }
    else {
      cout << "Warning, unable to interpret turn signal information!" << endl;
    }
    for (TurnSignalProbaMap::iterator turn_signal_map_it = turn_signal_proba_map.begin(); turn_signal_map_it != turn_signal_proba_map.end(); ++turn_signal_map_it) {
      VehIdTurnSignal veh_id_turn_signal;
      veh_id_turn_signal.veh_id_ = *veh_id_it;
      veh_id_turn_signal.turn_signal_ = turn_signal_map_it->first;
      veh_id_turn_signal_proba_map_[veh_id_turn_signal] = turn_signal_map_it->second;
    }

    // Specification of P(path)

    plProbValue table_path_knowing_exit[nr_possible_paths_ * nr_possible_entrance_lanes_ * nr_possible_exit_lanes_];

    for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {
      for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {
        for (uint32_t p = 0; p < nr_possible_paths_; p++) {
          table_path_knowing_exit[subToIndex(p, nr_possible_paths_, en, ex, nr_possible_exit_lanes_)] = 1.0 / nr_possible_paths_;
        }
      }
    }

    plDistributionTable* P_path = new plDistributionTable(bn_path_, bn_entrance_ ^ bn_exit_, table_path_knowing_exit);
    P_path_ = *P_path;

    // Specification of P(turn_signal_int)

    plProbValue table_turn_signal_int_knowing_exit[nr_possible_turn_signals_ * nr_possible_exit_lanes_];

    for (uint32_t ex = 0; ex < nr_possible_exit_lanes_; ex++) {
      table_turn_signal_int_knowing_exit[subToIndex(0, nr_possible_turn_signals_, ex)] = 1.0 / nr_possible_turn_signals_;
      table_turn_signal_int_knowing_exit[subToIndex(1, nr_possible_turn_signals_, ex)] = 1.0 / nr_possible_turn_signals_;
      table_turn_signal_int_knowing_exit[subToIndex(2, nr_possible_turn_signals_, ex)] = 1.0 / nr_possible_turn_signals_;
    }

    plDistributionTable* P_turn_signal_int = new plDistributionTable(bn_turn_signal_int_, bn_exit_, table_turn_signal_int_knowing_exit);
    P_turn_signal_int_ = *P_turn_signal_int;

    //// Temporary: find possible destinations
    //VertexProbaMap exit_vertex_proba_map;
    //computeDistributionExitVertices(&exit_vertex_proba_map, entrance_vertex_proba_map);

    //// Class variable exit vertices
    //for (VertexProbaMap::iterator vertex_map_it = exit_vertex_proba_map.begin(); vertex_map_it != exit_vertex_proba_map.end(); ++vertex_map_it) {
    //VehIdVertex veh_id_vertex;
    //veh_id_vertex.veh_id_ = *veh_id_it;
    //veh_id_vertex.vertex_ = vertex_map_it->first;
    //veh_id_exit_vertex_proba_map_[veh_id_vertex] = vertex_map_it->second;
    //}

  }
}

void ObstaclePredictor::update() {

  if (vehicle_manager_->vehicle_map.empty()) {
    return;
  }
  if (!isAllWayStopIntersection()) {
    return;
  } // TODO: check if necessary

  pthread_mutex_lock(&intersection_predictor_mutex_);
  // Relevant vehicles = vehicles in next_intersection + vehicles approaching next_intersection

  //relevant_vehicles_old_ = relevant_vehicles_;
  relevant_vehicles_.clear();
  //relevant_vehicles_in_intersection_old_ = relevant_vehicles_in_intersection_;
  relevant_vehicles_in_intersection_.clear();
  //relevant_vehicles_approaching_intersection_old_ = relevant_vehicles_approaching_intersection_;
  relevant_vehicles_approaching_intersection_.clear();

  determineRelevantVehicles();
  updateVehiclesInIntersection();
  updateVehiclesApproachingIntersection();

  /////////////////////////////////
  // Bayesian Network: question  //
  /////////////////////////////////

  for (VehicleIdSet::iterator veh_id_it = relevant_vehicles_.begin(); veh_id_it != relevant_vehicles_.end(); ++veh_id_it) {

    VertexProbaMap exit_vertex_proba_map;

    bool in_intersection = (relevant_vehicles_in_intersection_.find(*veh_id_it) != relevant_vehicles_in_intersection_.end());

    VehIdVertex veh_id_vertex;
    veh_id_vertex.veh_id_ = *veh_id_it;

    veh_id_vertex.vertex_ = index_entrance_map_[0];
    bool seen_before_intersection = (veh_id_entrance_vertex_proba_map_.find(veh_id_vertex) != veh_id_entrance_vertex_proba_map_.end());
    //HERECOUTcout << "Vehicle " << *veh_id_it << " was seen before intersection? " << seen_before_intersection << endl;

    //cout << "Seen before intersection = " << seen_before_intersection << endl;

    // Specification of P_soft_evidence

    if (in_intersection) {

      // Specification of P_soft_evidence_path_

      VehIdPath veh_id_path;
      veh_id_path.veh_id_ = *veh_id_it;
      plProbValue table_soft_evidence_path[nr_possible_paths_];
      for (uint32_t p = 0; p < nr_possible_paths_; p++) {
        veh_id_path.path_ = intersection_paths_[index_path_map_[p]];
        table_soft_evidence_path[p] = veh_id_path_proba_map_[veh_id_path];
      }
      plProbTable* P_soft_evidence_path = new plProbTable(bn_soft_evidence_path_, table_soft_evidence_path);
      P_soft_evidence_path_ = *P_soft_evidence_path;

      // Specification of P_soft_evidence_turn_signal_int_

      VehIdTurnSignal veh_id_turn_signal_int;
      veh_id_turn_signal_int.veh_id_ = *veh_id_it;
      plProbValue table_soft_evidence_turn_signal_int[nr_possible_turn_signals_];
      for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
        veh_id_turn_signal_int.turn_signal_ = t;
        table_soft_evidence_turn_signal_int[t] = veh_id_turn_signal_int_proba_map_[veh_id_turn_signal_int];
      }
      plProbTable* P_soft_evidence_turn_signal_int = new plProbTable(bn_soft_evidence_turn_signal_int_, table_soft_evidence_turn_signal_int);
      P_soft_evidence_turn_signal_int_ = *P_soft_evidence_turn_signal_int;

    }

    else {

      // Specification of P_soft_evidence_path_

      plProbValue table_soft_evidence_path[nr_possible_paths_];
      for (uint32_t p = 0; p < nr_possible_paths_; p++) {
        table_soft_evidence_path[p] = 1.0 / nr_possible_paths_;
      }
      plProbTable* P_soft_evidence_path = new plProbTable(bn_soft_evidence_path_, table_soft_evidence_path);
      P_soft_evidence_path_ = *P_soft_evidence_path;

      // Specification of P_soft_evidence_turn_signal_int_

      plProbValue table_soft_evidence_turn_signal_int[nr_possible_turn_signals_];
      for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
        table_soft_evidence_turn_signal_int[t] = 1.0 / nr_possible_turn_signals_;
      }
      plProbTable* P_soft_evidence_turn_signal_int = new plProbTable(bn_soft_evidence_turn_signal_int_, table_soft_evidence_turn_signal_int);
      P_soft_evidence_turn_signal_int_ = *P_soft_evidence_turn_signal_int;

    }

    if (seen_before_intersection) {

      // Specification of P_soft_evidence_entrance_

      plProbValue table_soft_evidence_entrance[nr_possible_entrance_lanes_];
      for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {
        veh_id_vertex.vertex_ = index_entrance_map_[en];
        table_soft_evidence_entrance[en] = veh_id_entrance_vertex_proba_map_[veh_id_vertex];
      }
      plProbTable* P_soft_evidence_entrance = new plProbTable(bn_soft_evidence_entrance_, table_soft_evidence_entrance);
      P_soft_evidence_entrance_ = *P_soft_evidence_entrance;

      // Specification of P_soft_evidence_turn_signal_

      VehIdTurnSignal veh_id_turn_signal;
      veh_id_turn_signal.veh_id_ = *veh_id_it;
      plProbValue table_soft_evidence_turn_signal[nr_possible_turn_signals_];
      for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
        veh_id_turn_signal.turn_signal_ = t;
        table_soft_evidence_turn_signal[t] = veh_id_turn_signal_proba_map_[veh_id_turn_signal];
      }
      plProbTable* P_soft_evidence_turn_signal = new plProbTable(bn_soft_evidence_turn_signal_, table_soft_evidence_turn_signal);
      P_soft_evidence_turn_signal_ = *P_soft_evidence_turn_signal;

    }

    else {

      // Specification of P_soft_evidence_entrance_

      plProbValue table_soft_evidence_entrance[nr_possible_entrance_lanes_];
      for (uint32_t en = 0; en < nr_possible_entrance_lanes_; en++) {
        table_soft_evidence_entrance[en] = 1.0 / nr_possible_entrance_lanes_;
      }
      plProbTable* P_soft_evidence_entrance = new plProbTable(bn_soft_evidence_entrance_, table_soft_evidence_entrance);
      P_soft_evidence_entrance_ = *P_soft_evidence_entrance;

      // Specification of P_soft_evidence_turn_signal_

      plProbValue table_soft_evidence_turn_signal[nr_possible_turn_signals_];
      for (uint32_t t = 0; t < nr_possible_turn_signals_; t++) {
        table_soft_evidence_turn_signal[t] = 1.0 / nr_possible_turn_signals_;
      }
      P_soft_evidence_turn_signal_ = plProbTable(bn_soft_evidence_turn_signal_, table_soft_evidence_turn_signal);
    }

    // Get the inferred conditional distribution
    plCndDistribution CndP_exit;

    jd_ = plJointDistribution(bn_exit_ ^ bn_entrance_ ^ bn_segment_ ^ bn_path_ ^ bn_turn_signal_ ^ bn_turn_signal_int_ ^ bn_wrong_lane_
        ^ bn_wrong_segment_ ^ bn_coherence_entrance_ ^ bn_soft_evidence_entrance_ ^ bn_coherence_turn_signal_ ^ bn_soft_evidence_turn_signal_
        ^ bn_coherence_turn_signal_int_ ^ bn_soft_evidence_turn_signal_int_ ^ bn_coherence_path_ ^ bn_soft_evidence_path_, P_entrance_ * P_exit_ * P_segment_
        * P_path_ * P_turn_signal_ * P_turn_signal_int_ * P_wrong_lane_ * P_wrong_segment_ * P_coherence_entrance_ * P_soft_evidence_entrance_
        * P_coherence_turn_signal_ * P_soft_evidence_turn_signal_ * P_coherence_turn_signal_int_ * P_soft_evidence_turn_signal_int_ * P_coherence_path_
        * P_soft_evidence_path_);

    //jd_.ask(CndP_exit, bn_exit_, bn_coherence_entrance_^bn_coherence_turn_signal_^bn_coherence_turn_signal_int_^bn_coherence_path_^bn_segment_);

    //if (seen_before_intersection) {
    //jd_.ask(CndP_exit, bn_exit_, bn_coherence_entrance_^bn_coherence_turn_signal_^bn_coherence_turn_signal_int_^bn_coherence_path_^bn_segment_);
    //}
    //else {
    jd_.ask(CndP_exit, bn_exit_, bn_coherence_entrance_ ^ bn_coherence_turn_signal_ ^ bn_coherence_turn_signal_int_ ^ bn_coherence_path_);
    //}

    //if (in_intersection) {
    //if (seen_before_intersection) {
    ////plJointDistribution* jd = new plJointDistribution(bn_exit_^bn_entrance_^bn_segment_^bn_path_^bn_turn_signal_^bn_turn_signal_int_^bn_wrong_lane_^bn_wrong_segment_^bn_coherence_entrance_^bn_soft_evidence_entrance_^bn_coherence_turn_signal_^bn_soft_evidence_turn_signal_^bn_coherence_turn_signal_int_^bn_soft_evidence_turn_signal_int_^bn_coherence_path_^bn_soft_evidence_path_, P_entrance_*P_exit_*P_segment_*P_path_*P_turn_signal_*P_turn_signal_int_*P_wrong_lane_*P_wrong_segment_*P_coherence_entrance_*P_soft_evidence_entrance_*P_coherence_turn_signal_*P_soft_evidence_turn_signal_*P_coherence_turn_signal_int_*P_soft_evidence_turn_signal_int_*P_coherence_path_*P_soft_evidence_path_);
    ////jd_ = *jd;
    //jd_.ask(CndP_exit, bn_exit_, bn_coherence_entrance_^bn_coherence_turn_signal_^bn_coherence_turn_signal_int_^bn_coherence_path_^bn_segment_);
    //}
    //else {
    ////plJointDistribution* jd = new plJointDistribution(bn_exit_^bn_path_^bn_coherence_path_^bn_soft_evidence_path_^bn_turn_signal_int_^bn_coherence_turn_signal_int_^bn_soft_evidence_turn_signal_int_, P_exit_*P_path_*P_coherence_path_*P_soft_evidence_path_*P_turn_signal_int_*P_coherence_turn_signal_int_*P_soft_evidence_turn_signal_int_);
    ////jd_ = *jd;
    //jd_.ask(CndP_exit, bn_exit_, bn_coherence_path_^bn_coherence_turn_signal_int_);
    //}
    //}
    //else {
    ////plJointDistribution* jd = new plJointDistribution(bn_entrance_^bn_exit_^bn_segment_^bn_turn_signal_^bn_wrong_lane_^bn_wrong_segment_^bn_coherence_entrance_^bn_soft_evidence_entrance_^bn_coherence_turn_signal_^bn_soft_evidence_turn_signal_, P_entrance_*P_exit_*P_segment_*P_turn_signal_*P_wrong_lane_*P_wrong_segment_*P_coherence_entrance_*P_soft_evidence_entrance_*P_coherence_turn_signal_*P_soft_evidence_turn_signal_);
    ////jd_ = *jd;
    //jd_.ask(CndP_exit, bn_exit_, bn_coherence_entrance_^bn_coherence_turn_signal_^bn_segment_);
    //}

    // Create the value representing the evidence
    plValues evidence;

    //if (seen_before_intersection) {
    //evidence_p = new plValues(bn_coherence_entrance_^bn_coherence_turn_signal_^bn_coherence_turn_signal_int_^bn_coherence_path_^bn_segment_);
    //}
    //else {
    evidence = plValues(bn_coherence_entrance_ ^ bn_coherence_turn_signal_ ^ bn_coherence_turn_signal_int_ ^ bn_coherence_path_);
    //}
    //if (in_intersection && seen_before_intersection) {
    //evidence = plValues(bn_coherence_entrance_^bn_coherence_turn_signal_^bn_coherence_turn_signal_int_^bn_coherence_path_^bn_segment_);
    //}
    //else if (in_intersection && !seen_before_intersection) {
    //evidence = plValues(bn_coherence_path_^bn_coherence_turn_signal_int_);
    //}
    //else {
    //evidence = plValues(bn_coherence_entrance_^bn_coherence_turn_signal_^bn_segment_);
    //}

    //if (seen_before_intersection) {

    // Soft evidence coherence entrance
    evidence[bn_coherence_entrance_] = true;

    //// Hard evidence segment
    //// Find max proba entrance
    //double max_proba_entrance = 0.0;
    //RndfVertex* entrance_vertex = NULL;
    //for (VehIdVertexProbaMap::const_iterator vertex_map_it = veh_id_entrance_vertex_proba_map_.begin(); vertex_map_it != veh_id_entrance_vertex_proba_map_.end(); ++vertex_map_it) {
    //if ((vertex_map_it->first.veh_id_ == *veh_id_it) && (vertex_map_it->second > max_proba_entrance)) {
    //max_proba_entrance = vertex_map_it->second;
    //entrance_vertex = vertex_map_it->first.vertex_;
    //}
    //}
    //evidence[bn_segment_] = vertex_segment_map_[entrance_vertex];
    ////cout << "evidence: segment " << vertex_segment_map_[entrance_vertex] << endl;

    // Soft evidence coherence turn signal
    evidence[bn_coherence_turn_signal_] = true;

    //}

    //if (in_intersection) {

    //// Soft evidence coherence path
    evidence[bn_coherence_path_] = true;

    //// Soft evidence turn signal int
    evidence[bn_coherence_turn_signal_int_] = true;

    //}

    // Get the distribution representing P(exit|evidence)
    plDistribution P_exit_evidence;
    CndP_exit.instantiate(P_exit_evidence, evidence);
    //cout << P_exit_evidence << endl;

    // Get the normalized distribution representing P(exit|evidence)
    plDistribution T_P_exit_evidence;
    P_exit_evidence.compile(T_P_exit_evidence);
    //cout << T_P_exit_evidence << endl;

    std::vector<plValues> values;
    std::vector<plProbValue> probabilities;
    T_P_exit_evidence.tabulate(values, probabilities);
    for (uint32_t ind = 0; ind < nr_possible_exit_lanes_; ind++) {
      exit_vertex_proba_map[index_exit_map_[ind]] = probabilities[ind];
      if (*veh_id_it == 8) {
        //HERECOUTcout << "Vehicle " << *veh_id_it << " is heading for exit vertex " << index_exit_map_[ind]->name() << " with probability " << probabilities[ind] << endl;
      }
    }

    // Class variable exit vertices
    for (VertexProbaMap::iterator vertex_map_it = exit_vertex_proba_map.begin(); vertex_map_it != exit_vertex_proba_map.end(); ++vertex_map_it) {
      VehIdVertex veh_id_vertex;
      veh_id_vertex.veh_id_ = *veh_id_it;
      veh_id_vertex.vertex_ = vertex_map_it->first;
      veh_id_exit_vertex_proba_map_[veh_id_vertex] = vertex_map_it->second;
    }

  }

  pthread_mutex_unlock(&intersection_predictor_mutex_);
}

void ObstaclePredictor::computeDistributionEntranceVertices(VertexProbaMap* entrance_vertex_proba_map, EdgeProbaMap edge_proba_map,
    VertexEdgesMap edges_leading_to_vertices) {

  // soft_evidence
  EdgeProbaMap soft_evidence;
  for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
    soft_evidence.insert(EdgeProbaMap::value_type(edge_map_it->first, edge_map_it->second));
  }

  // marginal_distribution
  EdgeProbaMap marginal_distribution;
  double marginal_proba_edge = 1.0 / edge_proba_map.size();
  for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
    marginal_distribution.insert(EdgeProbaMap::value_type(edge_map_it->first, marginal_proba_edge));
    //cout << "marginal_distribution=" << marginal_proba_edge << endl;
  }

  // joint_distribution_orig and joint_distribution_updated
  VertexEdgeProbaMap joint_distribution_orig;
  VertexEdgeProbaMap joint_distribution_updated;
  for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {
    for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
      VertexEdge vertex_edge;
      vertex_edge.vertex_ = *vertex_it;
      vertex_edge.edge_ = edge_map_it->first;
      double proba = 0.0;
      if (edges_leading_to_vertices[*vertex_it]->size() > 0) {
        if (edges_leading_to_vertices[*vertex_it]->find(edge_map_it->first) != edges_leading_to_vertices[*vertex_it]->end()) {
          proba = 1.0 / entrance_vertices_.size() * 1.0 / edges_leading_to_vertices[*vertex_it]->size();
        }
      }
      //cout << "proba=" << proba << endl;
      joint_distribution_orig.insert(VertexEdgeProbaMap::value_type(vertex_edge, proba));
      joint_distribution_updated.insert(VertexEdgeProbaMap::value_type(vertex_edge, proba));
    }
  }

  // Update joint distribution (Jeffrey's rule)
  for (VertexEdgeProbaMap::iterator vertex_edge_map_it = joint_distribution_orig.begin(); vertex_edge_map_it != joint_distribution_orig.end(); ++vertex_edge_map_it) {
    VertexEdge vertex_edge;
    vertex_edge.vertex_ = vertex_edge_map_it->first.vertex_;
    vertex_edge.edge_ = vertex_edge_map_it->first.edge_;
    if (marginal_distribution[vertex_edge_map_it->first.edge_] > 0.0) {
      joint_distribution_updated[vertex_edge] = vertex_edge_map_it->second * soft_evidence[vertex_edge_map_it->first.edge_]
          / marginal_distribution[vertex_edge_map_it->first.edge_];
      //cout << "joint_distribution_updated[vertex_edge]=" << joint_distribution_updated[vertex_edge] << endl;
    }
  }

  // Proba for each entrance vertex
  double sum_of_probas_entrance = 0.0;
  for (VertexEdgeProbaMap::const_iterator vertex_edge_map_it = joint_distribution_updated.begin(); vertex_edge_map_it != joint_distribution_updated.end(); ++vertex_edge_map_it) {
    entrance_vertex_proba_map->insert(VertexProbaMap::value_type(vertex_edge_map_it->first.vertex_, 0.0));
    (*entrance_vertex_proba_map)[vertex_edge_map_it->first.vertex_] += vertex_edge_map_it->second;
    sum_of_probas_entrance += vertex_edge_map_it->second;
  }

  // Normalize probas
  if (sum_of_probas_entrance > 0.0) {
    for (VertexProbaMap::iterator vertex_map_it = entrance_vertex_proba_map->begin(); vertex_map_it != entrance_vertex_proba_map->end(); ++vertex_map_it) {
      vertex_map_it->second = vertex_map_it->second / sum_of_probas_entrance;
      //cout << "Vehicle is heading for vertex " << vertex_map_it->first->name() << " with probability " << vertex_map_it->second << endl;
    }
  }

}

void ObstaclePredictor::computeDistributionExitVertices(VertexProbaMap* exit_vertex_proba_map, VertexProbaMap entrance_vertex_proba_map) {

  // soft_evidence
  VertexProbaMap soft_evidence;
  for (VertexProbaMap::iterator entrance_vertex_map_it = entrance_vertex_proba_map.begin(); entrance_vertex_map_it != entrance_vertex_proba_map.end(); ++entrance_vertex_map_it) {
    soft_evidence.insert(VertexProbaMap::value_type(entrance_vertex_map_it->first, entrance_vertex_map_it->second));
  }

  // marginal_distribution
  VertexProbaMap marginal_distribution;
  double marginal_proba_vertex = 1.0 / entrance_vertex_proba_map.size();
  for (VertexProbaMap::iterator entrance_vertex_map_it = entrance_vertex_proba_map.begin(); entrance_vertex_map_it != entrance_vertex_proba_map.end(); ++entrance_vertex_map_it) {
    marginal_distribution.insert(VertexProbaMap::value_type(entrance_vertex_map_it->first, marginal_proba_vertex));
  }

  // joint_distribution_orig and joint_distribution_updated
  VertexVertexProbaMap joint_distribution_orig;
  VertexVertexProbaMap joint_distribution_updated;
  for (TRndfVertexSet::const_iterator exit_vertex_it = exit_vertices_.begin(); exit_vertex_it != exit_vertices_.end(); ++exit_vertex_it) {
    // Count number of paths leading to exit vertex
    uint32_t nr_paths_leading_to_vertex = 0;
    for (TRndfVertexSet::const_iterator entrance_vertex_it = exit_vertex_entrance_vertices_map_[*exit_vertex_it]->begin(); entrance_vertex_it
        != exit_vertex_entrance_vertices_map_[*exit_vertex_it]->end(); ++entrance_vertex_it) {
      if (entrance_vertex_proba_map[*entrance_vertex_it] > 0.0) {
        nr_paths_leading_to_vertex++;
      }
    }
    //cout << "Nr paths leading to vertex " << (*exit_vertex_it)->name() << " = " << nr_paths_leading_to_vertex << endl;
    for (VertexProbaMap::iterator entrance_vertex_map_it = entrance_vertex_proba_map.begin(); entrance_vertex_map_it != entrance_vertex_proba_map.end(); ++entrance_vertex_map_it) {
      VertexVertex entrance_exit;
      entrance_exit.entrance_vertex_ = entrance_vertex_map_it->first;
      entrance_exit.exit_vertex_ = *exit_vertex_it;
      double proba = 0.0;
      if ((nr_paths_leading_to_vertex > 0) && (entrance_vertex_map_it->second > 0.0) && (exit_vertex_entrance_vertices_map_[entrance_exit.exit_vertex_]->find(
          entrance_exit.entrance_vertex_) != exit_vertex_entrance_vertices_map_[entrance_exit.exit_vertex_]->end())) {
        proba = 1.0 / exit_vertices_.size() * 1.0 / nr_paths_leading_to_vertex;
      }
      joint_distribution_orig.insert(VertexVertexProbaMap::value_type(entrance_exit, proba));
      joint_distribution_updated.insert(VertexVertexProbaMap::value_type(entrance_exit, proba));
      //cout << "entrance_vertex " << entrance_exit.entrance_vertex_->name() << " exit vertex " << entrance_exit.exit_vertex_->name() << " proba " << proba << endl;
    }
  }

  // Update joint distribution (Jeffrey's rule)
  for (VertexVertexProbaMap::iterator entrance_exit_map_it = joint_distribution_orig.begin(); entrance_exit_map_it != joint_distribution_orig.end(); ++entrance_exit_map_it) {
    VertexVertex entrance_exit;
    entrance_exit.entrance_vertex_ = entrance_exit_map_it->first.entrance_vertex_;
    entrance_exit.exit_vertex_ = entrance_exit_map_it->first.exit_vertex_;
    if (marginal_distribution[entrance_exit.entrance_vertex_] > 0) {
      joint_distribution_updated[entrance_exit] = entrance_exit_map_it->second * soft_evidence[entrance_exit.entrance_vertex_]
          / marginal_distribution[entrance_exit.entrance_vertex_];
    }
  }

  // Proba for each exit vertex
  double sum_of_probas_exit = 0.0;
  for (VertexVertexProbaMap::const_iterator entrance_exit_map_it = joint_distribution_updated.begin(); entrance_exit_map_it != joint_distribution_updated.end(); ++entrance_exit_map_it) {
    exit_vertex_proba_map->insert(VertexProbaMap::value_type(entrance_exit_map_it->first.exit_vertex_, 0.0));
    (*exit_vertex_proba_map)[entrance_exit_map_it->first.exit_vertex_] += entrance_exit_map_it->second;
    sum_of_probas_exit += entrance_exit_map_it->second;
  }

  // Normalize probas
  if (sum_of_probas_exit > 0.0) {
    for (VertexProbaMap::iterator vertex_map_it = exit_vertex_proba_map->begin(); vertex_map_it != exit_vertex_proba_map->end(); ++vertex_map_it) {
      vertex_map_it->second = vertex_map_it->second / sum_of_probas_exit;
      cout << "Vehicle has destination " << vertex_map_it->first->name() << " with probability " << vertex_map_it->second << endl;
    }
  }

}

void ObstaclePredictor::computeDistributionExitVertices(VertexProbaMap* exit_vertex_proba_map, PathEdgeProbaMap path_edge_proba_map) {

  // soft_evidence
  PathProbaMap soft_evidence;
  for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
    soft_evidence.insert(PathProbaMap::value_type(path_map_it->first, path_map_it->second.proba_));
  }

  // marginal_distribution
  PathProbaMap marginal_distribution;
  double marginal_proba_vertex = 1.0 / path_edge_proba_map.size();
  for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
    marginal_distribution.insert(PathProbaMap::value_type(path_map_it->first, marginal_proba_vertex));
  }

  // joint_distribution_orig and joint_distribution_updated
  VertexPathProbaMap joint_distribution_orig;
  VertexPathProbaMap joint_distribution_updated;
  for (TRndfVertexSet::const_iterator vertex_it = exit_vertices_.begin(); vertex_it != exit_vertices_.end(); ++vertex_it) {
    // Count number of paths leading to the vertex
    uint32_t nr_paths_leading_to_vertex = exit_vertex_entrance_vertices_map_[*vertex_it]->size();
    cout << "Nr paths leading to vertex " << (*vertex_it)->name() << " = " << nr_paths_leading_to_vertex << endl;
    for (PathEdgeProbaMap::iterator path_map_it = path_edge_proba_map.begin(); path_map_it != path_edge_proba_map.end(); ++path_map_it) {
      VertexPath vertex_path;
      vertex_path.vertex_ = *vertex_it;
      vertex_path.path_ = path_map_it->first;
      double proba = 0.0;
      if ((nr_paths_leading_to_vertex > 0) && (path_map_it->first->exit_vertex_ == (*vertex_it))) {
        proba = 1.0 / exit_vertices_.size() * 1.0 / nr_paths_leading_to_vertex;
      }
      joint_distribution_orig.insert(VertexPathProbaMap::value_type(vertex_path, proba));
      joint_distribution_updated.insert(VertexPathProbaMap::value_type(vertex_path, proba));
      cout << "Before update: path id " << vertex_path.path_ << " exit vertex " << vertex_path.vertex_->name() << " proba " << proba << endl;
    }
  }

  // Update joint distribution (Jeffrey's rule)

  for (VertexPathProbaMap::iterator vertex_path_map_it = joint_distribution_orig.begin(); vertex_path_map_it != joint_distribution_orig.end(); ++vertex_path_map_it) {
    VertexPath vertex_path;
    vertex_path.vertex_ = vertex_path_map_it->first.vertex_;
    vertex_path.path_ = vertex_path_map_it->first.path_;
    double proba = 0.0;
    if (marginal_distribution[vertex_path_map_it->first.path_] > 0) {
      proba = vertex_path_map_it->second * soft_evidence[vertex_path_map_it->first.path_] / marginal_distribution[vertex_path_map_it->first.path_];
      joint_distribution_updated[vertex_path] = proba;
    }
    cout << "After update: path id " << vertex_path.path_ << " exit vertex " << vertex_path.vertex_->name() << " proba " << proba << endl;
  }

  // Proba for each exit vertex
  double sum_of_probas_exit = 0.0;
  for (VertexPathProbaMap::const_iterator vertex_path_map_it = joint_distribution_updated.begin(); vertex_path_map_it != joint_distribution_updated.end(); ++vertex_path_map_it) {
    exit_vertex_proba_map->insert(VertexProbaMap::value_type(vertex_path_map_it->first.vertex_, 0.0));
    (*exit_vertex_proba_map)[vertex_path_map_it->first.vertex_] += vertex_path_map_it->second;
    sum_of_probas_exit += vertex_path_map_it->second;
  }

  // Normalize probas
  if (sum_of_probas_exit > 0.0) {
    for (VertexProbaMap::iterator vertex_map_it = exit_vertex_proba_map->begin(); vertex_map_it != exit_vertex_proba_map->end(); ++vertex_map_it) {
      vertex_map_it->second = vertex_map_it->second / sum_of_probas_exit;
      cout << "Vehicle has destination " << vertex_map_it->first->name() << " with probability " << vertex_map_it->second << endl;
    }
  }

}

void ObstaclePredictor::findVerticesLeadingToExitVertices() {

  for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths_.begin(); path_map_it != intersection_paths_.end(); ++path_map_it) {
    TRndfVertexSet* entrance_vertices = new TRndfVertexSet;
    exit_vertex_entrance_vertices_map_.insert(VertexVerticesMap::value_type(path_map_it->second->exit_vertex_, entrance_vertices));
    exit_vertex_entrance_vertices_map_[path_map_it->second->exit_vertex_]->insert(path_map_it->second->entrance_vertex_);
  }

}

void ObstaclePredictor::findReachableVerticesFromEntranceVertex() {

  // Look in every path in intersection if the entrance vertex corresponds
  for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths_.begin(); path_map_it != intersection_paths_.end(); ++path_map_it) {
    TRndfVertexSet* exit_vertices = new TRndfVertexSet;
    entrance_vertex_exit_vertices_map_.insert(VertexVerticesMap::value_type(path_map_it->second->entrance_vertex_, exit_vertices));
    entrance_vertex_exit_vertices_map_[path_map_it->second->entrance_vertex_]->insert(path_map_it->second->exit_vertex_);
  }

}

void ObstaclePredictor::findEdgesLeadingToVertices(EdgeVerticesMap reachable_vertices_from_edges, VertexEdgesMap* edges_leading_to_vertices) {

  for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {
    TRndfEdgeSet* edges = new TRndfEdgeSet;
    RndfVertex* vertex = *vertex_it;
    edges_leading_to_vertices->insert(VertexEdgesMap::value_type(vertex, edges));
  }

  for (EdgeVerticesMap::const_iterator edge_map_it = reachable_vertices_from_edges.begin(); edge_map_it != reachable_vertices_from_edges.end(); ++edge_map_it) {

    RndfEdge* edge = edge_map_it->first;
    TRndfVertexSet* vertices = edge_map_it->second;
    for (TRndfVertexSet::const_iterator vertex_it = vertices->begin(); vertex_it != vertices->end(); ++vertex_it) {
      (*edges_leading_to_vertices)[*vertex_it]->insert(edge);
    }

  }

}

void ObstaclePredictor::findReachableVerticesFromEdge(RndfEdge* edge, TRndfVertexSet* reachable_entrance_vertices) {

  RndfVertex* connectedVertex = edge->toVertex();
  TRndfEdgeSet out_edges;

  // If the vertex is not an entrance to the intersection, explore next edges
  if (entrance_vertices_.find(connectedVertex) == entrance_vertices_.end()) {
    out_edges = connectedVertex->getOutEdges();
    for (TRndfEdgeSet::const_iterator branch_it = out_edges.begin(); branch_it != out_edges.end(); ++branch_it) {
      if (distanceBetweenPositions(intersection_->center().x(), intersection_->center().y(), (*branch_it)->toVertex()->x(), (*branch_it)->toVertex()->y())
          < intersection_->getRadius() + TRIGGER_DIST_APPROACH_INTERSECTION) {
        findReachableVerticesFromEdge(*branch_it, reachable_entrance_vertices);
      }
    }
  }
  else {
    reachable_entrance_vertices->insert(connectedVertex);
    //cout << "edge " << edge->name() << ", insert vertex " << connectedVertex->name() << endl;
  }

}

void ObstaclePredictor::findReachableVerticesFromEdges(EdgeProbaMap edge_proba_map, EdgeVerticesMap* reachable_vertices_from_edges) {

  for (EdgeProbaMap::iterator edge_map_it = edge_proba_map.begin(); edge_map_it != edge_proba_map.end(); ++edge_map_it) {
    RndfEdge* edge = edge_map_it->first;
    TRndfVertexSet* reachable_entrance_vertices = new TRndfVertexSet;
    findReachableVerticesFromEdge(edge, reachable_entrance_vertices);

    //cout << "edge " << edge->name() << " has " << reachable_entrance_vertices->size() << " reachable vertices" << endl;

    reachable_vertices_from_edges->insert(EdgeVerticesMap::value_type(edge, reachable_entrance_vertices));
  }

}

void ObstaclePredictor::insertAdjacentEdges(Vehicle* vehicle, RndfEdge* edge, EdgeProbaMap* edge_proba_map, uint32_t side) {

  double proba = computeProbaVehicleOnEdge(vehicle, edge);
  edge_proba_map->insert(EdgeProbaMap::value_type(edge, proba));
  //cout << "Consider edge: " << edge->name() << ", proba = " << proba << "side=" << side << endl;

  TRndfEdgeSet edges;
  if (side == 1) { // Left
    edges = edge->getLeftEdges();
  }
  else if (side == 2) { // Right
    edges = edge->getRightEdges();
  }
  else if (side != 0) {
    cout << "Error, side has to be 0 or 1 or 2" << endl;
  }

  if ((side != 0) && (edges.size() > 0)) {

    //double proba_on_edge = 0.0;
    RndfEdge* closest_edge = *(edges.begin());
    //double max_proba_on_edge = computeProbaVehicleOnEdge(vehicle, closest_edge);
    double smallest_dist = vehicle->distToEdge(closest_edge);

    for (TRndfEdgeSet::iterator edge_it = edges.begin(); edge_it != edges.end(); ++edge_it) {
      // Select closest edge of the lane
      //proba_on_edge = computeProbaVehicleOnEdge(vehicle, *edge_it);
      double dist = vehicle->distToEdge(*edge_it);
      if (dist < smallest_dist) {
        smallest_dist = dist;
        closest_edge = *edge_it;
      }
      //if (proba_on_edge > max_proba_on_edge) {
      //max_proba_on_edge = proba_on_edge;
      //closest_edge = *edge_it;
      //}

    }
    insertAdjacentEdges(vehicle, closest_edge, edge_proba_map, side);
  }

}

void ObstaclePredictor::insertAdjacentEdgesInIntersection(Vehicle* vehicle, RndfEdge* matched_edge, EdgeProbaMap* edge_proba_map) {

  const TRndfEdgeSet& intersection_edges = matched_edge->getIntersection()->getEdges();
  TRndfVertexSet entrance_vertices;
  TRndfVertexSet exit_vertices;
  VertexVertexPathMap intersection_paths;

  // Find entrance and exit vertices

  for (TRndfEdgeSet::const_iterator edge_it = intersection_edges.begin(); edge_it != intersection_edges.end(); ++edge_it) {

    //if (!(*edge_it)->isLaneChangeEdge()) {

    RndfVertex* connectedVertex;

    // Find entrance vertices
    if ((*edge_it)->isVirtualEdge()) {
      findEntranceVerticesOfVirtualEdge(*edge_it, &entrance_vertices, false);
    }
    else {
      connectedVertex = (*edge_it)->fromVertex();
      entrance_vertices.insert(connectedVertex);
    }

    // Find exit vertices
    if ((*edge_it)->isVirtualEdge()) {
      findExitVerticesOfVirtualEdge(*edge_it, &exit_vertices);
    }
    else {
      connectedVertex = (*edge_it)->toVertex();
      exit_vertices.insert(connectedVertex);
    }

    //}

  }

  // Display results
  //for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices.begin(); vertex_it != entrance_vertices.end(); ++vertex_it) {
  //cout << "Current intersection: Entrance vertex " << (*vertex_it)->name() << endl;
  //}
  //for (TRndfVertexSet::const_iterator vertex_it = exit_vertices.begin(); vertex_it != exit_vertices.end(); ++vertex_it) {
  //cout << "Current intersection: Exit vertex " << (*vertex_it)->name() << endl;
  //}

  // Find all possible paths in intersection
  PathSet path_set;
  for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices.begin(); vertex_it != entrance_vertices.end(); ++vertex_it) {
    IntersectionPath* path = new IntersectionPath(*vertex_it, NULL);
    path_set.insert(path);
    createPath(path, &path_set, exit_vertices);
  }
  for (PathSet::const_iterator path_it = path_set.begin(); path_it != path_set.end(); ++path_it) {
    VertexVertex entrance_vertex_exit_vertex;
    entrance_vertex_exit_vertex.entrance_vertex_ = (*path_it)->entrance_vertex_;
    entrance_vertex_exit_vertex.exit_vertex_ = (*path_it)->exit_vertex_;
    intersection_paths.insert(VertexVertexPathMap::value_type(entrance_vertex_exit_vertex, *path_it));
  }

  // Display results
  //for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths.begin(); path_map_it != intersection_paths.end(); ++path_map_it) {
  //cout << "Current intersection: Path " << endl;
  //cout << "Current intersection: Entrance vertex " << path_map_it->second->entrance_vertex_->name() << endl;
  //cout << "Current intersection: Exit vertex " << path_map_it->second->exit_vertex_->name() << endl;
  //cout << "Current intersection: Edges:";
  //for (TRndfEdgeSet::const_iterator edge_it = path_map_it->second->edges_.begin(); edge_it != path_map_it->second->edges_.end(); ++edge_it) {
  //cout << " " << (*edge_it)->name();
  //}
  //cout << endl;
  //}

  // Find the list of relevant edges (the edges the closest to vehicle in each path)


  for (VertexVertexPathMap::const_iterator path_map_it = intersection_paths.begin(); path_map_it != intersection_paths.end(); ++path_map_it) {
    // Calculate distance to each edge of the path and keep track of the closest edge
    //double proba_on_edge = 0.0;
    RndfEdge* closest_edge = *(path_map_it->second->edges().begin());
    double max_proba_on_edge = computeProbaVehicleOnEdge(vehicle, closest_edge);
    double smallest_dist = vehicle->distToEdge(closest_edge);
    for (TRndfEdgeSet::iterator edge_it = path_map_it->second->edges().begin(); edge_it != path_map_it->second->edges().end(); ++edge_it) {
      // Select closest edge of the lane
      //proba_on_edge = computeProbaVehicleOnEdge(vehicle, *edge_it);
      double dist = vehicle->distToEdge(*edge_it);
      if (dist < smallest_dist) {
        smallest_dist = dist;
        closest_edge = *edge_it;
        max_proba_on_edge = computeProbaVehicleOnEdge(vehicle, *edge_it);
      }
      //if (proba_on_edge > max_proba_on_edge) {
      //max_proba_on_edge = proba_on_edge;
      //closest_edge = *edge_it;
      //}
    }
    //cout << "Lane separation. Consider edge: " << closest_edge->name() << ", proba = " << max_proba_on_edge << endl;
    edge_proba_map->insert(EdgeProbaMap::value_type(closest_edge, max_proba_on_edge));
  }

}

double ObstaclePredictor::distanceBetweenPositions(double x1, double y1, double x2, double y2) {

  double diff_x = x1 - x2;
  double diff_y = y1 - y2;

  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

void ObstaclePredictor::vehicleHeadingToNextIntersection(Vehicle vehicle, RndfEdge* edge, bool* output, uint32_t* count) {

  (*count)++;

  double dist_max = intersection_->getRadius() + TRIGGER_DIST_APPROACH_INTERSECTION;

  double distance = distanceBetweenPositions(vehicle.xMatchedFrom(), vehicle.yMatchedFrom(), intersection_->center().x(), intersection_->center().y());

  if (entrance_vertices_.find(edge->toVertex()) != entrance_vertices_.end()) {
    *output = true;
  }
  else if ((*count > 50) || (distance > dist_max) || (edge->isVirtualEdge() && (edge->fromVertex()->isStopVertex()
      || edge->fromVertex()->isTrafficLightVertex()))) {
    //if (*count > 50) {
    //cout << "Not relevant because count > 50" << endl;
    //}
    //if (distance > dist_max) {
    //cout << "Not relevant because too far from intersection" << endl;
    //}
    //if ((edge->isVirtualEdge() && (edge->fromVertex()->isStopVertex() || edge->fromVertex()->isTrafficLightVertex()))) {
    //if (edge->fromVertex()->isStopVertex()) {
    //cout << "Not relevant because the next intersection is not the right one: vertex " << edge->fromVertex()->name() << " is a stop vertex" << endl;
    //}
    //if (edge->fromVertex()->isTrafficLightVertex()) {
    //cout << "Not relevant because the next intersection is not the right one: vertex " << edge->fromVertex()->name() << " is a traffic light vertex" << endl;
    //}
    //}
  }
  else {// if (distance<intersection->getRadius()+TRIGGER_DIST_APPROACH_INTERSECTION) {
    TRndfEdgeSet next_edges = edge->toVertex()->getOutEdges();
    if (next_edges.size() > 0) {
      for (TRndfEdgeSet::const_iterator branch_it = next_edges.begin(); branch_it != next_edges.end(); ++branch_it) {
        //cout << "Next test: " << (*branch_it)->name() << endl;
        vehicleHeadingToNextIntersection(vehicle, *branch_it, output, count);
      }
    }
  }

}

//HERECHANGESIGMAS
double ObstaclePredictor::computeProbaVehicleOnEdge(Vehicle* vehicle, RndfEdge* edge) {

  double diff_pos = vehicle->distToEdge(edge);
  double diff_angle = abs(vehicle->angleToEdge(edge));
  //double diff_angle = abs(vehicle->angleToEdge(edge))/(diff_pos/vehicle->speed());

  //double sigma_pos = 4.0;
  //double sigma_angle = M_PI/3.5;
  //double sigma_pos = 3.0;
  //double sigma_angle = M_PI/4.0;

  double sigma_pos;
  double sigma_angle;
  if (relevant_vehicles_in_intersection_.find(vehicle->id()) != relevant_vehicles_in_intersection_.end()) {// Necessary for now because in reality vehicles do not follow predifined trajectories
    sigma_pos = 1.5;
    sigma_angle = M_PI / 6.0;
  }
  else {
    sigma_pos = 1.0;//1.5;1.5 ok
    sigma_angle = M_PI / 6.0;//6.0;6.0 ok, 4.0 ok (demo)
  }

  //double err_pos = diff_pos*diff_pos / (2.0*sigma_pos*sigma_pos);
  //double err_angle = diff_angle*diff_angle / (2.0*sigma_angle*sigma_angle);

  //double proba = exp(-0.5*(err_pos + err_angle));
  double proba = gaussian_func2(diff_pos, 0.0, sigma_pos, diff_angle, 0.0, sigma_angle);

  //cout << "Edge " << edge->name() << ", proba=" << proba << ", diff_pos=" << diff_pos << ", diff_angle=" << diff_angle << ", err_pos=" << err_pos << ", err_angle=" << err_angle <<endl;

  return proba;

}

//double ObstaclePredictor::computeWeightEntrance(double distance_to_entrance, double distance_to_exit, double angle, double velocity) {

////double ratio_distance = distance_to_entrance / (distance_to_entrance + distance_to_exit);
////double sigma_ratio_distance = 0.15;

////double ratio_distance = intersection->getRadius() / distance_to_exit;
////double sigma_ratio_distance = 1.0;

//double ratio_distance = distance_to_entrance;
////double sigma_ratio_distance = 2.0;//ok sans l'angle
//double sigma_ratio_distance = 3.0;

//double sigma_angle = M_PI / 4.0;
////double sigma_velocity = 2.0;

//double err_ratio_distance = ratio_distance*ratio_distance / (sigma_ratio_distance*sigma_ratio_distance);
//double err_angle = angle*angle / (sigma_angle*sigma_angle);//attention, penser à remettre à 0 ? et à remettre la vitesse
//double err_velocity = 0.0;//velocity*velocity / (sigma_velocity*sigma_velocity);// remarque : velocity à prendre en compte pour other paths!

//double weight = exp(-0.5*(err_ratio_distance + err_angle + err_velocity));

//return weight;

//}


void ObstaclePredictor::findEntranceVerticesOfVirtualEdge(RndfEdge* edge, TRndfVertexSet* entrance_vertices, bool check_if_all_way_stop_intersection) {

  RndfVertex* connectedVertex;
  TRndfEdgeSet in_edges;

  if (edge->isVirtualEdge()) {
    in_edges = edge->fromVertex()->getInEdges();
    for (TRndfEdgeSet::iterator it = in_edges.begin(); it != in_edges.end();) {
      //cout << "edge " << (*it)->name() << endl;
      TRndfEdgeSet::iterator erase_element = it++;
      if ((*erase_element)->isLaneChangeEdge()) {
        //cout << "erased edge " << (*erase_element)->name() << endl;
        in_edges.erase(erase_element++);
      }
    }
    for (TRndfEdgeSet::const_iterator branch_it = in_edges.begin(); branch_it != in_edges.end(); ++branch_it) {
      //if (!(*branch_it)->isLaneChangeEdge()) {
      findEntranceVerticesOfVirtualEdge(*branch_it, entrance_vertices, check_if_all_way_stop_intersection);
      //}
    }
  }
  else {
    connectedVertex = edge->toVertex();
    entrance_vertices->insert(connectedVertex);
    // Check if the intersection is all-way-stop
    if (check_if_all_way_stop_intersection && !connectedVertex->isStopVertex()) {
      is_all_way_stop_intersection_ = false;
    }
  }

}

void ObstaclePredictor::findExitVerticesOfVirtualEdge(RndfEdge* edge, TRndfVertexSet* exit_vertices) {

  RndfVertex* connectedVertex;
  TRndfEdgeSet out_edges;

  if (edge->isVirtualEdge()) {
    out_edges = edge->toVertex()->getOutEdges();
    for (TRndfEdgeSet::iterator it = out_edges.begin(); it != out_edges.end();) {
      //cout << "edge " << (*it)->name() << endl;
      TRndfEdgeSet::iterator erase_element = it++;
      if ((*erase_element)->isLaneChangeEdge()) {
        //cout << "erased edge " << (*erase_element)->name() << endl;
        out_edges.erase(erase_element++);
      }
    }
    for (TRndfEdgeSet::const_iterator branch_it = out_edges.begin(); branch_it != out_edges.end(); ++branch_it) {
      //if (!(*branch_it)->isLaneChangeEdge()) {
      findExitVerticesOfVirtualEdge(*branch_it, exit_vertices);
      //}
    }
  }
  else {
    connectedVertex = edge->fromVertex();
    exit_vertices->insert(connectedVertex);
  }

}

void ObstaclePredictor::createPath(IntersectionPath* current_path, PathSet* path_set, TRndfVertexSet exit_vertices) {

  RndfVertex* next_vertex = current_path->exit_vertex_;
  RndfEdge* next_edge;
  TRndfEdgeSet out_edges;
  uint32_t nr_created_paths = 0;
  uint32_t nr_paths = 0;
  IntersectionPath* next_path;

  //if (next_vertex != NULL) {
  //cout << "entrance " << current_path->entrance_vertex_->name() << ", next vertex is " << next_vertex->name() << endl;
  //}
  //cout << "next vertex is " << next_vertex->name() << endl;

  while ((exit_vertices.find(next_vertex) == exit_vertices.end()) && current_path->entrance_vertex_ != next_vertex) {

    if (next_vertex != NULL) {
      out_edges = next_vertex->getOutEdges();
    }
    else {
      out_edges = current_path->entrance_vertex_->getOutEdges();
    }

    for (TRndfEdgeSet::iterator it = out_edges.begin(); it != out_edges.end();) {
      //cout << "edge " << (*it)->name() << endl;
      TRndfEdgeSet::iterator erase_element = it++;
      if ((*erase_element)->isLaneChangeEdge()) {
        //cout << "erased edge " << (*erase_element)->name() << endl;
        out_edges.erase(erase_element++);
      }
    }

    if (out_edges.size() == 1) {
      next_edge = *(out_edges.begin());
      current_path->edges_.insert(next_edge);
      next_vertex = next_edge->toVertex();
      current_path->exit_vertex_ = next_vertex;
      //cout << "1 next edge " << next_edge->name() << ", next vertex" << next_vertex->name() << endl;
    }
    else {
      nr_paths = out_edges.size();
      nr_created_paths = 0;
      for (TRndfEdgeSet::const_iterator branch_it = out_edges.begin(); branch_it != out_edges.end(); ++branch_it) {
        //if (!(*branch_it)->isLaneChangeEdge()) {
        //cout << "2 next edge " << (*branch_it)->name() << ", next vertex" << (*branch_it)->toVertex()->name() << endl;
        next_vertex = (*branch_it)->toVertex();
        current_path->exit_vertex_ = next_vertex;
        if (nr_created_paths < nr_paths - 1) {
          nr_created_paths++;
          next_path = new IntersectionPath(current_path);
          path_set->insert(next_path);
        }
        else {
          next_path = current_path;
        }
        next_path->edges_.insert(*branch_it);
        createPath(next_path, path_set, exit_vertices);
        //}
        //else {
        //if (nr_created_paths < nr_paths-1) {
        //nr_created_paths++;
        //}
        //}
      }
    }

  }

  //if (current_path->entrance_vertex_ != next_vertex) {
  current_path->exit_vertex_ = next_vertex;
  //}
  //else {

  //}

}

double ObstaclePredictor::angleBetweenEdges(RndfEdge* edge_out, RndfEdge* edge_in) {

  double angle;

  angle = edge_out->getAngle() - edge_in->getAngle();
  if (angle > M_PI) {
    angle = angle - 2.0 * M_PI;
  }
  else if (angle < -M_PI) {
    angle = angle + 2.0 * M_PI;
  }

  return angle;

}

void ObstaclePredictor::createSegmentsIds() {

  RndfEdge* in_edge = NULL;
  RndfEdge* in_edge_other = NULL;
  RndfEdge* out_edge = NULL;
  RndfEdge* out_edge_other = NULL;

  for (TRndfVertexSet::const_iterator vertex_it = entrance_vertices_.begin(); vertex_it != entrance_vertices_.end(); ++vertex_it) {

    if (vertex_segment_map_.find(*vertex_it) == vertex_segment_map_.end()) {

      TRndfVertexSet* vertices_set = new TRndfVertexSet;

      vertex_segment_map_[*vertex_it] = nr_possible_segments_;
      vertices_set->insert(*vertex_it);

      //// Set segment id for adjacent vertices
      //TRndfEdgeSet adjacent_edges;
      //RndfVertex* adjacent_vertex;
      //in_edge = *((*vertex_it)->getInEdges().begin());
      //adjacent_edges = in_edge->getLeftEdges();
      //for (TRndfEdgeSet::const_iterator adjacent_edge = adjacent_edges.begin(); adjacent_edge != adjacent_edges.end(); ++adjacent_edge) {
      //adjacent_vertex = (*adjacent_edge)->toVertex();
      //vertex_segment_map_[adjacent_vertex] = nr_possible_segments_;
      //vertices_set->insert(adjacent_vertex);
      //}

      in_edge = *((*vertex_it)->getInEdges().begin());

      // Check other entrance vertices
      for (TRndfVertexSet::const_iterator other_vertex_it = entrance_vertices_.begin(); other_vertex_it != entrance_vertices_.end(); ++other_vertex_it) {
        in_edge_other = *((*other_vertex_it)->getInEdges().begin());
        TRndfEdgeSet checked_edges;
        uint32_t side;
        if (areInSameSegment(in_edge, in_edge_other, &checked_edges, &side)) {
          cout << "edge " << in_edge->name() << " and edge " << in_edge_other->name() << " are in same segment, side=" << side << endl;
          vertex_segment_map_[*other_vertex_it] = nr_possible_segments_;
          vertices_set->insert(*other_vertex_it);
          // Store information on sides
          VertexVertex vertex_vertex;
          vertex_vertex.entrance_vertex_ = *vertex_it;
          vertex_vertex.exit_vertex_ = *other_vertex_it;
          side_vertices_in_same_segment_[vertex_vertex] = side;
          vertex_vertex.entrance_vertex_ = *other_vertex_it;
          vertex_vertex.exit_vertex_ = *vertex_it;
          if (side == 1) {
            side_vertices_in_same_segment_[vertex_vertex] = 2;
          }
          else if (side == 2) {
            side_vertices_in_same_segment_[vertex_vertex] = 1;
          }
          else {
            side_vertices_in_same_segment_[vertex_vertex] = side;
          }
        }
      }

      // Check exit vertices
      for (TRndfVertexSet::const_iterator other_vertex_it = exit_vertices_.begin(); other_vertex_it != exit_vertices_.end(); ++other_vertex_it) {
        out_edge_other = *((*other_vertex_it)->getOutEdges().begin());
        TRndfEdgeSet checked_edges;
        uint32_t side;
        if (areInSameSegment(in_edge, out_edge_other, &checked_edges, &side)) {
          cout << "edge " << in_edge->name() << " and edge " << out_edge_other->name() << " are in same segment" << endl;
          vertex_segment_map_[*other_vertex_it] = nr_possible_segments_;
          vertices_set->insert(*other_vertex_it);
          // Store information on sides
          VertexVertex vertex_vertex;
          vertex_vertex.entrance_vertex_ = *vertex_it;
          vertex_vertex.exit_vertex_ = *other_vertex_it;
          side_vertices_in_same_segment_[vertex_vertex] = side;
          vertex_vertex.entrance_vertex_ = *other_vertex_it;
          vertex_vertex.exit_vertex_ = *vertex_it;
          if (side == 1) {
            side_vertices_in_same_segment_[vertex_vertex] = 2;
          }
          else if (side == 2) {
            side_vertices_in_same_segment_[vertex_vertex] = 1;
          }
          else {
            side_vertices_in_same_segment_[vertex_vertex] = side;
          }
        }
      }

      segment_vertices_map_[nr_possible_segments_] = vertices_set;

      nr_possible_segments_++;
      nr_possible_entrance_segments_++;
    }

  }

  for (TRndfVertexSet::const_iterator vertex_it = exit_vertices_.begin(); vertex_it != exit_vertices_.end(); ++vertex_it) {

    if (vertex_segment_map_.find(*vertex_it) == vertex_segment_map_.end()) {

      TRndfVertexSet* vertices_set = new TRndfVertexSet;

      vertex_segment_map_[*vertex_it] = nr_possible_segments_;
      vertices_set->insert(*vertex_it);

      // Check other exit vertices
      out_edge = *((*vertex_it)->getOutEdges().begin());
      for (TRndfVertexSet::const_iterator other_vertex_it = exit_vertices_.begin(); other_vertex_it != exit_vertices_.end(); ++other_vertex_it) {
        out_edge_other = *((*other_vertex_it)->getOutEdges().begin());
        TRndfEdgeSet checked_edges;
        uint32_t side;
        if (areInSameSegment(out_edge, out_edge_other, &checked_edges, &side)) {
          vertex_segment_map_[*other_vertex_it] = nr_possible_segments_;
          vertices_set->insert(*other_vertex_it);
          // Store information on sides
          VertexVertex vertex_vertex;
          vertex_vertex.entrance_vertex_ = *vertex_it;
          vertex_vertex.exit_vertex_ = *other_vertex_it;
          side_vertices_in_same_segment_[vertex_vertex] = side;
          vertex_vertex.entrance_vertex_ = *other_vertex_it;
          vertex_vertex.exit_vertex_ = *vertex_it;
          if (side == 1) {
            side_vertices_in_same_segment_[vertex_vertex] = 2;
          }
          else if (side == 2) {
            side_vertices_in_same_segment_[vertex_vertex] = 1;
          }
          else {
            side_vertices_in_same_segment_[vertex_vertex] = side;
          }
        }
      }

      // Find opposite vertices
      for (TRndfVertexSet::const_iterator other_vertex_it = entrance_vertices_.begin(); other_vertex_it != entrance_vertices_.end(); ++other_vertex_it) {
        in_edge_other = *((*other_vertex_it)->getInEdges().begin());
        TRndfEdgeSet checked_edges;
        uint32_t side;
        if (areInSameSegment(out_edge, in_edge_other, &checked_edges, &side)) {
          vertex_segment_map_[*other_vertex_it] = nr_possible_segments_;
          vertices_set->insert(*other_vertex_it);
          // Store information on sides
          VertexVertex vertex_vertex;
          vertex_vertex.entrance_vertex_ = *vertex_it;
          vertex_vertex.exit_vertex_ = *other_vertex_it;
          side_vertices_in_same_segment_[vertex_vertex] = side;
          vertex_vertex.entrance_vertex_ = *other_vertex_it;
          vertex_vertex.exit_vertex_ = *vertex_it;
          if (side == 1) {
            side_vertices_in_same_segment_[vertex_vertex] = 2;
          }
          else if (side == 2) {
            side_vertices_in_same_segment_[vertex_vertex] = 1;
          }
          else {
            side_vertices_in_same_segment_[vertex_vertex] = side;
          }
        }
      }

      segment_vertices_map_[nr_possible_segments_] = vertices_set;

      nr_possible_segments_++;
      //nr_possible_exit_segments_++;
    }

  }

}

bool ObstaclePredictor::areInSameSegment(RndfEdge* edge_1, RndfEdge* edge_2, TRndfEdgeSet* checked_edges, uint32_t* side) {

  bool are_in_same_segment = false;

  if (edge_1 == edge_2) {
    are_in_same_segment = true;
    *side = 0;
  }
  else if (edge_1->hasLeftEdge(edge_2)) {
    are_in_same_segment = true;
    *side = 1;
  }
  else if (edge_1->hasRightEdge(edge_2)) {
    are_in_same_segment = true;
    *side = 2;
  }
  else if (edge_1->hasLeftOppositeEdge(edge_2)) {
    are_in_same_segment = true;
    *side = 3;
  }

  else {

    TRndfEdgeSet adjacent_edges;

    TRndfEdgeSet left_edges = edge_1->getLeftEdges();
    for (TRndfEdgeSet::const_iterator adjacent_edge = left_edges.begin(); adjacent_edge != left_edges.end(); ++adjacent_edge) {
      if (checked_edges->find(*adjacent_edge) == checked_edges->end()) {
        adjacent_edges.insert(*adjacent_edge);
        checked_edges->insert(*adjacent_edge);
      }
    }
    TRndfEdgeSet right_edges = edge_1->getRightEdges();
    for (TRndfEdgeSet::const_iterator adjacent_edge = right_edges.begin(); adjacent_edge != right_edges.end(); ++adjacent_edge) {
      if (checked_edges->find(*adjacent_edge) == checked_edges->end()) {
        adjacent_edges.insert(*adjacent_edge);
        checked_edges->insert(*adjacent_edge);
      }
    }
    TRndfEdgeSet left_opposite_edges = edge_1->getLeftOppositeEdges();
    for (TRndfEdgeSet::const_iterator adjacent_edge = left_opposite_edges.begin(); adjacent_edge != left_opposite_edges.end(); ++adjacent_edge) {
      if (checked_edges->find(*adjacent_edge) == checked_edges->end()) {
        adjacent_edges.insert(*adjacent_edge);
        checked_edges->insert(*adjacent_edge);
      }
    }

    for (TRndfEdgeSet::const_iterator adjacent_edge = adjacent_edges.begin(); adjacent_edge != adjacent_edges.end(); ++adjacent_edge) {
      if (areInSameSegment(*adjacent_edge, edge_2, checked_edges, side)) {
        are_in_same_segment = true;
      }
    }

  }

  return are_in_same_segment;

}

double ObstaclePredictor::gaussian_func(double val, double mean, double sigma) {

  double res = exp(-0.5 * (val - mean) * (val - mean) / (sigma * sigma));

  return res;
}

double ObstaclePredictor::gaussian_func2(double val1, double mean1, double sigma1, double val2, double mean2, double sigma2) {

  double res = exp(-0.5 * ((val1 - mean1) * (val1 - mean1) / (2.0 * sigma1 * sigma1) + (val2 - mean2) * (val2 - mean2) / (sigma2 * sigma2)));

  return res;
}
#endif

} // namespace vlr
