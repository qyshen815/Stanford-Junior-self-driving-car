#ifndef RNDF_LOOKUP_H
#define RNDF_LOOKUP_H

#include <roadrunner.h>
#include <rndf.h>
#include <kdtree.h>
#include "vectors.h"

class RndfLookup {
 public:	
  int rndf_count, true_rndf_count;

  RndfLookup(dgc::rndf_file *rndf, double closeToLaneChange,
	     double closeToNormalLane);
  ~RndfLookup();
  void update_localize_offset(const Vec2 &newOffset);
  bool closeToRoad(const Vec2 &p, Vec2 *roadVec);
  bool closeToRoad(const Vec2 &p, double *road_angle=NULL);


 private:
  double closeToLaneChange; 
  double closeToNormalLane;
  double maxRng;
  dgc::rndf_file *rndf;

  dgc_kdtree_p rndf_kdtree;
  dgc::rndf_waypoint **rndf_wp;
  Vec2 locOffset;

  int max_waypoints;
  double *rndf_x, *rndf_y, *rndf_heading;
  int *is_exit;
  void resize_wp_memory(int num_waypoints);
  void build_rndf_kdtree();
};
#endif
