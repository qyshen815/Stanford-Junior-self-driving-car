#ifndef RNDF_LOOKUP_H
#define RNDF_LOOKUP_H

#include <roadrunner.h>
#include <rndf.h>
#include <kdtree.h>

class RndfLookup {
 public:	
  RndfLookup(dgc::rndf_file *rndf);
  ~RndfLookup();
  double road_angle(double x, double y);

 private:
  dgc_kdtree_p rndf_kdtree;
  dgc::rndf_waypoint **rndf_wp;

  int max_waypoints;
  double *rndf_x, *rndf_y;
  int *is_exit;
  void resize_wp_memory(int num_waypoints);
  void build_rndf_kdtree(dgc::rndf_file *rndf);
};
#endif
