#ifndef DGC_DP_H
#define DGC_DP_H

#include <rndf.h>
#include <vector>

class global_plan {
 public:
  dgc::rndf_waypoint *waypoint(int i) const { return waypoint_[i]; }
  int num_waypoints(void) const { return (signed)waypoint_.size(); }
  void append_waypoint(dgc::rndf_waypoint *w) { waypoint_.push_back(w); }
  void clear(void) { waypoint_.clear(); }
 private:
  std::vector<dgc::rndf_waypoint *> waypoint_;
};

void initialize_dp(dgc::rndf_file *rndf, double lane_change_pr);

void do_dp(dgc::rndf_waypoint *goal);

void draw_dp(dgc::rndf_file *rndf, double origin_x, double origin_y);

global_plan *plan_from(dgc::rndf_waypoint *src);

void draw_global_plan(global_plan *p, double origin_x, double origin_y);

#endif
