#ifndef DGC_PLANNER_DATA_H
#define DGC_PLANNER_DATA_H

#include <rndf.h>

typedef struct {
  bool is_stop;
  double x, y;
  float theta, curvature, true_k;
  float min_v, max_v, dist_to_stop;
} bt_waypoint;

typedef struct {
  int num_waypoints(void) const { return (signed)waypoint.size(); }
  std::vector <bt_waypoint> waypoint;
  double length;
} bt_segment;

class planner_data : public dgc::rndf_data {
public:
  double cost;
  double dist_to_stop;
  planner_data *clone(void);
  bt_segment bt;
  std::vector <bt_segment> exit_bt;
};

#define PDATA(x) (static_cast<planner_data *>(x))

void add_waypoint_data(dgc::rndf_file *rndf);

#endif
