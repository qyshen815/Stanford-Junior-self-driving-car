#ifndef DGC_RNDF_H
#define DGC_RNDF_H

  // This is just a compatibility header ...

#include <rndfRoadNetwork.h>

namespace dgc {

inline bool is_left_lane_change(rndf_waypoint *w, int i) {
  return w->is_left_lane_change(i);
}

inline rndf_waypoint* left_lane(rndf_waypoint *w) {
  return w->left_lane();
}

inline bool is_right_lane_change(rndf_waypoint *w, int i) {
  return w->is_right_lane_change(i);
}

inline rndf_waypoint *right_lane(rndf_waypoint *w) {
  return w->right_lane();
}

}

#endif
