#ifndef RNDF_RNDF_SPOT_H
#define RNDF_RNDF_SPOT_H

#include <rndfWayPoint.h>

namespace dgc {

class rndf_spot {
 public:
  rndf_spot();
  ~rndf_spot();
  rndf_spot(const rndf_spot &s);
  rndf_spot& operator=(const rndf_spot &s);

  int lookup_spot_id(void) const;

  int num_waypoints(void) const { return (signed)waypoint_.size(); }
  rndf_waypoint *waypoint(int i) const;  // inlined below
  void insert_waypoint(int i, rndf_waypoint *w);
  void append_waypoint(rndf_waypoint *w);
  void delete_waypoint(int i);
  void clear_waypoints(void);

  float width(void) const { return width_; }
  void width(float w) { width_ = w; }

  rndf_spot *next(void) const { return next_; }
  rndf_spot *prev(void) const { return prev_; }

  rndf_waypoint *first_waypoint(void) const;
  rndf_waypoint *last_waypoint(void) const;
  class rndf_zone *parentzone(void) const { return parentzone_; }
  friend class rndf_zone;

 private:
  float width_;
  std::vector<rndf_waypoint *> waypoint_;
  rndf_spot *next_, *prev_;
  class rndf_zone *parentzone_;
  void parentzone(class rndf_zone *z) { parentzone_ = z; }
  void next(rndf_spot *n) { next_ = n; }
  void prev(rndf_spot *p) { prev_ = p; }
};

inline rndf_waypoint *rndf_spot::waypoint(int i) const
{
  if(i < 0 || i >= num_waypoints())
    dgc_die("Error: rndf_spot::waypoint : index out of range\n");
    return waypoint_[i];
}

inline rndf_waypoint *rndf_spot::first_waypoint(void) const
{
  return (num_waypoints() == 0) ? NULL : waypoint_[0];
}

inline rndf_waypoint *rndf_spot::last_waypoint(void) const
{
  return (num_waypoints() == 0) ? NULL : waypoint_[num_waypoints() - 1];
}

} // namespace dgc

#endif // RNDF_RNDF_SPOT_H
