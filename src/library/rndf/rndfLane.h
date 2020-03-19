#ifndef RNDF_RNDF_LANE_H
#define RNDF_RNDF_LANE_H

#include <rndfWayPoint.h>

namespace dgc {

class rndf_waypoint;

typedef enum { unknown, double_yellow, solid_white, broken_white } rndf_linetype;
typedef enum { car_lane, bike_lane } rndf_lanetype;

class rndf_lane {
 public:
  rndf_lane();
  ~rndf_lane();
  rndf_lane(const rndf_lane &l);
  rndf_lane& operator=(const rndf_lane &l);

  int lookup_lane_id(void) const;

  int num_waypoints(void) const { return (signed)waypoint_.size(); }
  rndf_waypoint *waypoint(int i) const;    // defined inline below

  int num_original_waypoints(void) const { return (signed)original_waypoint_.size(); }
  rndf_waypoint *original_waypoint(int i) const;  // defined inline below

  void insert_waypoint(int i, rndf_waypoint *w);
  void append_waypoint(rndf_waypoint *w);
  void delete_waypoint(int i);
  void clear_waypoints(void);

  void width(float w) { width_ = w; }
  float width(void) const { return width_; }
  void type(rndf_lanetype t) { type_ = t; }
  rndf_lanetype type(void) const { return type_; }

  void left_boundary(rndf_linetype t) { left_boundary_ = t; }
  rndf_linetype left_boundary(void) const { return left_boundary_; }

  void right_boundary(rndf_linetype t) { right_boundary_ = t; }
  rndf_linetype right_boundary(void) const { return right_boundary_; }

  rndf_lane *next(void) const { return next_; }
  rndf_lane *prev(void) const { return prev_; }

  rndf_waypoint *first_waypoint(void) const; // defined inline below
  rndf_waypoint *last_waypoint(void) const;  // defined inline below
  class rndf_segment *parentsegment(void) const { return parentsegment_; }
  friend class rndf_segment;
  friend class rndf_waypoint;

  void print(char *name) const;

 private:
  float width_;
  rndf_lanetype type_;
  rndf_linetype left_boundary_, right_boundary_;
  std::vector<rndf_waypoint *> waypoint_;
  std::vector<rndf_waypoint *> original_waypoint_;
  class rndf_segment *parentsegment_;
  rndf_lane *next_, *prev_;
  void parentsegment(class rndf_segment *s) { parentsegment_ = s; }
  void next(rndf_lane *n) { next_ = n; }
  void prev(rndf_lane *p) { prev_ = p; }
};

inline rndf_waypoint *rndf_lane::waypoint(int i) const
{
  if(i < 0 || i >= num_waypoints())
    dgc_die("Error: rndf_lane::waypoint : index out of range\n");
  return waypoint_[i];
}

inline rndf_waypoint *rndf_lane::original_waypoint(int i) const
{
  if(i < 0 || i >= num_original_waypoints())
    dgc_die("Error: rndf_lane::original_waypoint : index out of range\n");
  return original_waypoint_[i];
}

inline rndf_waypoint *rndf_lane::first_waypoint(void) const
{
  return (num_waypoints() == 0) ? NULL : waypoint_[0];
}

inline rndf_waypoint *rndf_lane::last_waypoint(void) const
{
  return (num_waypoints() == 0) ? NULL : waypoint_[num_waypoints() - 1];
}

} // namespace dgc

#endif // RNDF_RNDF_LANE_H
