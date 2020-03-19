#ifndef RNDF_RNDF_ZONE_H
#define RNDF_RNDF_ZONE_H

#include <rndfSpot.h>

namespace dgc {

class rndf_zone {
 public:
  rndf_zone();
  ~rndf_zone();
  rndf_zone(const rndf_zone &z);
  rndf_zone& operator=(const rndf_zone &z);

  int lookup_zone_id(void) const;

  int num_perimeter_points(void) const { return (signed)perimeter_.size(); }
  rndf_waypoint *perimeter(int i) const;  // inlined below
  void insert_perimeter_point(int i, rndf_waypoint *w);
  void append_perimeter_point(rndf_waypoint *w);
  void delete_perimeter_point(int i);
  void clear_perimeter_points(void);

  int num_spots(void) const { return (signed)spot_.size(); }
  rndf_spot *spot(int i) const;  // inlined below
  void insert_spot(int i, rndf_spot *s);
  void append_spot(rndf_spot *s);
  void delete_spot(int i);
  void clear_spots(void);

  std::string name(void) const { return name_; }
  void name(std::string s) { name_ = s; }

  bool point_inside(double x, double y);
  bool car_inside(double x, double y, double theta,
      double width_buffer, double length_buffer);
  bool car_partially_inside(double x, double y, double theta,
          double width_buffer, double length_buffer);
  bool car_outside(double x, double y, double theta,
       double width_buffer, double length_buffer);

  rndf_waypoint *first_perimeter_point(void) const;
  rndf_waypoint *last_perimeter_point(void) const;
  rndf_spot *first_spot(void) const;
  rndf_spot *last_spot(void) const;
  rndf_zone *next(void) const { return next_; }
  rndf_zone *prev(void) const { return prev_; }
  class rndf_file *parentrndf(void) const { return parentrndf_; }
  friend class rndf_file;

 private:
  std::string name_;
  std::vector<rndf_waypoint *> perimeter_;
  std::vector<rndf_spot *> spot_;
  rndf_zone *next_, *prev_;
  class rndf_file *parentrndf_;
  void parentrndf(class rndf_file *r) { parentrndf_ = r; }
  void next(rndf_zone *n) { next_ = n; }
  void prev(rndf_zone *p) { prev_ = p; }
};

inline rndf_waypoint *rndf_zone::perimeter(int i) const
{
  if(i < 0 || i >= num_perimeter_points())
    dgc_die("Error: rndf_zone::perimeter : index out of range\n");
  return perimeter_[i];
}

inline rndf_spot *rndf_zone::spot(int i) const
{
  if(i < 0 || i >= num_spots())
    dgc_die("Error: rndf_zone::spot : index out of range\n");
  return spot_[i];
}

inline rndf_waypoint *rndf_zone::first_perimeter_point(void) const
{
  return (num_perimeter_points() == 0) ? NULL : perimeter_[0];
}

inline rndf_waypoint *rndf_zone::last_perimeter_point(void) const
{
  return (num_perimeter_points() == 0) ? NULL :
    perimeter_[num_perimeter_points() - 1];
}

inline rndf_spot *rndf_zone::first_spot(void) const
{
  return (num_spots() == 0) ? NULL : spot_[0];
}

inline rndf_spot *rndf_zone::last_spot(void) const
{
  return (num_spots() == 0) ? NULL : spot_[num_spots() - 1];
}

} // namespace dgc

#endif // RNDF_RNDF_ZONE_H
