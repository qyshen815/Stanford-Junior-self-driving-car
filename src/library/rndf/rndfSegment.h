#ifndef RNDF_RNDF_SEGMENT_H
#define RNDF_RNDF_SEGMENT_H

#include <vector>
#include <string>

#include <rndfLane.h>

namespace dgc {

class rndf_file;
class rndf_lane;
class rndf_crosswalk;

class rndf_segment {

 public:
  rndf_segment();
  ~rndf_segment();
  rndf_segment(const rndf_segment &s);
  rndf_segment& operator=(const rndf_segment &s);

  int lookup_segment_id(void) const;

  int num_lanes(void) const { return (signed)lane_.size(); }
  rndf_lane *lane(int i) const;  // defined inline below
  void insert_lane(int i, rndf_lane *l);
  void append_lane(rndf_lane *l);
  void delete_lane(int i);
  void clear_lanes(void);

  int num_crosswalks(void) const { return (signed)crosswalk_.size(); }
  rndf_crosswalk *crosswalk(int i) const;  // defined inline below
  void insert_crosswalk(int i, rndf_crosswalk *c);
  void append_crosswalk(rndf_crosswalk *c);
  void delete_crosswalk(int i);
  void clear_crosswalks(void);

  std::string name(void) const { return name_; }
  void name(std::string s) { name_ = s; fprintf(stderr, "set name to %s\n", name_.c_str());}

  rndf_segment *next(void) const { return next_; }
  rndf_segment *prev(void) const { return prev_; }

  rndf_lane *first_lane(void) const;
  rndf_lane *last_lane(void) const;
  rndf_file* parentrndf(void) const { return parentrndf_; }
//  friend class rndf_file;

  void next(rndf_segment *n) { next_ = n; }
  void prev(rndf_segment *p) { prev_ = p; }
  void parentrndf(rndf_file* r) { parentrndf_ = r; }
  std::string& name() {return name_;}

  double& speed_limit() {return speed_limit_;}
  void speed_limit(const double& speed_limit) {speed_limit_ = speed_limit;}

private:
  rndf_segment *next_, *prev_;
  std::string name_;
  double speed_limit_; // speed limit in m/s
  std::vector<rndf_lane *> lane_;
  std::vector<rndf_crosswalk *> crosswalk_;
  rndf_file* parentrndf_;
};

inline rndf_lane *rndf_segment::lane(int i) const
{
  if(i < 0 || i >= num_lanes())
    dgc_die("Error: rndf_segment::lane : index out of range\n");
  return lane_[i];
}

inline rndf_lane *rndf_segment::first_lane(void) const
{
  return (num_lanes() == 0) ? NULL : lane_[0];
}

inline rndf_lane *rndf_segment::last_lane(void) const
{
  return (num_lanes() == 0) ? NULL : lane_[num_lanes() - 1];
}

inline rndf_crosswalk *rndf_segment::crosswalk(int i) const
{
  if(i < 0 || i >= num_crosswalks())
    dgc_die("Error: rndf_segment::crosswalk : index out of range\n");
  return crosswalk_[i];
}

} // namespace dgc

#endif // RNDF_RNDF_SEGMENT_H
