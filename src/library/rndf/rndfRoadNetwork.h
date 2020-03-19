#ifndef RNDF_RNDF_ROADNETWORK_H
#define RNDF_RNDF_ROADNETWORK_H

#include <vector>
#include <map>
#include <string>

#include <roadrunner.h>

#include <rndfSegment.h>
#include <rndfZone.h>
#include <rndfCrossWalk.h>
#include <rndfTrafficLight.h>

#define     RNDF_LIBRARY_VERSION       1.3

namespace dgc {

class rndf_file {
 public:
  rndf_file();
  ~rndf_file();
  rndf_file(const rndf_file &r);
  rndf_file& operator=(const rndf_file &r);

  int save(char *filename, bool save_as_srndf = false);
  int load(char *filename);
  void print();

  void tag_uturns(void);

  int num_segments(void) const { return (signed)segment_.size(); }
  rndf_segment *segment(int i) const;  // inlined below
  void insert_segment(int i, rndf_segment *s);
  void append_segment(rndf_segment *s);
  void delete_segment(int i);
  void clear_segments(void);

  int num_zones(void) const { return (signed)zone_.size(); }
  rndf_zone *zone(int i) const;    // inlined below
  void insert_zone(int i, rndf_zone *z);
  void append_zone(rndf_zone *z);
  void delete_zone(int i);
  void clear_zones(void);

  int num_trafficlights(void) const { return (signed)trafficlight_.size(); }
  rndf_trafficlight *trafficlight(int i) const;  // inlined below
  void insert_trafficlight(int i, rndf_trafficlight *s);
  //void append_trafficlight(rndf_trafficlight *s);
  void delete_trafficlight(int i);
  void clear_trafficlights(void);

  rndf_zone *first_zone(void) const;          // inlined below
  rndf_zone *last_zone(void) const;           // inlined below
  rndf_segment *first_segment(void) const;    // inlined below
  rndf_segment *last_segment(void) const;     // inlined below

  std::string filename(void) const { return filename_; }
  void filename(std::string name) { filename_ = name; }
  std::string format_version(void) const { return format_version_; }
  void format_version(std::string format) { format_version_ = format; }
  std::string creation_date(void) const { return creation_date_; }
  void creation_date(std::string date) { creation_date_ = date; }

  std::string id_string(void) const { return id_string_; }
  void id_string(std::string id) { id_string_ = id; }

  void string_to_waypoint_id(char *str, int *segment, int *lane, int *waypoint);
  void string_to_crosswalk_id(char *str, int *segment, int *crosswalk);

  rndf_waypoint *lookup_waypoint(int s, int l, int w) const;
  rndf_waypoint *closest_waypoint(double lat, double lon) const;
  rndf_waypoint *closest_waypoint(double utm_x, double utm_y, char *utmzone) const;

  rndf_crosswalk *lookup_crosswalk(int s, int w) const;
  rndf_crosswalk *closest_crosswalk(double utm_x, double utm_y, char *utmzone, int& index) const;

  rndf_trafficlight *lookup_trafficlight(int s, int w) const;
  rndf_trafficlight *closest_trafficlight(double utm_x, double utm_y, char *utmzone) const;

  bool is_super(void) const { return is_super_; }

  void cleanup(void);
  void upsample(double max_dist);
  void linkup_lanes(double change_length = 10);
  void mark_possible_uturns(void);

  void check_segment_crossings(rndf_waypoint *w, rndf_waypoint *wnext);
  void check_exit_crossings(rndf_waypoint *w, int e);

  void mark_merge_exits(void);
  void mark_yield_exits(void);

  void mark_intersections(void);

  int average_waypoint(double *mean_x, double *mean_y, char *utmzone);
  void rndf_bounds(double *min_x, double *min_y,
                   double *max_x, double *max_y);

  void insert_checkpoint(rndf_waypoint *new_cp);
  void renumber_checkpoints(void);

  void build_checkpoint_map(void);
  rndf_waypoint *checkpoint_waypoint(int id) { if(checkpoint_list_.find(id) != checkpoint_list_.end()) return checkpoint_list_[id]; else return NULL; }

  std::vector<rndf_trafficlight *> trafficlight_;

 private:
  inline int seg_seg_intersection(double x1, double y1, double x2, double y2,
                           double x3, double y3, double x4, double y4,
                           double *xc, double *yc);

  void clear_offrndf_links(rndf_waypoint *w);

  bool is_merge_exit(rndf_waypoint *w, int i);
  void add_waypoint_links(int s1, int l1, int w1, double change_length);
  void add_uturn_waypoint_links(int s1, int l1, int w1);
  void build_checkpoint_map(std::vector <rndf_waypoint *> &checkpoint_map);

  std::vector<rndf_segment *> segment_;
  std::vector<rndf_zone *>zone_;
  std::string filename_;
  std::string format_version_;
  std::string creation_date_;
  std::string id_string_;
  std::map<int, rndf_waypoint *> checkpoint_list_;
  bool is_super_;
};

inline rndf_trafficlight *rndf_file::trafficlight(int i) const
{
  if(i < 0 || i >= num_trafficlights())
    dgc_die("Error: rndf_file::trafficlight : index out of range\n");
  return trafficlight_[i];
}

inline rndf_segment *rndf_file::segment(int i) const
{
  if(i < 0 || i >= num_segments())
    dgc_die("Error: rndf_file::segment : index out of range\n");
  return segment_[i];
}

inline rndf_zone *rndf_file::zone(int i) const
{
  if(i < 0 || i >= num_zones())
    dgc_die("Error: rndf_file::zone : index out of range\n");
  return zone_[i];
}

inline rndf_segment *rndf_file::first_segment(void) const
{
  return (num_segments() == 0) ? NULL : segment_[0];
}

inline rndf_segment *rndf_file::last_segment(void) const
{
  return (num_segments() == 0) ? NULL : segment_[num_segments() - 1];
}

inline rndf_zone *rndf_file::first_zone(void) const
{
  return (num_zones() == 0) ? NULL : zone_[0];
}

inline rndf_zone *rndf_file::last_zone(void) const
{
  return (num_zones() == 0) ? NULL : zone_[num_zones() - 1];
}

} // namespace dgc

#endif // RNDF_RNDF_ROADNETWORK_H
