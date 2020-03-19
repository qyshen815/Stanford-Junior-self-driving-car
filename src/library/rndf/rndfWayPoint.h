#ifndef RNDF_RNDF_WAYPOINT_H
#define RNDF_RNDF_WAYPOINT_H

#include <vector>
#include <string>

#include <roadrunner.h>

#include <rndfCrossWalk.h>

namespace dgc {

class rndf_zone;
class rndf_file;
class rndf_crosswalk;
class rndf_crosswalk_link;
class rndf_trafficlight;

//#ifndef RNDF_RNDF_CROSSWALK_H
//typedef enum { incoming_waypoint, stop_waypoint } rndf_crosswalk_linktype;
//#endif

typedef enum {transition, lanechange, uturn} rndf_linktype;

class rndf_link {
 public:
  class rndf_waypoint* link;
  float length;
  bool merge, yield, original;
};

struct yieldto_data {
  rndf_waypoint *waypoint;
  int exit_num;
};

typedef std::vector<yieldto_data> yieldto_list;

class rndf_data {
 public:
  virtual rndf_data *clone(void) = 0;
  virtual ~rndf_data() {};
 private:
};

class rndf_waypoint {
 public:
  rndf_waypoint();
  ~rndf_waypoint();
  rndf_waypoint(const rndf_waypoint &w);
  rndf_waypoint& operator=(const rndf_waypoint &w);

  char *rndf_string(void) const;
  void lookup_id(int *s, int *l, int *w) const;

  double lat(void) const { return lat_; }
  double lon(void) const { return lon_; }
  double utm_x(void) const { return utm_x_; }
  double utm_y(void) const { return utm_y_; }
  std::string utmzone(void) const { return utmzone_; }
  void set_ll(double lat, double lon);
  void set_utm(double utm_x, double utm_y, std::string utmzone);

  int num_exits(rndf_linktype t = transition) const;  // inlined below
  rndf_waypoint *exit(int i, rndf_linktype t = transition) const; // inlined below
  float exit_length(int i, rndf_linktype = transition) const; // inlined below
  bool exits_to(rndf_waypoint *dest, rndf_linktype = transition) const;

  int num_entries(rndf_linktype t = transition) const; // inlined below
  rndf_waypoint *entry(int i, rndf_linktype t = transition) const; // inlined below
  float entry_length(int i, rndf_linktype = transition) const; // inlined below
  bool entry_from(rndf_waypoint *src, rndf_linktype = transition) const;

  bool yield(void) const { return yield_; }
  void yield(bool yield) { yield_ = yield; }

  int num_yieldtos(void) const { return (int)yieldto_.size(); }
  yieldto_data yieldto(int i) const; // inlined below

  int num_exit_yieldtos(int i) const; // inlined below
  yieldto_data exit_yieldto(int i, int j) const; // inlined below

  bool exit_original(int i, rndf_linktype = transition) const;
  bool entry_original(int i, rndf_linktype = transition) const;

  bool yield_exit(int i) const; // inlined below
  void yield_exit(int i, bool yield);
  bool yield_entry(int i) const;
  void yield_entry(int i, bool yield);

  bool merge_exit(int i) const; // inlined below
  void merge_exit(int i, bool merge);
  bool merge_entry(int i) const;
  void merge_entry(int i, bool merge);

  void add_yieldto(rndf_waypoint *w, int exit_num);
  void add_exit_yieldto(int i, rndf_waypoint *w, int exit_num);

  void add_exit(rndf_waypoint *dest, rndf_linktype = transition, bool yield = false, bool merge = false, bool original = true);
  void delete_exit(rndf_waypoint *dest, rndf_linktype = transition);
  void clear_exits(void);

  void add_entry(rndf_waypoint *src, rndf_linktype = transition, bool yield = false, bool merge = false, bool original = true);

  void add_crosswalk(rndf_crosswalk *crosswalk, rndf_crosswalk_linktype = stop_waypoint );
  void delete_crosswalk(rndf_crosswalk *c);

  void add_trafficlight(rndf_trafficlight *trafficlight );
  void delete_trafficlight(rndf_trafficlight *c);

  void stop(bool s) { stop_ = s; }
  bool stop(void) const { return stop_; }

  bool crosswalk(void) const {return crosswalk_.size()>0;}
  bool trafficlight(void) const {return trafficlight_.size()>0;}

  void allway_stop(bool s) { allway_stop_ = s; }
  bool allway_stop(void) const { return allway_stop_; }

  void original(bool o) { original_ = o; update_original_links(); }
  bool original(void) const { return original_; }

  void checkpoint(bool c) { checkpoint_ = c; }
  bool checkpoint(void) const { return checkpoint_; }
  void checkpoint_id(int id) { checkpoint_id_ = id; }
  int checkpoint_id(void) const { return checkpoint_id_; }

  int lookup_waypoint_id(void) const;
  int lookup_original_waypoint_id(void) const;
  bool in_lane(void) const { return (parentlane_ != NULL) ? true : false; }
  bool in_zone(void) const { return (parentzone_ != NULL) ? true : false; }
  bool in_spot(void) const { return (parentspot_ != NULL) ? true : false; }
  void print(char *name) const;

  float length(void) const { return length_; }
  float heading(void) const { return heading_; }

  rndf_waypoint *next(void) const { return next_; }
  rndf_waypoint *prev(void) const { return prev_; }
  rndf_waypoint *next_original(void) const;
  rndf_waypoint *prev_original(void) const;
  class rndf_lane *parentlane(void) const { return parentlane_; }
  friend class rndf_lane;
  class rndf_zone *parentzone(void) const { return parentzone_; }
  friend class rndf_zone;
  class rndf_spot *parentspot(void) const { return parentspot_; }
  friend class rndf_spot;
  rndf_file* parentrndf(void) const;

  void clear_intersection_waypoints(void);
  void delete_intersection_waypoint(int i);
  void add_intersection_waypoint(rndf_waypoint *w);
  bool intersection(void) const { return intersection_waypoint_.size() > 0; }
  int num_intersection_waypoints(void) const { return (signed)intersection_waypoint_.size(); }
  rndf_waypoint *intersection_waypoint(int i) const;
  rndf_waypoint* left_lane();
  rndf_waypoint* right_lane();
  bool is_left_lane_change(int i);
  bool is_right_lane_change(int i);

  rndf_data *data;

  //TODO: make this private and add const access functions
  std::vector<rndf_crosswalk_link> crosswalk_;
  std::vector<rndf_trafficlight *> trafficlight_;

 private:
  double lat_, lon_;
  double utm_x_, utm_y_;
  std::string utmzone_;
  std::vector<rndf_link *> exit_;
  std::vector<rndf_link *> entry_;
  std::vector<rndf_link *> lanelink_exit_;
  std::vector<rndf_link *> lanelink_entry_;
  std::vector<rndf_link *> uturn_exit_;
  std::vector<rndf_link *> uturn_entry_;
  std::vector<rndf_waypoint *> intersection_waypoint_;
  bool stop_, allway_stop_;
  bool checkpoint_;
  int checkpoint_id_;
  bool original_;
  rndf_waypoint *next_, *prev_;

  float length_, heading_;
  bool yield_;

  yieldto_list yieldto_;
  std::vector <yieldto_list> exit_yieldto_;

  void calculate_theta(int do_neighbors);
  void recalculate_exitentry_lengths(int do_neighbors);

  bool exits_to(const std::vector<rndf_link *> *, rndf_waypoint *dest) const;
  bool entry_from(const std::vector<rndf_link *> *, rndf_waypoint *src) const;
  void add_exit(std::vector<rndf_link *> *exitlist,
    std::vector<rndf_link *> *entrylist,
    rndf_waypoint *dest, bool yield, bool merge, bool original);
  void add_entry(std::vector<rndf_link *> *entrylist,
     std::vector<rndf_link *> *exitlist,
     rndf_waypoint *src, bool yield, bool merge, bool original);

  class rndf_lane *parentlane_;
  class rndf_zone *parentzone_;
  class rndf_spot *parentspot_;
  void parentlane(rndf_lane *l);
  void parentzone(rndf_zone *z);
  void parentspot(rndf_spot *s);
  void update_original_links(void);
  void next(rndf_waypoint *n) { next_ = n; }
  void prev(rndf_waypoint *p) { prev_ = p; }
};


inline rndf_waypoint *rndf_waypoint::intersection_waypoint(int i) const
{
 if(i < 0 || i >= (signed)intersection_waypoint_.size())
   dgc_die("Error: rndf_waypoint::intersection_waypoint : entry index out of range.\n");
 return intersection_waypoint_[i];
}

inline int rndf_waypoint::num_exits(rndf_linktype t) const
{
 if(t == transition)
   return (signed)exit_.size();
 else if(t == lanechange)
   return (signed)lanelink_exit_.size();
 else if(t == uturn)
   return (signed)uturn_exit_.size();
 else
   dgc_die("Error: rndf_waypoint::num_exits : Illegal transition type\n");
 return 0;
}

inline rndf_waypoint *rndf_waypoint::exit(int i, rndf_linktype t) const
{
 switch(t) {
 case transition:
   if(i >= (signed)exit_.size())
     dgc_die("Error: rndf_waypoint::exit : exit index out of range\n");
   return exit_[i]->link;
   break;
 case lanechange:
   if(i >= (signed)lanelink_exit_.size())
     dgc_die("Error: rndf_waypoint::exit : lanelink index out of range\n");
   return lanelink_exit_[i]->link;
   break;
 case uturn:
   if(i >= (signed)uturn_exit_.size())
     dgc_die("Error: rndf_waypoint::exit : uturn index out of range\n");
   return uturn_exit_[i]->link;
   break;
 default:
   dgc_die("Error: rndf_waypoint::exit : Illegal transition type\n");
   break;
 }
 return NULL;
}

inline yieldto_data rndf_waypoint::yieldto(int i) const
{
 if(i < 0 || i >= (int)yieldto_.size())
   dgc_die("Error: rndf_waypoint::yieldto : index out of range.\n");
 return yieldto_[i];
}

inline int rndf_waypoint::num_exit_yieldtos(int i) const
{
 if(i < 0 || i >= (int)num_exits())
   dgc_die("Error: rndf_waypoint::num_exit_yieldtos : index out of range.\n");
 return (int)exit_yieldto_[i].size();
}

inline yieldto_data rndf_waypoint::exit_yieldto(int i, int j) const
{
 if(i < 0 || i >= (int)num_exits())
   dgc_die("Error: rndf_waypoint::num_exit_yieldtos : index i out of range.\n");
 if(j < 0 || j >= (int)exit_yieldto_[i].size())
   dgc_die("Error: rndf_waypoint::num_exit_yieldtos : index j out of range.\n");
 return exit_yieldto_[i][j];
}

/* yield exit functions */

inline bool rndf_waypoint::yield_exit(int i) const
{
 if(i < 0 || i >= (signed)exit_.size())
   dgc_die("Error: rndf_waypoint::yield_exit : exit index out of range.\n");
 return exit_[i]->yield;
}

inline void rndf_waypoint::yield_exit(int i, bool yield)
{
 rndf_waypoint *w;
 int j;

 if(i < 0 || i >= (signed)exit_.size())
   dgc_die("Error: rndf_waypoint::yield_exit : exit index out of range.\n");
 exit_[i]->yield = yield;
 w = exit_[i]->link;
 for(j = 0; j < w->num_entries(); j++)
   if(w->entry_[j]->link == this)
     w->entry_[j]->yield = yield;
}

inline bool rndf_waypoint::yield_entry(int i) const
{
 if(i < 0 || i >= (signed)entry_.size())
   dgc_die("Error: rndf_waypoint::yield_entry : entry index out of range.\n");
 return entry_[i]->yield;
}

inline void rndf_waypoint::yield_entry(int i, bool yield)
{
 rndf_waypoint *w;
 int j;

 if(i < 0 || i >= (signed)entry_.size())
   dgc_die("Error: rndf_waypoint::yield_entry : entry index out of range.\n");
 entry_[i]->yield = yield;
 w = entry_[i]->link;
 for(j = 0; j < w->num_exits(); j++)
   if(w->exit_[j]->link == this)
     w->exit_[j]->yield = yield;
}

/* merge exit functions */

inline void rndf_waypoint::merge_exit(int i, bool merge)
{
 rndf_waypoint *w;
 int j;

 if(i < 0 || i >= (signed)exit_.size())
   dgc_die("Error: rndf_waypoint::merge_exit : exit index out of range.\n");
 exit_[i]->merge = merge;
 w = exit_[i]->link;
 for(j = 0; j < w->num_entries(); j++)
   if(w->entry_[j]->link == this)
     w->entry_[j]->merge = merge;
}

inline bool rndf_waypoint::merge_exit(int i) const
{
 if(i < 0 || i >= (signed)exit_.size())
   dgc_die("Error: rndf_waypoint::merge_exit : exit index out of range.\n");
 return exit_[i]->merge;
}

inline bool rndf_waypoint::merge_entry(int i) const
{
 if(i < 0 || i >= (signed)entry_.size())
   dgc_die("Error: rndf_waypoint::merge_entry : entry index out of range.\n");
 return entry_[i]->merge;
}

inline void rndf_waypoint::merge_entry(int i, bool merge)
{
 rndf_waypoint *w;
 int j;

 if(i < 0 || i >= (signed)entry_.size())
   dgc_die("Error: rndf_waypoint::merge_entry : entry index out of range.\n");
 entry_[i]->merge = merge;
 w = entry_[i]->link;
 for(j = 0; j < w->num_exits(); j++)
   if(w->exit_[j]->link == this)
     w->exit_[j]->merge = merge;
}

inline bool rndf_waypoint::exit_original(int i, rndf_linktype t) const
{
 if(t == transition) {
   if(i < 0 || i >= (signed)exit_.size())
     dgc_die("Error: rndf_waypoint::exit_original : exit index out of range.\n");
   return exit_[i]->original;
 }
 else if(t == lanechange) {
   if(i < 0 || i >= (signed)lanelink_exit_.size())
     dgc_die("Error: rndf_waypoint::exit_original : exit index out of range.\n");
   return lanelink_exit_[i]->original;
 }
 else if(t == uturn) {
   if(i < 0 || i >= (signed)uturn_exit_.size())
     dgc_die("Error: rndf_waypoint::exit_original : exit index out of range.\n");
   return uturn_exit_[i]->original;
 }
 return false;
}

inline bool rndf_waypoint::entry_original(int i, rndf_linktype t) const
{
 if(t == transition) {
   if(i < 0 || i >= (signed)entry_.size())
     dgc_die("Error: rndf_waypoint::entry_original : exit index out of range.\n");
   return entry_[i]->original;
 }
 else if(t == lanechange) {
   if(i < 0 || i >= (signed)lanelink_entry_.size())
     dgc_die("Error: rndf_waypoint::entry_original : exit index out of range.\n");
   return lanelink_entry_[i]->original;
 }
 else if(t == uturn) {
   if(i < 0 || i >= (signed)uturn_entry_.size())
     dgc_die("Error: rndf_waypoint::entry_original : exit index out of range.\n");
   return uturn_entry_[i]->original;
 }
 return false;
}

inline float rndf_waypoint::exit_length(int i, rndf_linktype t) const
{
 if(t == transition)
   return exit_[i]->length;
 else if(t == lanechange)
   return lanelink_exit_[i]->length;
 else if(t == uturn)
   return uturn_exit_[i]->length;
 else
   dgc_die("Error: rndf_waypoint::exit_length : Illegal transition type\n");
 return 0;
}

inline int rndf_waypoint::num_entries(rndf_linktype t) const
{
 if(t == transition)
   return (signed)entry_.size();
 else if(t == lanechange)
   return (signed)lanelink_entry_.size();
 else if(t == uturn)
   return (signed)uturn_entry_.size();
 else
   dgc_die("Error: rndf_waypoint::num_entries : Illegal transition type\n");
 return 0;
}

inline float rndf_waypoint::entry_length(int i, rndf_linktype t) const
{
 if(t == transition)
   return entry_[i]->length;
 else if(t == lanechange)
   return lanelink_entry_[i]->length;
 else if(t == uturn)
   return uturn_entry_[i]->length;
 else
   dgc_die("Error: rndf_waypoint::entry_length : Illegal transition type\n");
 return 0;
}

inline rndf_waypoint *rndf_waypoint::entry(int i, rndf_linktype t) const
{
 switch(t) {
 case transition:
   if(i >= (signed)entry_.size())
     dgc_die("Error: rndf_waypoint::entry : entry index out of range\n");
   return entry_[i]->link;
   break;
 case lanechange:
   if(i >= (signed)lanelink_entry_.size())
     dgc_die("Error: rndf_waypoint::entry : lanelink index out of range\n");
   return lanelink_entry_[i]->link;
   break;
 case uturn:
   if(i >= (signed)uturn_entry_.size())
     dgc_die("Error: rndf_waypoint::entry : uturn index out of range\n");
   return uturn_entry_[i]->link;
   break;
 default:
   dgc_die("Error: rndf_waypoint::entry : Illegal transition type\n");
   break;
 }
 return NULL;
}

} // namespace dgc

#endif // RNDF_RNDF_WAYPOINT_H
