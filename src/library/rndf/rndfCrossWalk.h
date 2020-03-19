#ifndef RNDF_RNDF_CROSSWALK_H
#define RNDF_RNDF_CROSSWALK_H

#include <string>
#include <rndfSegment.h>
#include <rndfWayPoint.h>

namespace dgc {

typedef enum { incoming_waypoint, stop_waypoint } rndf_crosswalk_linktype;

class rndf_crosswalk;
class rndf_waypoint;

class rndf_crosswalk_link {
public:

 rndf_crosswalk* crosswalk_;
 rndf_crosswalk_linktype type_;
};

class rndf_crosswalk {
 public:
  rndf_crosswalk();
  ~rndf_crosswalk();
  rndf_crosswalk(const rndf_crosswalk &c);
  rndf_crosswalk& operator=(const rndf_crosswalk &c);

  double lat1(void) const { return lat1_; }
  double lon1(void) const { return lon1_; }
  double utm_x1(void) const { return utm_x1_; }
  double utm_y1(void) const { return utm_y1_; }
  double lat2(void) const { return lat2_; }
  double lon2(void) const { return lon2_; }
  double utm_x2(void) const { return utm_x2_; }
  double utm_y2(void) const { return utm_y2_; }

  double width(void) const { return width_; }
  void width(float w) { width_ = w; }

  void set_ll1(double lat, double lon);
  void set_ll2(double lat, double lon);
  void set_utm1(double utm_x, double utm_y, std::string utmzone);
  void set_utm2(double utm_x, double utm_y, std::string utmzone);

  int lookup_crosswalk_id(void) const;

  class rndf_segment *parentsegment(void) const { return parentsegment_; }

  friend class rndf_segment;
  //TODO: make this private
  std::vector<rndf_waypoint*> linked_waypoints_;

 private:
  double lat1_, lon1_;
  double lat2_, lon2_;
  double utm_x1_, utm_y1_;
  double utm_x2_, utm_y2_;
  std::string utmzone_;

  double width_;

  class rndf_segment *parentsegment_;
  void parentsegment(class rndf_segment *s) { parentsegment_ = s; }
};

} // namespace dgc

#endif // RNDF_RNDF_CROSSWALK_H
