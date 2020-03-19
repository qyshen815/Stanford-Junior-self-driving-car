#ifndef AW_RNDF_CROSSWALK_H
#define AW_RNDF_CROSSWALK_H

#include <string>
#include <map>
#include <aw_wayPoint.h>

namespace vlr {

namespace rndf {

typedef enum { incoming_waypoint, stop_waypoint } crosswalk_linktype;

class WayPoint;
class Crosswalk;

class CrosswalkLink {
public:

 Crosswalk* crosswalk_;
 crosswalk_linktype type_;
};

class Crosswalk : public NetElement {
 public:
  Crosswalk(uint32_t id, const std::string& strName);
  ~Crosswalk();
  Crosswalk(const Crosswalk &c);
  Crosswalk& operator=(const Crosswalk &c);

  inline double lat1() const { return lat1_; }
  inline double lon1() const { return lon1_; }
  inline double utm_x1() const { return utm_x1_; }
  inline double utm_y1() const { return utm_y1_; }
  inline double lat2() const { return lat2_; }
  inline double lon2() const { return lon2_; }
  inline double utm_x2() const { return utm_x2_; }
  inline double utm_y2() const { return utm_y2_; }

  inline double width(void) const { return width_; }
  inline void width(float w) { width_ = w; }

  bool addWayPoint(WayPoint* w);
  void removeWayPoint(WayPoint* w);
  const std::map<std::string, WayPoint*>& linkedWayPoints() {return linked_waypoints_;}

  void set_ll1(double lat, double lon);
  void set_ll2(double lat, double lon);
  void set_utm1(double utm_x, double utm_y, std::string utmzone);
  void set_utm2(double utm_x, double utm_y, std::string utmzone);

  void dump() const;

 protected:
  double lat1_, lon1_;
  double lat2_, lon2_;
  double utm_x1_, utm_y1_;
  double utm_x2_, utm_y2_;
  std::string utmzone_;

  double width_;

  std::map<std::string, WayPoint*> linked_waypoints_;

  friend std::ostream& operator<<(std::ostream& os, const Crosswalk& cw);
};


// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Crosswalk& cw);

typedef std::map<std::string, CrosswalkLink> TCrosswalkLinkMap;
typedef std::map<std::string, Crosswalk*> TCrosswalkMap;
} // namespace rndf

} // namespace vlr

#endif // AW_RNDF_CROSSWALK_H
