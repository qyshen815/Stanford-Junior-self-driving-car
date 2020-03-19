#pragma once

#include <string>
#include <rndfRoadNetwork.h>
#include <rndfWayPoint.h>

namespace dgc {

class rndf_trafficlight;
class rndf_waypoint;

class rndf_trafficlight {
 public:
  rndf_trafficlight();
  ~rndf_trafficlight();
  rndf_trafficlight(const double& lat, const double& lon, const double& z);
  rndf_trafficlight(const rndf_trafficlight &c);
  rndf_trafficlight& operator=(const rndf_trafficlight &c);

  double lat(void) const { return lat_; }
  double lon(void) const { return lon_; }
  double utm_x(void) const { return utm_x_; }
  double utm_y(void) const { return utm_y_; }

  inline const double& orientation(){ return orientation_; };
  void compute_orientation();

  rndf_file* parentrndf(void) const { return parentrndf_; }
  void parentrndf(rndf_file* r) { parentrndf_ = r; }

  double z(void) const { return z_; }
  void z(double z) { z_ = z; }

  void set_ll(double lat, double lon);
  void set_utm(double utm_x, double utm_y, std::string utmzone);


  int lookup_trafficlight_id(void) const;

  void add_waypoint (rndf_waypoint *wp);
  void remove_waypoint (const rndf_waypoint *wp);

  int group_id;

 private:

  std::vector<rndf_waypoint*> linked_waypoints_;

  double orientation_;

  double lat_, lon_;
  double utm_x_, utm_y_;
  std::string utmzone_;

  double z_;
  rndf_file *parentrndf_;
};

} // namespace dgc
