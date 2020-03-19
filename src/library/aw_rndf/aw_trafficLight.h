#ifndef AW_RNDF_TRAFFICLIGHT_H_
#define AW_RNDF_TRAFFICLIGHT_H_

#include <string>
#include <stdint.h>
#include <aw_wayPoint.h>

namespace vlr {
namespace rndf {

enum lightState {
  LIGHT_STATE_RED,
  LIGHT_STATE_YELLOW,
  LIGHT_STATE_GREEN,
  LIGHT_STATE_UNKNOWN,
};

class TrafficLight;

typedef std::map<std::string, TrafficLight*> TTrafficLightMap;

class TrafficLight : public NetElement {
 public:
  TrafficLight(uint32_t id, const std::string& strName);
  ~TrafficLight();
//  TrafficLight(const double& lat, const double& lon, const double& z);
  TrafficLight(const TrafficLight& c);
  TrafficLight& operator=(const TrafficLight &c);

  inline double lat() const { return lat_; }
  inline double lon() const { return lon_; }
  inline double utm_x() const { return utm_x_; }
  inline double utm_y() const { return utm_y_; }
  inline const std::string& utmZone() const {return utmzone_;}
  inline const double& orientation() {return orientation_;}
  inline void orientation(double yaw) {orientation_ = yaw;}
  void computeOrientation();

  double z() const { return z_; }
  void z(double z) { z_ = z; }

  uint32_t groupId() const { return group_id_; }
  void groupId(uint32_t id) { group_id_=id; }

  void setLatLon(double lat, double lon);
  void setUtm(double utm_x, double utm_y, std::string utmzone);

  bool addWayPoint(WayPoint *wp);
  void removeWayPoint(const WayPoint* wp);
  inline const std::map<std::string, WayPoint*>& waypoints() const {return linked_waypoints_;}

 private:

  std::map<std::string, WayPoint*> linked_waypoints_;

  double lat_, lon_;
  double utm_x_, utm_y_;
  std::string utmzone_;
  double z_;
  double orientation_;
  uint32_t group_id_;
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const TrafficLight& cw);

} // namespace rndf
} // namespace vlr

#endif // AW_RNDF_TRAFFICLIGHT_H_
