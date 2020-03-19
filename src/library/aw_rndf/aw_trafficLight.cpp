#include <cmath>
#include <iomanip>
#include <lltransform.h>

#include <aw_wayPoint.h>
#include <aw_trafficLight.h>

namespace vlr {

namespace rndf {

using namespace std;

TrafficLight::TrafficLight(uint32_t id, const std::string& strName) :
                            NetElement(id, strName), lat_(0), lon_(0), utm_x_(0), utm_y_(0),
                            z_(0), orientation_(0), group_id_(0) { //, parentrndf_(NULL) {
}

//TrafficLight::TrafficLight(const double& lat, const double& lon, const double& z) {
//  set_ll(lat, lon);
//  z_=z;
//  orientation_ = 0.;
//  group_id_=0;
////  parentrndf_ = NULL;
//}

TrafficLight::~TrafficLight() {
  for (TWayPointMap::iterator it = linked_waypoints_.begin(); it != linked_waypoints_.end(); ++it) {
    (*it).second->removeTrafficLight(this);
  }
}

TrafficLight::TrafficLight(const TrafficLight& c) : NetElement(c.getID(), c.name())
{
  orientation_ = c.orientation_;
  lat_ = c.lat_;
  lon_ = c.lon_;
  utm_x_ = c.utm_x_;
  utm_y_ = c.utm_y_;
  z_ = c.z_;
  utmzone_ = c.utmzone_;
//
  group_id_ = c.group_id_;

  for (TWayPointMap::const_iterator it =
      c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.insert(*it);
  }
}


TrafficLight& TrafficLight::operator=(const TrafficLight &c)
{
  orientation_ = c.orientation_;
  lat_ = c.lat_;
  lon_ = c.lon_;
  utm_x_ = c.utm_x_;
  utm_y_ = c.utm_y_;
  z_ = c.z_;
  utmzone_ = c.utmzone_;

  group_id_ = c.group_id_;

  linked_waypoints_.clear();
  for (TWayPointMap::const_iterator it = c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.insert(*it);
  }

  return *this;
}

//int TrafficLight::lookup_trafficlight_id(void) const {
//  int i;
//
//  for(i = 0; i < parentrndf()->num_trafficlights(); i++)
//    if(parentrndf()->trafficlight(i) == this)
//      return i;
//
//  return -1;
//}

void TrafficLight::computeOrientation()
{
  orientation_ = 0;
  double next_orientation;
  uint32_t count = 0;

  for (TWayPointMap::const_iterator it = linked_waypoints_.begin();
       it != linked_waypoints_.end(); ++it)
  {
    next_orientation = atan2((*it).second->utm_y()-utm_y_, (*it).second->utm_x()-utm_x_);

    //normalize orientation
    while (next_orientation - orientation_ > 180.)
      next_orientation -= 360.;

    while (next_orientation - orientation_ <= -180.)
      next_orientation += 360.;

    count++;
    orientation_ = ( orientation_ * count + next_orientation ) / count;
  }

}

bool TrafficLight::addWayPoint(WayPoint* w) {
  if (!w) {return false;}
  if (linked_waypoints_.find(w->name()) != linked_waypoints_.end()) {return false;}

  linked_waypoints_.insert( make_pair(w->name(), w) );
  computeOrientation();
  return true;
}

void TrafficLight::removeWayPoint(const WayPoint* w) {
  if (!w) {return;}
  if (linked_waypoints_.find(w->name()) == linked_waypoints_.end()) {return;}

  linked_waypoints_.erase(w->name());
  computeOrientation();
}

void TrafficLight::setLatLon(double lat, double lon)
{
  char str[10];

  lat_ = lat;
  lon_ = lon;
  latLongToUtm(lat, lon, &utm_x_, &utm_y_, str);
  utmzone_ = str;
}

void TrafficLight::setUtm(double utm_x, double utm_y, string utmzone)
{
  utm_x_ = utm_x;
  utm_y_ = utm_y;
  utmzone_ = utmzone;
  utmToLatLong(utm_x, utm_y, utmzone_, &lat_, &lon_);
}

std::ostream& operator<<(std::ostream& os, const TrafficLight& tl) {

os << RNDF_TRAFFIC_LIGHT_BEGIN << " " << tl.name() << endl;
os << RNDF_TRAFFIC_LIGHT_GROUP_ID << " " << tl.groupId() << endl;
os << RNDF_TRAFFIC_LIGHT_POSITION << " " << setprecision(16) << tl.lat() << " " << tl.lon() << " " << tl.z() << endl;
os << RNDF_TRAFFIC_LIGHT_END << endl;

return os;
}

} // namespace rndf

} // namespace vlr
