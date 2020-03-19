#include <lltransform.h>

#include <rndfWayPoint.h>
#include <rndfTrafficLight.h>

namespace dgc {

using namespace std;

rndf_trafficlight::rndf_trafficlight()
{
  lat_ = 0;
  lon_ = 0;
  utm_x_ = 0;
  utm_y_ = 0;
  z_ = 0;

  group_id =0;


  orientation_ = 0.;
  parentrndf_ = NULL;

}
rndf_trafficlight::rndf_trafficlight(const double& lat, const double& lon, const double& z)
{
  set_ll(lat,lon);
  z_=z;

  group_id =0;


  orientation_ = 0.;
  parentrndf_ = NULL;

}

void rndf_trafficlight::compute_orientation()
{
  orientation_ = 0;
  double next_orientation;
  unsigned int count = 0;

  for (std::vector<rndf_waypoint*>::iterator it = linked_waypoints_.begin();
       it != linked_waypoints_.end(); ++it)
  {
    next_orientation = atan2((**it).utm_y()-utm_y_, (**it).utm_x()-utm_x_);

    //normalize orientation
    while (next_orientation - orientation_ > 180.)
      next_orientation -= 360.;

    while (next_orientation - orientation_ <= -180.)
      next_orientation += 360.;

    // This line was wrong, I've split it so order is operations is correct
    // give the assumption it's doing an average.  (mvs Dec 2009)
    orientation_ = ( orientation_ * count + next_orientation );
    orientation_ /= ++count;
  }

}

void rndf_trafficlight::remove_waypoint (const rndf_waypoint *wp)
{
  std::vector<rndf_waypoint*>::iterator it;
  for (it = linked_waypoints_.begin(); it != linked_waypoints_.end(); ++it) {
    if (*it == wp) {
      linked_waypoints_.erase(it);
      break;
    }
  }
  compute_orientation();
}

void rndf_trafficlight::add_waypoint (rndf_waypoint *wp)
{
  linked_waypoints_.push_back( wp );
  compute_orientation();
}


rndf_trafficlight::~rndf_trafficlight()
{
  for ( std::vector<rndf_waypoint*>::iterator it = linked_waypoints_.begin();
        it != linked_waypoints_.end(); ++it)
  {
    for( std::vector<rndf_trafficlight*>::iterator it2 = (**it).trafficlight_.begin();
         it2 != (**it).trafficlight_.end();++it2)
    {
      if(*it2 == this)
      {
        (**it).trafficlight_.erase(it2);
        break;
      }
    }
  }
}



rndf_trafficlight::rndf_trafficlight(const dgc::rndf_trafficlight & c)
{
  orientation_ = c.orientation_;
  lat_ = c.lat_;
  lon_ = c.lon_;
  utm_x_ = c.utm_x_;
  utm_y_ = c.utm_y_;
  z_ = c.z_;
  utmzone_ = c.utmzone_;
  parentrndf_ = c.parentrndf_;

  group_id = c.group_id;

  linked_waypoints_.reserve(c.linked_waypoints_.size());
  for (std::vector<rndf_waypoint*>::const_iterator it =
      c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.push_back(*it);
  }
}


rndf_trafficlight& rndf_trafficlight::operator=(const rndf_trafficlight &c)
{
  orientation_ = c.orientation_;
  lat_ = c.lat_;
  lon_ = c.lon_;
  utm_x_ = c.utm_x_;
  utm_y_ = c.utm_y_;
  z_ = c.z_;
  utmzone_ = c.utmzone_;
  parentrndf_ = c.parentrndf_;

  group_id = c.group_id;

  linked_waypoints_.reserve(c.linked_waypoints_.size());
  for (std::vector<rndf_waypoint*>::const_iterator it =
      c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.push_back(*it);
  }

  return *this;
}

int rndf_trafficlight::lookup_trafficlight_id(void) const {
  int i;

  for(i = 0; i < parentrndf()->num_trafficlights(); i++)
    if(parentrndf()->trafficlight(i) == this)
      return i;

  return -1;
}

void rndf_trafficlight::set_ll(double lat, double lon)
{
  char str[10];

  lat_ = lat;
  lon_ = lon;
  vlr::latLongToUtm(lat, lon, &utm_x_, &utm_y_, str);
  utmzone_ = str;
}

void rndf_trafficlight::set_utm(double utm_x, double utm_y, string utmzone)
{
  utm_x_ = utm_x;
  utm_y_ = utm_y;
  utmzone_ = utmzone;
  vlr::utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat_, &lon_);
}

} // namespace dgc
