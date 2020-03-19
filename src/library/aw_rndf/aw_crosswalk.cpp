#include <iostream>
#include <iomanip>
#include <stdio.h>

#include <global.h>
#include <lltransform.h>

#include <aw_wayPoint.h>
#include <aw_crosswalk.h>

namespace vlr {

namespace rndf {

using namespace std;

Crosswalk::Crosswalk(uint32_t id, const std::string& strName) :
                          NetElement(id, strName), lat1_(0), lon1_(0), lat2_(0), lon2_(0),
                          utm_x1_(0), utm_y1_(0), utm_x2_(0), utm_y2_(0),
                          width_(0) { //, parentsegment_(NULL) {
}

Crosswalk::~Crosswalk() {
  for (TWayPointMap::iterator it = linked_waypoints_.begin(); it != linked_waypoints_.end(); ++it) {
    (*it).second->removeCrosswalk(this);
  }
}

Crosswalk::Crosswalk(const Crosswalk & c) : NetElement(c.getID(), c.name()) {
  lat1_ = c.lat1_;
  lon1_ = c.lon1_;
  lat2_ = c.lat2_;
  lon2_ = c.lon2_;
  utm_x1_ = c.utm_x1_;
  utm_y1_ = c.utm_y1_;
  utm_x2_ = c.utm_x2_;
  utm_y2_ = c.utm_y2_;
  width_ = c.width_;
  utmzone_ = c.utmzone_;

  for (TWayPointMap::const_iterator it = c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.insert(*it);

  }
 // parentsegment_ = NULL;
}

Crosswalk& Crosswalk::operator=(const Crosswalk &c) {
  printf("TODO: copy operator!\n");
  lat1_ = c.lat1_;
  lon1_ = c.lon1_;
  lat2_ = c.lat2_;
  lon2_ = c.lon2_;
  utm_x1_ = c.utm_x1_;
  utm_y1_ = c.utm_y1_;
  utm_x2_ = c.utm_x2_;
  utm_y2_ = c.utm_y2_;
  width_ = c.width_;
  return *this;
}

//int Crosswalk::lookup_crosswalk_id() const
//{
//  int i;
//
//  for(i = 0; i < parentsegment()->num_crosswalks(); i++)
//    if(parentsegment()->Crosswalk(i) == this)
//      return i;
//
//  return -1;
//}

bool Crosswalk::addWayPoint(WayPoint* w) {
  if (!w) {return false;}
  if (linked_waypoints_.find(w->name()) != linked_waypoints_.end()) {return false;}

  linked_waypoints_.insert( make_pair(w->name(), w) );
  return true;
}

void Crosswalk::removeWayPoint(WayPoint* w) {
  if (!w) {return;}
  if (linked_waypoints_.find(w->name()) == linked_waypoints_.end()) {return;}

  linked_waypoints_.erase(w->name());
}

void Crosswalk::set_ll1(double lat, double lon) {
  char str[10];

  lat1_ = lat;
  lon1_ = lon;
  latLongToUtm(lat, lon, &utm_x1_, &utm_y1_, str);
  utmzone_ = str;
}

void Crosswalk::set_ll2(double lat, double lon)
{
  char str[10];

  lat2_ = lat;
  lon2_ = lon;
  latLongToUtm(lat, lon, &utm_x2_, &utm_y2_, str);
  utmzone_ = str;
}


void Crosswalk::set_utm1(double utm_x, double utm_y, string utmzone)
{
  utm_x1_ = utm_x;
  utm_y1_ = utm_y;
  utmzone_ = utmzone;
  utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat1_, &lon1_);
}

void Crosswalk::set_utm2(double utm_x, double utm_y, string utmzone)
{
  utm_x2_ = utm_x;
  utm_y2_ = utm_y;
  utmzone_ = utmzone;
  utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat2_, &lon2_);
}

void Crosswalk::dump() const
{
  std::cout << "  width: " << width() << endl;
  std::cout << "  first point: " << utm_x1() << ", " << utm_y1() << " (" << lat1() << ", " << lon1() << ") " << endl;
  std::cout << "  second point: " << utm_x2() << ", " << utm_y2() << " (" << lat2() << ", " << lon2() << ") " << endl;
  std::cout << "  # linked WayPoints: " << linked_waypoints_.size() << endl;

  TWayPointMap::const_iterator wit, wit_end;
  for(wit = linked_waypoints_.begin(), wit_end = linked_waypoints_.end(); wit != wit_end; ++wit)
    std::cout << *wit->second;
}

std::ostream& operator<<(std::ostream& os, const Crosswalk& cw) {

os << RNDF_CROSSWALK_BEGIN << " " << cw.name() << endl;
os << RNDF_CROSSWALK_WIDTH << " " << dgc_meters2feet(cw.width()) << endl;
os << RNDF_CROSSWALK_P1 << " " << setprecision(16) << cw.lat1() << " " << cw.lon1() << endl;
os << RNDF_CROSSWALK_P2 << " " << setprecision(16) << cw.lat2() << " " << cw.lon2() << endl;
os << RNDF_CROSSWALK_END << endl;

return os;
}

} // namespace rndf
} // namespace vlr
