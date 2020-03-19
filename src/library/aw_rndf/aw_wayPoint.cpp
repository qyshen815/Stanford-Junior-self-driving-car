#include <iostream>
#include <iomanip>
#include <algorithm>

#include <lltransform.h>

#include "aw_lane.h"
#include "aw_wayPoint.h"

using namespace std;

namespace vlr {

namespace rndf {
WayPoint::WayPoint(uint32_t id, const string& strName) :
  NetElement(id, strName), lat_(0), lon_(0), utm_x_(0), utm_y_(0),
      is_virtual_(false), checkpoint_(NULL), stop_(NULL), parentlane_(NULL), parentspot_(NULL) {
  exits_.clear();
}

WayPoint::~WayPoint(void) {
}

void WayPoint::setCheckPoint(CheckPoint* cp) {
  if (cp == checkpoint_) return;
  checkpoint_ = cp;
}

void WayPoint::setStop(Stop* s) {
  if (s == stop_) return;
  stop_ = s;
}

void WayPoint::addExit(Exit* e) {
  if (!e) return;
  if (exits_.find(e->name()) != exits_.end()) return;
  if (parentlane_) {parentlane_->addExit(e);}
  exits_.insert(make_pair(e->name(), e));
}

void WayPoint::removeExit(Exit* e) {
  if (!e) return;
  if (exits_.find(e->name()) == exits_.end()) return;
  if (parentlane_) {parentlane_->removeExit(e);}
  exits_.erase(e->name());
}

void WayPoint::addEntry(Exit* e) {
  if (!e) return;
  if (entries_.find(e->name()) != entries_.end()) return;
  if (parentlane_) {parentlane_->addEntry(e);}
  entries_.insert(make_pair(e->name(), e));
}

void WayPoint::removeEntry(Exit* e) {
  if (!e) return;
  if (entries_.find(e->name()) == entries_.end()) return;
  if (parentlane_) {parentlane_->removeEntry(e);}
  entries_.erase(e->name());
}

void WayPoint::addCrosswalk(Crosswalk* cw, crosswalk_linktype type) {
  if (!cw) {
    return;
  }
  if (crosswalks_.find(cw->name()) != crosswalks_.end()) {
    return;
  }
  if (!cw->addWayPoint(this)) {
    return;
  }

  CrosswalkLink link;
  link.type_ = type;
  link.crosswalk_ = cw;

  //  if (parentlane_) parentlane_->addCrosswalk(cw, type);
  crosswalks_.insert(make_pair(cw->name(), link));
}

void WayPoint::removeCrosswalk(Crosswalk* cw) {
  if (!cw) {
    return;
  }
  if (crosswalks_.find(cw->name()) == crosswalks_.end()) {
    return;
  }

  //  if (parentlane_) parentlane_->removeCrosswalk(e);
  crosswalks_.erase(cw->name());
}

uint32_t WayPoint::nextCrosswalkId() const {
  vector<uint32_t> ids;
  for (TCrosswalkLinkMap::const_iterator it = crosswalks_.begin(); it != crosswalks_.end(); ++it) {
    ids.push_back((*it).second.crosswalk_->getID());
  }
  sort(ids.begin(), ids.end());

  for (uint32_t i = 1; i <= ids.size(); ++i) {
    if (i != ids[i - 1]) {
      return i;
    }
  }

  return ids.size() + 1;
}

void WayPoint::addTrafficLight(TrafficLight* tl) {
  if (!tl) {
    return;
  }
  if (trafficlights_.find(tl->name()) != trafficlights_.end()) {
    return;
  }
  if (!tl->addWayPoint(this)) {
    return;
  }

  //  if (parentlane_) parentlane_->addTrafficLight(cw, type);
  trafficlights_.insert(make_pair(tl->name(), tl));
}

void WayPoint::removeTrafficLight(TrafficLight* tl) {
  if (!tl) {
    return;
  }
  if (trafficlights_.find(tl->name()) == trafficlights_.end()) {
    return;
  }

  //  if (parentlane_) parentlane_->removeTrafficLight(e);
  trafficlights_.erase(tl->name());
}

std::string WayPoint::nextTrafficLightStr() const {
  return name() + "." + boost::lexical_cast<std::string>(nextTrafficLightId());
}

uint32_t WayPoint::nextTrafficLightId() const {
  vector<uint32_t> ids;
  for (TTrafficLightMap::const_iterator it = trafficlights_.begin(); it != trafficlights_.end(); ++it) {
    ids.push_back((*it).second->getID());
  }
  sort(ids.begin(), ids.end());

  for (uint32_t i = 1; i <= ids.size(); ++i) {
    if (i != ids[i - 1]) {
      return i;
    }
  }

  return ids.size() + 1;
}

void WayPoint::setLatLon(double lat, double lon) {
  lat_ = lat;
  lon_ = lon;
  latLongToUtm(lat_, lon_, &utm_x_, &utm_y_, utm_zone_);

  //	cout << "  -> setCoords LL( "<< lat <<", "<< lon <<" )  XY("<< m_coordinate_x <<","<< m_coordinate_y <<")" << endl;
}

void WayPoint::setUtm(double utm_x, double utm_y, const std::string& utm_zone) {
  utm_x_ = utm_x;
  utm_y_ = utm_y;
  utm_zone_ = utm_zone;

  utmToLatLong(utm_x, utm_y, utm_zone_.c_str(), &lat_, &lon_);
}

uint32_t WayPoint::index() const {
  if (parentlane_) return parentlane_->getWaypointIndex(this);
  else if (parentspot_) {
    // TODO fertig machen
    return 0;
  }
  return 0;
}

void WayPoint::dump() const {
  cout << "  WayPoint " << name() << " " << utm_x_ << " " << utm_y_ << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const WayPoint& w) {
  os << w.name_ << " " << setprecision(16) << w.lat_ << " " << w.lon_ << endl;
  return os;
}
}

} // namespace vlr

