#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>

#include <global.h>

#include "aw_segment.h"
#include "aw_lane.h"

using namespace std;

namespace vlr {

namespace rndf {

const double Lane::default_width;

Lane::Lane(uint32_t id, const string& strName, const bool isVirtual) :
  NetElement(id, strName), segment_(NULL), speed_limit_(NULL), lane_width_(Lane::default_width),
      left_boundary_type_(Lane::UnknownBoundary), right_boundary_type_(Lane::UnknownBoundary), lat_sum_(0),
      lon_sum_(0), length_(0), is_virtual_(isVirtual), bbox_valid_(false), type_(car_lane) {
}

Lane::~Lane() {
  // TODO: remove references in other objects...
}

bool Lane::addWayPoint(WayPoint* pWayPoint) {
  return addWayPoint(pWayPoint, waypoints_.size());
}

bool Lane::addWayPoint(WayPoint* pWayPoint, uint32_t insert_before) {
  if (insert_before >= waypoints_.size()) {
    waypoints_.push_back(pWayPoint);
  }
  else {
    waypoints_.insert(waypoints_.begin() + insert_before, pWayPoint);
  }
  lat_sum_ += pWayPoint->lat();
  lon_sum_ += pWayPoint->lon();
  bbox_valid_ = false;
  return true;
}

void Lane::removeWayPoint(WayPoint* wp) {
  removeWayPoint(getWaypointIndex(wp));
}

void Lane::removeWayPoint(uint32_t index) {
  if (index >= waypoints_.size()) {
    return;
  }
  lat_sum_ -= waypoints_[index]->lat();
  lon_sum_ -= waypoints_[index]->lon();
  waypoints_.erase(waypoints_.begin() + index);
  bbox_valid_ = false;
}

WayPoint* Lane::getWaypointById(uint32_t id) {
  for (TWayPointVec::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it)
    if ((*it)->getID() == id) return *it;
  return NULL;
}

uint32_t Lane::getWaypointIndex(const WayPoint* wp) const {
  for (uint32_t i = 0; i < waypoints_.size(); ++i) {
    if (waypoints_[i] == wp) return i;
  }
  return waypoints_.size();
}

void Lane::addExit(Exit* e) {
  exits_.insert(make_pair(e->name(), e));
}

void Lane::removeExit(Exit* e) {
  exits_.erase(e->name());
}

void Lane::addEntry(Exit* e) {
  entries_.insert(make_pair(e->name(), e));
}

void Lane::removeEntry(Exit* e) {
  entries_.erase(e->name());
}

LaneSegment* Lane::getLaneSegmentWithFromPoint(WayPoint* fromPoint) {
  for (TLaneSegmentVec::iterator it = lane_segments_.begin(); it != lane_segments_.end(); ++it)
    if ((*it)->fromWayPoint() == fromPoint) return *it;
  return NULL;
}

LaneSegment* Lane::getLaneSegmentWithToPoint(WayPoint* toPoint) {
  for (TLaneSegmentVec::iterator it = lane_segments_.begin(); it != lane_segments_.end(); ++it)
    if ((*it)->toWayPoint() == toPoint) return *it;
  return NULL;
}

bool Lane::centerLatLon(double& clat, double& clon) const {
  if (waypoints_.size() == 0) {
    return false;
  }

  clat = 0.;
  clon = 0.;

  TWayPointVec::const_iterator wit, wit_end;

  for (wit = waypoints_.begin(), wit_end = waypoints_.end(); wit != wit_end; ++wit) {
    clat += (*wit)->lat();
    clon += (*wit)->lon();
  }

  clat /= waypoints_.size();
  clon /= waypoints_.size();

  return true;
}

void Lane::getBoundingBox(double& xmin, double& ymin, double& xmax, double& ymax) {
  if (!bbox_valid_) {
    TWayPointVec::const_iterator wit = waypoints_.begin();
    xmin_ = xmax_ = (*wit)->x();
    ymin_ = ymax_ = (*wit)->y();
    wit++;

    for (; wit != waypoints_.end(); wit++) {
      if ((*wit)->x() < xmin_) {
        xmin_ = (*wit)->x();
      }
      else if ((*wit)->x() > xmax_) {
        xmax_ = (*wit)->x();
      }

      if ((*wit)->y() < ymin_) {
        ymin_ = (*wit)->y();
      }
      else if ((*wit)->y() > ymax_) {
        ymax_ = (*wit)->y();
      }
    }
    bbox_valid_ = true;
  }

  xmin = xmin_;
  ymin = ymin_;
  xmax = xmax_;
  ymax = ymax_;
}

void Lane::dump() const {
  cout << "  width: " << lane_width_ << endl;
  cout << "  length: " << length_ << endl;
  cout << "  left boundary: " << left_boundary_type_ << endl;
  cout << "  right boundary: " << right_boundary_type_ << endl;
  cout << "  # of way points: " << waypoints_.size() << endl;
  cout << "  # of exits: " << exits_.size() << endl;
  TCheckPointMap::const_iterator cit, cit_end;

  TExitMap::const_iterator sit, sit_end;
  for (sit = exits_.begin(), sit_end = exits_.end(); sit != sit_end; ++sit)
    (*sit).second->dump();
  TWayPointVec::const_iterator wpit = waypoints_.begin(), wpit_end = waypoints_.end();

  for (; wpit != wpit_end; wpit++) {
    TCrosswalkLinkMap& cwl_map = (*wpit)->crosswalks();
    if (!cwl_map.empty()) {
      TCrosswalkLinkMap::const_iterator cwlit = cwl_map.begin(), cwlit_end = cwl_map.end();
      for (; cwlit != cwlit_end; cwlit++) {
        cout << "cross " << (*wpit)->name() << " " << (*cwlit).first
            << ((*cwlit).second.type_ == stop_waypoint ? " stop" : " incoming");
      }
    }
  }

  wpit = waypoints_.begin(), wpit_end = waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    TTrafficLightMap& tl_map = (*wpit)->trafficLights();
    if (!tl_map.empty()) {
      TTrafficLightMap::const_iterator tlit = tl_map.begin(), tlit_end = tl_map.end();
      for (; tlit != tlit_end; tlit++) {
        cout << "light " << (*wpit)->name() << " " << (*tlit).first;
      }
    }
  }

  TWayPointVec::const_iterator it, it_end;
  for (it = waypoints_.begin(), it_end = waypoints_.end(); it != it_end; ++it)
    (*it)->dump();

}

string Lane::boundaryToRndfString(eBoundaryTypes boundary) {
  if (boundary == DoubleYellow) return RNDF_LANE_BOUNDARYTYPE_DOUBLEYELLOW;
  if (boundary == SolidYellow) return RNDF_LANE_BOUNDARYTYPE_SOLIDYELLOW;
  if (boundary == SolidWhite) return RNDF_LANE_BOUNDARYTYPE_SOLIDWHITE;
  if (boundary == BrokenWhite) return RNDF_LANE_BOUNDARYTYPE_BROKENWHITE;
  return "/* unknown boundary type! */";
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Lane& l) {
  os << RNDF_LANE_BEGIN << " " << l.name_ << endl;
  os << RNDF_LANE_NUM_WAYPOINTS << " " << l.waypoints_.size() << endl;
  os << RNDF_LANE_WIDTH << " " << dgc_meters2feet(l.lane_width_) << endl;
  if (l.speed_limit_) {
    os << RNDF_LANE_SPEED_LIMIT << " " << l.speed_limit_->maxSpeed() << " " << l.speed_limit_->minSpeed() << endl;
  }
  if (l.left_boundary_type_ != Lane::UnknownBoundary) os << RNDF_LANE_LEFT_BOUNDARY << " " << l.boundaryToRndfString(
      l.left_boundary_type_) << endl;
  if (l.right_boundary_type_ != Lane::UnknownBoundary) os << RNDF_LANE_RIGHT_BOUNDARY << " " << l.boundaryToRndfString(
      l.right_boundary_type_) << endl;

  TCheckPointMap checkpoints;
  TStopMap stoppoints;

  TWayPointVec::const_iterator wpit = l.waypoints_.begin(), wpit_end = l.waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    CheckPoint* cp = (*wpit)->checkPoint();
    if (cp) {
      checkpoints.insert(std::make_pair(cp->name(), cp));
    }
    Stop* sp = (*wpit)->stop();
    if (sp) {
      stoppoints.insert(std::make_pair(sp->name(), sp));
    }
  }
  TCheckPointMap::const_iterator cit, cit_end;
  for (cit = checkpoints.begin(), cit_end = checkpoints.end(); cit != cit_end; ++cit) {
    os << *cit->second;
  }
  TStopMap::const_iterator eit, eit_end;
  for (eit = stoppoints.begin(), eit_end = stoppoints.end(); eit != eit_end; ++eit) {
    os << *eit->second;
  }
  TExitMap::const_iterator sit, sit_end;
  for (sit = l.exits_.begin(), sit_end = l.exits_.end(); sit != sit_end; ++sit) {
    os << *sit->second;
  }

  wpit = l.waypoints_.begin(), wpit_end = l.waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    TCrosswalkLinkMap& cwl_map = (*wpit)->crosswalks();
    if (!cwl_map.empty()) {
      TCrosswalkLinkMap::const_iterator cwlit = cwl_map.begin(), cwlit_end = cwl_map.end();
      for (; cwlit != cwlit_end; cwlit++) {
        os << "cross " << (*wpit)->name() << " " << (*cwlit).first
            << ((*cwlit).second.type_ == stop_waypoint ? " stop" : " incoming") << endl;
      }
    }
  }

  wpit = l.waypoints_.begin(), wpit_end = l.waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    TTrafficLightMap& tl_map = (*wpit)->trafficLights();
    if (!tl_map.empty()) {
      TTrafficLightMap::const_iterator tlit = tl_map.begin(), tlit_end = tl_map.end();
      for (; tlit != tlit_end; tlit++) {
        os << "light " << (*wpit)->name() << " " << (*tlit).first << endl;
      }
    }
  }

  TWayPointVec::const_iterator it, it_end;
  for (it = l.waypoints_.begin(), it_end = l.waypoints_.end(); it != it_end; ++it) {
    os << **it;
  }

  os << RNDF_LANE_END << endl;

  return os;
}

std::string Lane::nextWayPointStr() const {
  std::cout << __FUNCTION__ << ": " << name() + "." + nextIdStr(waypoints_) << std::endl;
  return name() + "." + nextIdStr(waypoints_);
}

}

} // namespace vlr
