/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>
#include <algorithm>

#include "aw_segment.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Segment::Segment(uint32_t id, const string& strName) :
  NetElement(id, strName), speed_limit_(NULL), lat_sum_(0), lon_sum_(0), length_(0), offroad_(false) {
}

Segment::~Segment(void) {
}

void Segment::addLane(Lane* l) {
  if (lanes_.find(l) != lanes_.end()) return;
  lanes_.insert(l);

  double lat, lon;
  if (l->centerLatLon(lat, lon)) {
    lat_sum_ += lat;
    lon_sum_ += lon;
  }
}

void Segment::removeLane(Lane* l) {
  if (lanes_.find(l) == lanes_.end()) return;
  lanes_.erase(l);

  double lat, lon;
  if (l->centerLatLon(lat, lon)) {
    lat_sum_ -= lat;
    lon_sum_ -= lon;
  }
}

Lane* Segment::getLaneById(uint32_t id) {
  for (TLaneSet::iterator it = lanes_.begin(); it != lanes_.end(); ++it)
    if ((*it)->getID() == id) return *it;
  return NULL;
}

bool Segment::centerLatLon(double& clat, double& clon) const {
  double latSum = 0, lonSum = 0;
  uint32_t num = 0;

  if (lanes_.size() == 0) {
    return false;
  }

  TLaneSet::iterator lit, lit_end;
  for (lit = lanes_.begin(), lit_end = lanes_.end(); lit != lit_end; ++lit) {
    if ((*lit)->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  clat = latSum / num;
  clon = lonSum / num;
  return true;
}

void Segment::dump() {
  cout << "----------------------------------------" << endl;
  cout << "Segment: " << name() << endl;
  cout << "description: " << description_ << endl;
  cout << "# of lanes: " << lanes_.size() << endl;
  if (speed_limit_) {
    cout << "Speedlimit:" << endl;
    speed_limit_->dump();
  }
  TLaneSet::iterator it, it_end;
  for (it = lanes_.begin(), it_end = lanes_.end(); it != it_end; ++it) {
    (*it)->dump();
  }
  cout << "----------------------------------------" << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Segment& s) {

  os << RNDF_SEGMENT_BEGIN << " " << s.name_ << endl;
  os << RNDF_SEGMENT_NAME << " " << s.description_ << endl;
  if (s.speed_limit_) {
    os << RNDF_SEGMENT_SPEED_LIMIT << " " << s.speed_limit_->maxSpeed() << " " << s.speed_limit_->minSpeed() << endl;
  }
  os << RNDF_SEGMENT_NUM_LANES << " " << s.lanes_.size() << endl;

  TLaneSet::const_iterator it, it_end;
  for (it = s.lanes_.begin(), it_end = s.lanes_.end(); it != it_end; ++it)
    os << **it;

  os << RNDF_SEGMENT_END << endl;

  return os;
}
}

} // namespace vlr

