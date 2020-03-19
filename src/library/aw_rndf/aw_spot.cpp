/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>
#include <global.h>

#include "aw_spot.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Spot::Spot(uint32_t id, const string& strName) :
  NetElement(id, strName) {
}

Spot::~Spot(void) {
}

void Spot::dump() {
  cout << "Spot: " << name() << endl;
  cout << "  width: " << spot_width_ << endl;
  NetElement::dump();
  //cout << "  # of SpotPoints: " << m_SpotPoints.size() << endl;
  //cout << "  # of CheckPoints: " << checkpoints_.size() << endl;
  //TSpotPointMap::iterator it,it_end;
  //for(it = m_SpotPoints.begin(),it_end = m_SpotPoints.end(); it != it_end; ++it)
  //  (*it).second->dump();
  //cout << endl;;
}

uint32_t Spot::getWaypointIndex(const WayPoint* wp) const {
  for (uint32_t i = 0; i < waypoints_.size(); ++i) {
    if (waypoints_[i] == wp) return i;
  }
  return waypoints_.size();
}

bool Spot::addWayPoint(WayPoint* pWayPoint) {
  waypoints_.push_back(pWayPoint);
  return true;
}

void Spot::removeWayPoint(uint32_t index) {
  if (index >= waypoints_.size()) return;
  waypoints_.erase(waypoints_.begin() + index);
}

void Spot::removeWayPoint(WayPoint* wp) {
  removeWayPoint(getWaypointIndex(wp));
}

bool Spot::centerLatLon(double& clat, double& clon) const {
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

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Spot& s) {
  os << RNDF_SPOT_BEGIN << " " << s.name_ << endl;
  os << RNDF_SPOT_WIDTH << " " << dgc_meters2feet(s.spot_width_) << endl;

  TCheckPointMap checkpoints;

  TWayPointVec::const_iterator wpit = s.waypoints_.begin(), wpit_end = s.waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    CheckPoint* cp = (*wpit)->checkPoint();
    if (cp) {
      checkpoints.insert(std::make_pair(cp->name(), cp));
    }
  }
  TCheckPointMap::const_iterator cit, cit_end;
  for (cit = s.checkpoints_.begin(), cit_end = s.checkpoints_.end(); cit != cit_end; ++cit) {
    os << *cit->second;
  }

  TWayPointVec::const_iterator it, it_end;
  for (it = s.waypoints_.begin(), it_end = s.waypoints_.end(); it != it_end; ++it) {
    os << **it;
  }

  os << RNDF_SPOT_END << endl;

  return os;
}
}

} // namespace vlr

