/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>

#include "aw_roadNetwork.h"
#include "aw_perimeter.h"

using namespace std;

namespace vlr {

namespace rndf {
Perimeter::Perimeter(uint32_t id, const string& strName) :
  NetElement(id, strName), parent_zone_(NULL) {
}

Perimeter::~Perimeter(void) {
}

bool Perimeter::addPerimeterPoint(PerimeterPoint* pPerimeterPoint) {
  return addPerimeterPoint(pPerimeterPoint, perimeter_points_.size());
}

bool Perimeter::addPerimeterPoint(PerimeterPoint* pPerimeterPoint, uint32_t insert_before) {
  if (insert_before >= perimeter_points_.size()) perimeter_points_.push_back(pPerimeterPoint);
  else perimeter_points_.insert(perimeter_points_.begin() + insert_before, pPerimeterPoint);
  return true;
}

void Perimeter::removePerimeterPoint(PerimeterPoint* pPerimeterPoint) {
  removePerimeterPoint(perimeterPointIndex(pPerimeterPoint));
}

void Perimeter::removePerimeterPoint(uint32_t index) {
  if (index >= perimeter_points_.size()) return;
  perimeter_points_.erase(perimeter_points_.begin() + index);
}

PerimeterPoint* Perimeter::perimeterPointById(uint32_t id) {
  for (TPerimeterPointVec::iterator it = perimeter_points_.begin(); it != perimeter_points_.end(); ++it)
    if ((*it)->getID() == id) return *it;
  return NULL;
}

uint32_t Perimeter::perimeterPointIndex(const PerimeterPoint* pPerimeterPoint) const {
  for (uint32_t i = 0; i < perimeter_points_.size(); ++i) {
    if (perimeter_points_[i] == pPerimeterPoint) return i;
  }
  return perimeter_points_.size();
}

bool Perimeter::addExit(Exit* pExit) {
  pair<TExitMap::iterator, bool> result = exits_.insert(make_pair(pExit->name(), pExit));
  if (!result.second) {
    //      setStatus("addExit: Exit with name " + strName + " was not added to the network.");
    return false;
  }
  return true;
}

void Perimeter::removeExit(Exit* pExit) {
  exits_.erase(pExit->name());
}

bool Perimeter::centerLatLon(double& clat, double& clon) const {
  if (perimeter_points_.size() == 0) {
    return false;
  }

  clat = 0.;
  clon = 0.;

  TPerimeterPointVec::const_iterator pit, pit_end;

  for (pit = perimeter_points_.begin(), pit_end = perimeter_points_.end(); pit != pit_end; ++pit) {
    clat += (*pit)->lat();
    clon += (*pit)->lon();
  }

  clat /= perimeter_points_.size();
  clon /= perimeter_points_.size();

  return true;
}

void Perimeter::dump() {
  cout << "Perimeter: " << name() << endl;
  cout << "  # of PerimeterPoints: " << perimeter_points_.size() << endl;
  TPerimeterPointVec::iterator cit, cit_end;
  for (cit = perimeter_points_.begin(), cit_end = perimeter_points_.end(); cit != cit_end; ++cit)
    (*cit)->dump();

  TExitMap::iterator sit, sit_end;
  for (sit = exits_.begin(), sit_end = exits_.end(); sit != sit_end; ++sit) {
    (*sit).second->dump();
  }
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Perimeter& p) {
  os << RNDF_PERIMETER_BEGIN << " " << p.name_ << endl;
  os << RNDF_PERIMETER_NUM_PERIMETERPOINTS << " " << p.perimeter_points_.size() << endl;

  TExitMap::const_iterator sit, sit_end;
  for (sit = p.exits_.begin(), sit_end = p.exits_.end(); sit != sit_end; ++sit)
    os << *sit->second;
  TPerimeterPointVec::const_iterator cit, cit_end;
  for (cit = p.perimeter_points_.begin(), cit_end = p.perimeter_points_.end(); cit != cit_end; ++cit)
    os << **cit;

  os << RNDF_PERIMETER_END << endl;

  return os;
}
}

} // namespace vlr

