#include <iostream>
#include <algorithm>

#include "aw_zone.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Zone::Zone(uint32_t id, const string& strName) :
  NetElement(id, strName), m_speedLimit(NULL), m_offroad(false) {
}

Zone::~Zone(void) {
}

bool Zone::addPerimeter(Perimeter* pPerimeter) {
  pair<TPerimeterMap::iterator, bool> result = m_perimeters.insert(make_pair(pPerimeter->name(), pPerimeter));
  if (!result.second) {
    // setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    return false;
  }
  return true;
}

Perimeter* Zone::getPerimterById(uint32_t id) {
  for (TPerimeterMap::iterator it = m_perimeters.begin(); it != m_perimeters.end(); ++it)
    if (it->second->getID() == id) return it->second;
  return NULL;
}

bool Zone::addSpot(Spot* pSpot) {
  pair<TSpotMap::iterator, bool> result = m_spots.insert(make_pair(pSpot->name(), pSpot));
  if (!result.second) {
    // setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    return false;
  }
  return true;
}

Spot* Zone::getSpotById(uint32_t id) {
  for (TSpotMap::iterator it = m_spots.begin(); it != m_spots.end(); ++it)
    if (it->second->getID() == id) return it->second;
  return NULL;
}

void Zone::removePerimeter(Perimeter* p) {
  if (m_perimeters.find(p->name()) == m_perimeters.end()) {
    return;
  }
  m_perimeters.erase(p->name());
}

void Zone::removeSpot(Spot* s) {
  if (m_spots.find(s->name()) == m_spots.end()) {
    return;
  }
  m_spots.erase(s->name());
}

void Zone::dump() {
  cout << "----------------------------------------" << endl;
  cout << "Zone: " << name() << endl;
  cout << "description: " << m_description << endl;
  cout << "# of perimeters: " << m_perimeters.size() << endl;
  TPerimeterMap::iterator it, it_end;
  for (it = m_perimeters.begin(), it_end = m_perimeters.end(); it != it_end; ++it)
    (*it).second->dump();
  TSpotMap::iterator sit, sit_end;
  for (sit = m_spots.begin(), sit_end = m_spots.end(); sit != sit_end; ++sit)
    (*sit).second->dump();
  cout << "----------------------------------------" << endl;
}

uint32_t Zone::getNextPerimeterId() const {
  vector<uint32_t> ids;
  for (TPerimeterMap::const_iterator it = m_perimeters.begin(); it != m_perimeters.end(); ++it)
    ids.push_back(it->second->getID());
  for (TSpotMap::const_iterator it = m_spots.begin(); it != m_spots.end(); ++it)
    ids.push_back(it->second->getID());
  sort(ids.begin(), ids.end());

  for (uint32_t i = 1; i <= ids.size(); ++i)
    if (i != ids[i - 1]) return i;

  return ids.size() + 1;
}

std::string Zone::getNextPerimeterStr() const {
  return name() + "." + boost::lexical_cast<std::string>(getNextPerimeterId());
}

uint32_t Zone::getNextSpotId() const {
  return getNextPerimeterId();
}

std::string Zone::getNextSpotStr() const {
  return getNextPerimeterStr();
}

bool Zone::centerLatLon(double& clat, double& clon) {
  double latSum = 0, lonSum = 0;
  uint32_t num = 0;

  if (m_perimeters.size() == 0) {
    return false;
  }

  // sum valid Perimeter positions
  TPerimeterMap::iterator pit, pit_end;
  for (pit = m_perimeters.begin(), pit_end = m_perimeters.end(); pit != pit_end; ++pit) {
    if ((*pit).second->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  // add valid Spot positions
  TSpotMap::iterator sit, sit_end;
  for (sit = m_spots.begin(), sit_end = m_spots.end(); sit != sit_end; ++sit) {
    if ((*sit).second->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  clat = latSum / num;
  clon = lonSum / num;
  return true;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Zone& z) {
  os << RNDF_ZONE_BEGIN << " " << z.name_ << endl;
  os << RNDF_ZONE_NAME << " " << z.m_description << endl;
  os << RNDF_ZONE_NUM_SPOTS << " " << z.m_spots.size() << endl;

  TPerimeterMap::const_iterator it, it_end;
  for (it = z.m_perimeters.begin(), it_end = z.m_perimeters.end(); it != it_end; ++it)
    os << *it->second;
  TSpotMap::const_iterator sit, sit_end;
  for (sit = z.m_spots.begin(), sit_end = z.m_spots.end(); sit != sit_end; ++sit)
    os << *sit->second;

  os << RNDF_ZONE_END << endl;

  return os;
}
}

} // namespace vlr

