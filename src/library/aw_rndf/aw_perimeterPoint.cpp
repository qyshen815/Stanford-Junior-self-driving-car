/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>
#include <iomanip>
#include <string.h>

#include "aw_roadNetwork.h"
#include "aw_perimeterPoint.h"

using namespace std;

namespace vlr {

namespace rndf {
PerimeterPoint::PerimeterPoint(uint32_t id, const string& strName) :
  NetElement(id, strName), m_parentPerimeter(NULL), m_coordinate_lat(0), m_coordinate_lon(0), m_coordinate_utm_x(0),
      m_coordinate_utm_y(0) {
}

PerimeterPoint::~PerimeterPoint(void) {
}

void PerimeterPoint::addExit(Exit* e) {
  if (!e) return;
  if (m_exits.find(e->name()) != m_exits.end()) return;
  if (m_parentPerimeter) m_parentPerimeter->addExit(e);
  m_exits.insert(make_pair(e->name(), e));
}

void PerimeterPoint::removeExit(Exit* e) {
  if (!e) return;
  if (m_exits.find(e->name()) == m_exits.end()) return;
  if (m_parentPerimeter) m_parentPerimeter->removeExit(e);
  m_exits.erase(e->name());
}

void PerimeterPoint::addEntry(Exit* e) {
  m_entries.insert(make_pair(e->name(), e));
}

void PerimeterPoint::removeEntry(Exit* e) {
  m_entries.erase(e->name());
}

void PerimeterPoint::setLatLon(double lat, double lon) {
  m_coordinate_lat = lat;
  m_coordinate_lon = lon;
  latLongToUtm(m_coordinate_lat, m_coordinate_lon, &m_coordinate_utm_x, &m_coordinate_utm_y, m_coordinate_utm_zone);
}

void PerimeterPoint::setUtm(double utm_x, double utm_y, char* utm_zone) {
  m_coordinate_utm_x = utm_x;
  m_coordinate_utm_y = utm_y;
  strcpy(m_coordinate_utm_zone, utm_zone);

  utmToLatLong(utm_x, utm_y, utm_zone, &m_coordinate_lat, &m_coordinate_lon);
}

uint32_t PerimeterPoint::index() const {
  if (m_parentPerimeter) {
    return m_parentPerimeter->perimeterPointIndex(this);
  }

  return 0;
}

void PerimeterPoint::dump() {
  cout << "  PerimeterPoint " << name() << " " << m_coordinate_utm_x << " " << m_coordinate_utm_y << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p) {
  os << p.name_ << " " << setprecision(16) << p.m_coordinate_lat << " " << p.m_coordinate_lon << endl;
  return os;
}
}

} // namespace vlr

