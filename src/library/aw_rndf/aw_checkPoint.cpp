/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>

#include "aw_wayPoint.h"
#include "aw_checkPoint.h"

using namespace std;

namespace vlr {

namespace rndf {

CheckPoint::CheckPoint(uint32_t id, const string& strName) :
  NetElement(id, strName), m_waypoint(0) {
}

CheckPoint::~CheckPoint(void) {
}

void CheckPoint::dump() const {
  cout << "  CheckPoint " << name() << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const CheckPoint& cp) {
  os << RNDF_CHECKPOINT << " " << cp.m_waypoint->name() << " " << cp.name_ << endl;
  return os;
}

}

} // namespace vlr

