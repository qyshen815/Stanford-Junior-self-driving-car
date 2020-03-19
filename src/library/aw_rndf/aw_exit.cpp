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

using namespace std;

namespace vlr {

namespace rndf {
Exit::Exit(uint32_t id, const string& strName) : NetElement(id, strName), m_exitFromWayPoint(NULL),
                                                     m_exitToWayPoint(NULL), m_exitFromPerimeterPoint(NULL),
                                                     m_exitToPerimeterPoint(NULL), m_exitType(LaneToLane) {
}

Exit::~Exit(void) {
}

void Exit::dump() const {
  cout << "  Exit " << name() << " ";
  if (m_exitType == LaneToLane)
    cout << m_exitFromWayPoint->name() << " " << m_exitToWayPoint->name();
  else if (m_exitType == LaneToPerimeter)
    cout << m_exitFromWayPoint->name() << " " << m_exitToPerimeterPoint->name();
  else if (m_exitType == PerimeterToLane)
    cout << m_exitFromPerimeterPoint->name() << " " << m_exitToWayPoint->name();
  else if (m_exitType == PerimeterToPerimeter) cout << m_exitFromPerimeterPoint->name() << " "
      << m_exitToPerimeterPoint->name();
  cout << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Exit& e) {
  os << RNDF_EXIT << " ";
  if (e.m_exitType == Exit::LaneToLane)
    os << e.m_exitFromWayPoint->name() << " " << e.m_exitToWayPoint->name();
  else if (e.m_exitType == Exit::LaneToPerimeter)
    os << e.m_exitFromWayPoint->name() << " " << e.m_exitToPerimeterPoint->name();
  else if (e.m_exitType == Exit::PerimeterToLane)
    os << e.m_exitFromPerimeterPoint->name() << " " << e.m_exitToWayPoint->name();
  else if (e.m_exitType == Exit::PerimeterToPerimeter) os << e.m_exitFromPerimeterPoint->name() << " "
      << e.m_exitToPerimeterPoint->name();
  os << endl;
  return os;
}
}

} // namespace vlr

