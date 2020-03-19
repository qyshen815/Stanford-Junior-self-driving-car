/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/

#include "aw_SpeedLimitList.h"
#include "aw_segment.h"
#include "aw_zone.h"

using namespace std;

namespace vlr {

namespace rndf {

  SpeedLimitList::SpeedLimitList(unsigned int id, std::string& strName) : NetElement(id, strName) {
  }

  SpeedLimitList::~SpeedLimitList() {
  }

  int SpeedLimitList::size() {
    return m_vector.size();
  }

  bool SpeedLimitList::addSpeedLimit(SpeedLimit * limit) {
    m_vector.push_back(limit);
  return true;
  }

  void SpeedLimitList::dump() {
    std::cout << "speed limit list " << name() << std::endl;
    for (TSpeedLimitIterator it = begin(); it != end(); ++it) {
      (*it)->dump();
    }
  }

  SpeedLimit * SpeedLimitList::getSpeedLimitForSegment(std::string segmentName)
  {
  	//cout << "search for " << segmentName << endl;
    for (TSpeedLimitIterator it = m_vector.begin(); it != m_vector.end(); ++it) {
      SpeedLimit* sl = (*it);
      //sl->dump();
    	if (sl->segment() && sl->segment()->name() == segmentName) {
    		return sl;
    	}
    }
    return 0;
  }
  SpeedLimit * SpeedLimitList::getSpeedLimitForZone(std::string zoneName)
  {
    for (TSpeedLimitIterator it = m_vector.begin(); it != m_vector.end(); ++it) {
      SpeedLimit* sl = (*it);
    	if (sl->zone() && sl->zone()->name() == zoneName) {
    		return sl;
    	}
    }
    return 0;
  }

}

} // namespace vlr
