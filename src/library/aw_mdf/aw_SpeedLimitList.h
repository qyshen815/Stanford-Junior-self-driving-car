#ifndef AW_SPEEDLIMITLIST_H
#define AW_SPEEDLIMITLIST_H

#include <iostream>
#include <vector>

#include "aw_netElement.h"
#include "aw_SpeedLimit.h"

namespace vlr {

namespace rndf {

  typedef std::vector<SpeedLimit*> TSpeedLimitVector;
  typedef TSpeedLimitVector::iterator TSpeedLimitIterator;

  class SpeedLimitList : public NetElement {

  public:
    SpeedLimitList(unsigned int id, std::string & strName);
    virtual ~SpeedLimitList();

    int size();

    virtual bool addSpeedLimit(SpeedLimit * limit);

    TSpeedLimitIterator begin() { return m_vector.begin(); }
    TSpeedLimitIterator end() { return m_vector.end(); }

    SpeedLimit * getSpeedLimitForSegment(std::string segmentName);
    SpeedLimit * getSpeedLimitForZone(std::string zoneName);

    void dump();

  private:
    TSpeedLimitVector m_vector;

  };
}

} // namespace vlr

#endif
