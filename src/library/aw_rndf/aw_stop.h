#ifndef _STOP_H_
#define _STOP_H_

#include "aw_netElement.h"
#include "aw_wayPoint.h"

namespace vlr {

namespace rndf {
class Stop;
typedef std::map<std::string, Stop*> TStopMap;

class Stop: public NetElement {
public:
  friend class RoadNetwork;

  Stop(uint32_t id, const std::string& strName);
  Stop(const Stop&);
  virtual ~Stop(void);
  Stop& operator=(const Stop& other);
  Stop& copy(const Stop& other);

  WayPoint* wayPoint(void) {
    return way_point_;
  }
  void setWayPoint(WayPoint* way_point) {
    way_point_ = way_point;
  }
  void dump() const;

private:
  WayPoint* way_point_;
protected:
  friend std::ostream& operator<<(std::ostream& os, const Stop& s);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Stop& s);

}
;

} // namespace vlr

#endif

