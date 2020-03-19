#ifndef _AW_SPOT_H_
#define _AW_SPOT_H_

#include "aw_RndfId.h"
#include "aw_wayPoint.h"
#include "aw_checkPoint.h"


namespace vlr {

namespace rndf
{
class Zone;
class Spot;

typedef std::map<std::string, Spot*> TSpotMap;


class Spot : public NetElement
{
public:
	Spot(uint32_t id, const std::string& strName);
	virtual ~Spot(void);

	uint32_t getWaypointIndex(const WayPoint* wp) const;

	void setZone(Zone* z) { parent_zone_ = z; }
	Zone* zone() const {return parent_zone_;}

	void setSpotWidth(int spot_width) { spot_width_=spot_width;}
	int getSpotWidth(void) {return spot_width_;}

	bool addWayPoint(WayPoint* wp);
	void removeWayPoint(uint32_t index);
	void removeWayPoint(WayPoint* wp);

	const TWayPointVec& wayPoints() { return waypoints_; }
	const TCheckPointMap& checkPoints() { return checkpoints_;}

	bool centerLatLon(double& clat, double& clon) const;

  inline std::string nextSpotPointStr() const {
    return name() + "." + nextIdStr(waypoints_);
  }

	uint32_t numSpotPoints() { return waypoints_.size(); } // should alway be 2

	void dump();

private:
	int spot_width_;

	TWayPointVec      waypoints_;
	TCheckPointMap    checkpoints_;

	Zone* parent_zone_;

	friend std::ostream& operator<<(std::ostream& os, const Spot& s);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Spot& s);
};

} // namespace vlr

#endif


