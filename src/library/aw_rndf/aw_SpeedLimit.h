/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef _SPEEDLIMIT_H_
#define _SPEEDLIMIT_H_

#include <iostream>
#include "aw_netElement.h"


namespace vlr {

namespace rndf {

class Segment;
class Zone;
class Lane;

class SpeedLimit : public NetElement
{
public:
	SpeedLimit(uint32_t id, const std::string& name);
	virtual ~SpeedLimit();

  inline double minSpeed() const {return min_speed_;}
  inline double maxSpeed() const {return max_speed_;}

	void setRefName(const std::string& name) { ref_name_ = name; }
  void setSegment(Segment * seg);
  void setLane(Lane* lane);
	void setZone(Zone * Zone);
	void minSpeed(double speed) { min_speed_ = speed; }
	void maxSpeed(double speed) { max_speed_ = speed; }

  const Lane* lane() const { return lane_; };
  const Segment* segment() const { return segment_; };
	const Zone* zone() const { return zone_; };

	void dump();

	void setOverride(bool o=true) { override = o; };
	bool isOverride() const { return override; };

private:
	double min_speed_;
	double max_speed_;

	std::string ref_name_;
	bool ref_is_lane_, ref_is_segment_, ref_is_zone_;
	Lane* lane_;
	Segment* segment_;
	Zone* zone_;
	bool override;
};

}

} // namespace vlr

#endif
