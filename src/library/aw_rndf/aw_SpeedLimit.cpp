/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_SpeedLimit.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {

SpeedLimit::SpeedLimit(uint32_t id, const std::string& name="Default speed limit") :
                            NetElement(id, name), min_speed_(0), max_speed_(0),
                            ref_is_lane_(false), ref_is_segment_(false), ref_is_zone_(false),
                            lane_(NULL), segment_(NULL), zone_(NULL), override(false) {
}

SpeedLimit::~SpeedLimit() {
}

void SpeedLimit::setLane(Lane* lane) {
  ref_is_lane_ = true;
  ref_is_segment_ = false;
  ref_is_zone_ = false;
  lane_ = lane;
  segment_ = NULL;
  zone_ = NULL;
  lane->setSpeedLimit(this);
}

void SpeedLimit::setSegment(Segment* segment) {
  ref_is_lane_ = false;
  ref_is_segment_ = true;
  ref_is_zone_ = false;
  lane_ = NULL;
  segment_ = segment;
  zone_ = NULL;
  segment->setSpeedLimit(this);
}

void SpeedLimit::setZone(Zone* zone) {
  ref_is_lane_ = false;
  ref_is_segment_ = false;
  ref_is_zone_ = true;
  lane_ = NULL;
  segment_ = NULL;
	zone_ = zone;
	zone_->setSpeedLimit(this);
}


void SpeedLimit::dump() {
	std::cout << "  speed limit for " << (ref_is_zone_ ? "zone " : (ref_is_segment_ ? "segment " : "lane ")) << ref_name_ << ": [" << min_speed_ << ", " << max_speed_ << "]" << std::endl;
}

}

} // namespace vlr
