#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>

#include <global.h>

#include "aw_segment.h"
#include "aw_laneSegment.h"

using namespace std;

namespace vlr {

namespace rndf {

std::string LaneSegment::name() const {
  if (!from_way_point_ || !to_way_point_) return "LaneSegment_undef";
  return "LaneSegment_" + from_way_point_->name() + "_" + to_way_point_->name();
}

}

} // namespace vlr
