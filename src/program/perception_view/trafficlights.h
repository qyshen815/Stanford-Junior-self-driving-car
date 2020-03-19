#pragma once

#include <map>
#include <string>
#include <aw_roadNetwork.h>

#include "view.h"

namespace dgc {

extern std::map <std::string, vlr::rndf::lightState> trafficlight_state;

void draw_trafficlights(vlr::rndf::RoadNetwork& rn, double origin_x, double origin_y);
}
