#ifndef DGC_STOPZONES_H
#define DGC_STOPZONES_H

#include <roadrunner.h>
#include <perception_interface.h>
#include <kdtree.h>
#include <aw_roadNetwork.h>
#include <vector>

typedef std::vector <vlr::PerceptionStopZone> zone_vector;

class stop_zone_finder {
public:
  stop_zone_finder(vlr::rndf::RoadNetwork& rn);
  ~stop_zone_finder();
  void closest_zones(zone_vector* zones, double x, double y, double range);
  
private:
  vlr::rndf::RoadNetwork& rn_;
  std::vector <vlr::rndf::WayPoint*> stop_w;
  std::vector <vlr::PerceptionStopZone> stop_z;
  dgc_kdtree_p kdtree;
  double origin_x, origin_y;
};

#endif
