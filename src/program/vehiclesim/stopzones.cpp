#include <roadrunner.h>
#include "stopzones.h"

using std::vector;
using namespace vlr; 
using namespace vlr::rndf;



stop_zone_finder::stop_zone_finder(RoadNetwork& rn) :
  rn_(rn) {
  vector<double> stop_x, stop_y;
  PerceptionStopZone z;
  bool first = true;
  double heading;

  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
    WayPoint* w = (*wpit).second;
    if ((w->stop()) || (!w->trafficLights().empty())) {
      if (first) {
        origin_x = w->utm_x();
        origin_y = w->utm_y();
        first = false;
      }
      stop_x.push_back(w->utm_x() - origin_x);
      stop_y.push_back(w->utm_y() - origin_y);
      stop_w.push_back(w);

//      heading = w->heading();
//      if (k != 0) {
//        w2 = rndf->segment(i)->lane(j)->original_waypoint(k - 1);
//        heading = atan2(w->utm_y() - w2->utm_y(), w->utm_x() - w2->utm_x());
//      }
//
//      z.segment = i;
//      z.lane = j;
//      z.waypoint = k;
//      z.heading = heading;
//      z.utm_x = w->utm_x();
//      z.utm_y = w->utm_y();
//      z.width = dgc_feet2meters(14.0);
//      z.length = 8.0;
//      z.state = 0;
//      z.hits = 0;
//      stop_z.push_back(z);
    }

  }
  kdtree = dgc_kdtree_build_balanced(&(stop_x[0]), &(stop_y[0]), (signed)stop_x.size());
}

stop_zone_finder::~stop_zone_finder()
{
  dgc_kdtree_free(&kdtree);
}

void stop_zone_finder::closest_zones(zone_vector *zones, double x, 
				     double y, double range)
{
  dgc_dlist_node_p dlist = NULL, temp;

  zones->clear();
  dlist = dgc_kdtree_range_search(kdtree, x - origin_x, y - origin_y, range);
  temp = dlist;
  while(temp != NULL) {
    (*zones).push_back(stop_z[temp->id]);
    temp = temp->next;
  }
  dgc_dlist_free(&dlist);
}
