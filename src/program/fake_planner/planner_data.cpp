#include <roadrunner.h>
#include "planner_data.h"

using namespace dgc;

planner_data *planner_data::clone(void)
{
  planner_data *d = new planner_data;
  d->bt = this->bt;
  d->exit_bt = this->exit_bt;
  return d;
}

void add_waypoint_data(rndf_file *rndf)
{
  int i, j, k;

  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        planner_data *data = new planner_data;
        rndf->segment(i)->lane(j)->waypoint(k)->data = data;
      }
  for(i = 0; i < rndf->num_zones(); i++) {
    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
      planner_data *data = new planner_data;
      rndf->zone(i)->perimeter(j)->data = data;
    }
    for(j = 0; j < rndf->zone(i)->num_spots(); j++) 
      for(k = 0; k < rndf->zone(i)->spot(j)->num_waypoints(); k++) {
        planner_data *data = new planner_data;
        rndf->zone(i)->spot(j)->waypoint(k)->data = data;
      }
  }
}






