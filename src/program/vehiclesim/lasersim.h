#ifndef VLR_LASERSIM_H
#define VLR_LASERSIM_H

#include <roadrunner.h>
#include <ldlrs_interface.h>
#include "vehicle.h"

namespace vlr {
void generate_ldlrs_laser_scan(dgc::LdlrsLaser *ldlrs,
			  double laser_x, double laser_y, 
			  double laser_yaw, double start_angle,
			  double laser_max_range);

void
load_obstacle_map(char *filename);

void
fill_car_segments(int num_vehicles, vlr::vehicle_state *vehicle);

} // namespace vlr
#endif
