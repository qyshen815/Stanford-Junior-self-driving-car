#ifndef DGC_BT_H
#define DGC_BT_H

#include <rndf.h>
#include <mdf.h>

void bt_assign_speed_limits(dgc::rndf_file *rndf, dgc::mdf_file *mdf, 
                            double max_speed_mph, double max_lateral_accel);

void bt_compute_curvature(dgc::rndf_file *rndf);

void add_base_trajectory(dgc::rndf_file *rndf, char *filename);

#endif
