#ifndef DGC_TRAJECTORY_H
#define DGC_TRAJECTORY_H

#include <kdtree.h>

typedef struct {
  double x, y, theta, velocity, yaw_rate;
  double cumulative_distance;
  double z, pitch;
} dgc_trajectory_waypoint_t, *dgc_trajectory_waypoint_p;

typedef struct {
  int num_waypoints, looping;
  double total_distance;
  dgc_trajectory_waypoint_p waypoint;
} dgc_trajectory_t, *dgc_trajectory_p;

dgc_trajectory_p dgc_trajectory_init(int num_waypoints, int looping);

void dgc_trajectory_realloc(dgc_trajectory_p trajectory, int num_waypoints);

void dgc_trajectory_free(dgc_trajectory_p trajectory);

dgc_trajectory_p dgc_trajectory_read(char *filename);

void dgc_trajectory_write(dgc_trajectory_p trajectory, char *filename);

dgc_kdtree_p dgc_trajectory_to_kdtree(dgc_trajectory_p trajectory);

dgc_trajectory_p dgc_trajectory_decimate(dgc_trajectory_p t_in, 
                                         double waypoint_dist);

void dgc_trajectory_create_filename(char *input_filename, 
                                    char *output_filename);

#endif
