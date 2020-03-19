/*
 * perception_globals.h
 *
 *  Created on: May 6, 2010
 *      Author: duhadway
 */

#ifndef PERCEPTION_GLOBALS_H_
#define PERCEPTION_GLOBALS_H_

double                          velodyne_ts  = 0;
double                          last_velodyne_ts  = 0;

dgc_perception_map_cells_p     obstacles_s;
dgc_perception_map_cells_p     map_s;
pthread_mutex_t                radar_mutex[NUM_RADARS];

std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_predicted;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked;
std::vector<std::tr1::shared_ptr<Obstacle> >    obstacles_segmented;

char *imagery_root;
char *cal_filename = NULL;
//GlsOverlay                    * gls = NULL;

/* tracker stuff */
grid_stat_t                     grid_stat;
dgc_grid_p                      grid;

dgc_perception_map_cell_p       default_map_cell     = NULL;
dgc_perception_map_cell_p       default_terrain_cell = NULL;

LocalizePose                    localize_pos = {0, 0.0, 0.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "" };

char                          * rndf_filename = NULL;
dgc_global_pose_t               global_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"};
perception_settings_t           settings;

MultiBooster* booster = NULL;

#endif /* PERCEPTION_GLOBALS_H_ */
