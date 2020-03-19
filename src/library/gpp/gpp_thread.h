#ifndef DGC_GPP_THREAD_H
#define DGC_GPP_THREAD_H

#include <planner_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
#include <param_interface.h>
#include <grid.h>
#include <rndf.h>
#include "gpp.h"
#include "message_queue.h"

#define     DOORS_OPEN_INWARD      1
#define     DOORS_OPEN_OUTWARD     2
#define     DOORS_CLOSED           3

void read_gpp_parameters(dgc::ParamInterface* pint, int argc, char **argv,
			 gpp_params *p);

void gpp_send_gls(dgc::IpcInterface *ipc, gpp_params *params);

void gpp_start_thread(gpp_params* p, dgc::rndf_file *rndf,
		      dgc_grid_p obstacle_grid_param, 
		      pthread_mutex_t *obstacle_grid_mutex_param,
		      vlr::PerceptionObstacles *obstacles,
		      pthread_mutex_t *obstacle_mutex_param,
		      status_message_queue *message_queue);

void gpp_update_plan_velocities(gpp_path* path);

void gpp_plan_path(void);

bool gpp_plan_ready(gpp_path *path);

void gpp_set_door_state(int state);

void gpp_set_zone(int zone_num);

void gpp_set_uturn_boundary(dgc::rndf_waypoint* w1, dgc::rndf_waypoint* w2);

void gpp_use_fine_grid(void);

void gpp_pick_goal(gpp_goal *goal);

void gpp_use_smoother(int use);

void gpp_use_coarse_grid(void);

void gpp_clear_previous_goals(void);

void gpp_set_goal_fence(bool active);

void gpp_set_start_fence(bool active);

void gpp_set_pop_limit(int pop_limit);

bool gpp_goal_fence_active(void);

bool gpp_start_fence_active(void);

bool gpp_boundary_active(void);

void gpp_breadth_first(bool active);

void gpp_set_pose(dgc::ApplanixPose *applanix_pose);

void gpp_set_localize_offset(dgc::LocalizePose *localize_pose);

void gpp_get_plan_info(double *plan_from_x, double *plan_from_y, 
		       double *plan_from_theta);

void gpp_reset_planning(void);

// internal functions

void gpp_initialize(gpp_params *p);

void grid_to_obstacle_list(dgc_grid_p obstacle_grid, obstacle_info *obs, 
			   double smooth_x, double smooth_y, 
			   double localize_x_offset, double localize_y_offset,
			   double goal_x, double goal_y, double goal_theta);

extern vlr::GlsOverlay *gpp_gls;

#endif
