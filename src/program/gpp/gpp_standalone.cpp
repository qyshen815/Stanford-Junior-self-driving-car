#include <roadrunner.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <planner_interface.h>
#include <perception_interface.h>
#include <error_interface.h>
#include <gls_interface.h>
#include <grid.h>
#include <rndf.h>
#include <lltransform.h>
#include "gpp.h"
#include "gpp_thread.h"

/* TODO LIST */

/* check path while planning is happening */

/* fixed map size */

/* hook up to main planner */

/* holes in perimeter near entrances and exits */
/* cache boundary points */

/* max curvature constraint */

/* command to abort planning - is this necessary ? */

/* handle no path to goal in reasonable way */

using namespace dgc;

/* planner input data */

rndf_file *rndf = NULL;

int received_applanix_pose = 0;
dgc_applanix_pose_message applanix_pose;

dgc_localize_pose_message localize_pose;

int received_map = 0;
dgc_grid_p obstacle_grid = NULL;
double obstacle_grid_min_x = 0, obstacle_grid_min_y = 0;
double obstacle_grid_width = 0, obstacle_grid_height = 0;
pthread_mutex_t obstacle_grid_mutex = PTHREAD_MUTEX_INITIALIZER;

dgc_perception_obstacles_message obstacles;
pthread_mutex_t obstacles_mutex = PTHREAD_MUTEX_INITIALIZER;

status_message_queue message_queue;

int received_goal = 0;

int gpp_intermediate_goal_num = 0;

/* params */
gpp_params gpp_param;
double gpp_planner_hz;
char *rndf_filename = NULL;

void publish_message_queue(void)
{
  static status_message *status = NULL;
  static int num_status = 0, max_status = 0;
  int i;

  /* copy status messages out */
  message_queue.lock();
  num_status = 0;
  for(i = 0; i < message_queue.num_messages(); i++) {
    if(num_status == max_status) {
      max_status += 100;
      status = (status_message *)realloc(status, max_status * 
					 sizeof(status_message));
      dgc_test_alloc(status);
    }
    strcpy(status[num_status], message_queue.message[i]);
    num_status++;
  }
  message_queue.clear_messages();
  message_queue.unlock();

  /* publish status messages */
  for(i = 0; i < num_status; i++)
    dgc_error_send_status(status[i]);
}

void obstacle_map_initialize(int x_size, int y_size, double resolution)
{
  dgc_perception_map_grid_p default_cell = NULL;

  default_cell = (dgc_perception_map_grid_p)
    calloc(1, sizeof(dgc_perception_map_grid_t));
  dgc_test_alloc(default_cell);
  
  obstacle_grid = 
    dgc_grid_initialize(resolution, x_size, y_size,
                        sizeof(dgc_perception_map_grid_t), default_cell);
}

void clear_obstacle_map(void)
{
  int i, n;

  if(obstacle_grid != NULL) {
    n = obstacle_grid->rows * obstacle_grid->cols;
    for(i = 0; i < n; i++) 
      memcpy((dgc_perception_map_grid_p)obstacle_grid->cell + i, 
	     obstacle_grid->default_value, obstacle_grid->bytes_per_cell);
  }
}

void map_diff_handler(dgc_perception_map_diff_message *mapdiff)
{
  dgc_perception_map_grid_p cell;
  int i;

  pthread_mutex_lock(&obstacle_grid_mutex);
  if(obstacle_grid == NULL) {
    obstacle_map_initialize(mapdiff->mapsize_x, mapdiff->mapsize_y,
			    mapdiff->resolution);
    obstacle_grid_width = obstacle_grid->cols * obstacle_grid->resolution;
    obstacle_grid_height = obstacle_grid->rows * obstacle_grid->resolution;
    
    /* start GPP thread */
    gpp_start_thread(&gpp_param, rndf, obstacle_grid, &obstacle_grid_mutex,
		     &obstacles, &obstacles_mutex, 
		     &message_queue);
  }

  if(mapdiff->new_map) 
    clear_obstacle_map();

  dgc_grid_recenter_grid(obstacle_grid, mapdiff->center_x, mapdiff->center_y);
  for(i = 0; i < mapdiff->num_points; i++) {
    cell = (dgc_perception_map_grid_p)
      dgc_grid_get_xy(obstacle_grid, mapdiff->grid[i].x, mapdiff->grid[i].y);
    if(cell != NULL) {
      cell->x = mapdiff->grid[i].x;
      cell->y = mapdiff->grid[i].y;
      cell->type = mapdiff->grid[i].type;
    }
  }
  
  obstacle_grid_min_x = obstacle_grid->map_c0 * obstacle_grid->resolution;
  obstacle_grid_min_y = obstacle_grid->map_r0 * obstacle_grid->resolution;
  pthread_mutex_unlock(&obstacle_grid_mutex);

  received_map = 1;
}

void gpp_publish_plan(dgc_planner_trajectory_message *output_plan)
{
  IPC_RETURN_TYPE err;

  if(output_plan->num_waypoints == 0)
    return;
  output_plan->timestamp = dgc_get_time();
  err = IPC_publishData(DGC_PLANNER_TRAJECTORY_NAME, output_plan);
  dgc_test_ipc_exit(err, "Could not publish", DGC_PLANNER_TRAJECTORY_NAME);
}

bool gpp_iter(double requested_end_vel)
{
  static dgc_planner_trajectory_message output_plan;
  static double next_plan_time = 0;
  static bool waiting = false;
  static int first_iter = 1, no_perception = 1;
  static int count = 0;
  double current_time;
  gpp_path path;
  double end_vel = requested_end_vel;
  bool reached_goal = false;

  /* blank the map on the first iteration */
  if(first_iter) {
    output_plan.num_waypoints = 0;
    output_plan.waypoint = NULL;
    strcpy(output_plan.host, dgc_hostname());
    dgc_perception_map_reset();
    received_map = 0;
    first_iter = 0;
    return false;
  }

  publish_message_queue();

  /* wait until we get a map and a goal until we start planning */
  current_time = dgc_get_time();
  if(no_perception && received_map && received_goal) {
    next_plan_time = current_time + 1 / gpp_planner_hz;
    no_perception = 0;
    gpp_plan_path();
  }

  /* if a plan is ready, publish it */
  if(gpp_plan_ready(&path) && !waiting) {
    path.extract_dgc_trajectory(&output_plan, &end_vel, 
				gpp_param.forward_accel,
				gpp_param.lateral_accel);

    gpp_publish_plan(&output_plan);

    if(path.reached_goal && path.current_i == path.num_waypoints() - 1 &&
       fabs(applanix_pose.speed) < end_vel + dgc_mph2ms(0.1) &&
       path.goal_id == gpp_intermediate_goal_num)  {
      fprintf(stderr, "REACHED GOAL\n");
      reached_goal = true;
    }

    gpp_send_gls(&gpp_param);

    waiting = true;
    
    if(path.num_waypoints() > 0)
      count++;
    //   if(count == 1)
    //      exit(0);
  }
  
  /* wait to plan again, if necessary */
  if(waiting && current_time > next_plan_time) {
    waiting = false;
    while(current_time > next_plan_time)
      next_plan_time += (1 / gpp_planner_hz);
    gpp_plan_path();
  }

  return reached_goal;
}

void applanix_pose_handler(void)
{
  received_applanix_pose = 1;
  gpp_set_pose(&applanix_pose);
  gpp_iter(0.0);
}

void localize_pose_handler(void)
{
  gpp_set_localize_offset(&localize_pose);
}

void planner_goal_handler(dgc_planner_goal_message *planner_goal)
{
  char utmzone[10];
  double x, y;
  gpp_goal goal;
  int i, current_zone = -1;

  latLongToUtm(planner_goal->goal_lat, planner_goal->goal_lon, &x, &y, utmzone);

  goal.add_goal(x, y, planner_goal->goal_theta, 0.5, dgc_d2r(5.0), true,
		true, NULL, false);
  gpp_intermediate_goal_num++;
  goal.id = gpp_intermediate_goal_num;

  for(i = 0; i < rndf->num_zones(); i++)
    if(rndf->zone(i)->car_inside(x, y, planner_goal->goal_theta, 0, 0)) 
      current_zone = i;

  gpp_set_door_state(DOORS_OPEN_INWARD);
  gpp_set_zone(current_zone);
  gpp_set_goal_fence(gpp_param.use_goal_fence);
  gpp_set_start_fence(false);
  gpp_pick_goal(&goal);
  gpp_reset_planning();
  received_goal = 1;
}

void read_parameters(int argc, char **argv, gpp_params *p)
{
  dgc_param_t param[] = {
    {"rndf", "rndf_file", DGC_PARAM_FILE, &rndf_filename, 0, NULL},
  };
  dgc_param_install_params(argc, argv, param,
                           sizeof(param) / sizeof(param[0]));

  read_gpp_parameters(argc, argv, p);
}

int main(int argc, char **argv)
{
  /* IPC initialization */
  dgc_ipc_initialize_locked_with_name(argc, argv, "PLANNER");
  dgc_param_check_version(argv[0]);
  
  read_parameters(argc, argv, &gpp_param);

  /* read RNDF files */
  rndf = new rndf_file;
  if(rndf->load(rndf_filename) < 0)
    dgc_die("Error: could not read RNDF file %s\n", rndf_filename);
  rndf->build_checkpoint_map();

  dgc_ipc_define_test_exit(DGC_PLANNER_TRAJECTORY_NAME,
                           DGC_PLANNER_TRAJECTORY_FMT);

  dgc_applanix_subscribe_pose_message(&applanix_pose, (dgc_handler_t)
                                      applanix_pose_handler, 
                                      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_localize_subscribe_pose_message(&localize_pose, (dgc_handler_t)
                                      localize_pose_handler,
                                      DGC_SUBSCRIBE_LATEST, NULL);

  dgc_perception_subscribe_obstacles_message(&obstacles, (dgc_handler_t)
					     NULL, DGC_SUBSCRIBE_LATEST, 
					     &obstacles_mutex);
  dgc_perception_subscribe_map_diff_message(NULL, (dgc_handler_t)
					    map_diff_handler,
					    DGC_SUBSCRIBE_ALL, NULL);

  dgc_planner_subscribe_goal_message(NULL, (dgc_handler_t)
				     planner_goal_handler,
				     DGC_SUBSCRIBE_ALL, NULL);
  dgc_ipc_dispatch();
  return 0;
}
