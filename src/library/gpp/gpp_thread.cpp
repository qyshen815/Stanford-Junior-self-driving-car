#include <roadrunner.h>
#include <lltransform.h>
#include <param_interface.h>
#include "gpp.h"
#include "gpp_thread.h"
#include "heuristic.h"
#include "message_queue.h"

using namespace dgc;
using namespace vlr;

/* path planner and current path */

static general_planner *gpp = NULL;
static gpp_path current_path;

/* local copies of essential input data */

static pthread_mutex_t goal_mutex = PTHREAD_MUTEX_INITIALIZER;
static gpp_goal planner_goal, planner_prev_goal;
static int new_goal = 0, received_goal = 0, have_previous_goal = 0;
static int goal_count = 0;

static pthread_mutex_t applanix_mutex = PTHREAD_MUTEX_INITIALIZER;
static double applanix_pose_smooth_x = 0, applanix_pose_smooth_y = 0;
static double applanix_pose_yaw = 0, applanix_pose_speed = 0;
static int received_applanix_pose = 0;

static pthread_mutex_t localize_mutex = PTHREAD_MUTEX_INITIALIZER;
static double localize_x_offset = 0, localize_y_offset = 0;
static int received_localize_pose = 0;

static pthread_mutex_t *obstacle_grid_mutex = NULL;
static dgc_grid_p obstacle_grid = NULL;

static pthread_mutex_t *obstacles_mutex = NULL;
static PerceptionObstacles *obstacles = NULL;

static rndf_file *rndf = NULL;

static pthread_mutex_t boundary_mutex = PTHREAD_MUTEX_INITIALIZER;
static rndf_perimeter **rndf_boundary = NULL;
static rndf_perimeter *custom_boundary = NULL;
static rndf_perimeter *current_boundary = NULL;

static int rndf_door_state = DOORS_CLOSED;
static bool goal_fence_activated = true, start_fence_activated = true;
static bool use_fine = false;
static bool use_smoother = true;
static bool breadth_first = false;

static obstacle_info obs;

static int gpp_pop_limit = 100000;

/* mutexes and cond variables for thread coordination */

int assignment_ready = 0;
pthread_mutex_t ready_mutex;
pthread_cond_t ready_cond;

int assignment_accepted = 0;
pthread_mutex_t accepted_mutex;
pthread_cond_t accepted_cond;

bool planning_complete = false;

status_message_queue *planner_message_queue = NULL;

/* graphics */

vlr::GlsOverlay *gpp_gls = NULL;

void gpp_send_gls(IpcInterface *ipc, 
		  gpp_params *params __attribute__ ((unused)))
{
  if(gpp_gls != NULL)
    glsSend(ipc, gpp_gls);
}

void read_gpp_parameters(ParamInterface *pint, int argc, char **argv, 
			 gpp_params *p)
{
  Param param[] = {
    {"gpp", "astar_xy_resolution", DGC_PARAM_DOUBLE, &p->astar_xy_resolution, 0, NULL},
    {"gpp", "astar_grid_size", DGC_PARAM_DOUBLE, &p->astar_grid_size, 0, NULL},
    {"gpp", "astar_goal_window_deg", DGC_PARAM_DOUBLE, &p->astar_goal_window_deg, 0, NULL},
    {"gpp", "max_steer_deg", DGC_PARAM_DOUBLE, &p->max_steer_deg, 0, NULL},
    {"gpp", "allow_reverse", DGC_PARAM_ONOFF, &p->allow_reverse, 0, NULL},

    {"gpp", "reverse_direction_penalty", DGC_PARAM_DOUBLE, &p->reverse_direction_penalty, 0, NULL},
    {"gpp", "reverse_travel_penalty", DGC_PARAM_DOUBLE, &p->reverse_travel_penalty, 0, NULL},
    {"gpp", "cg_smoothness_gain", DGC_PARAM_DOUBLE, &p->cg_smoothness_gain, 0, NULL},
    {"gpp", "cg_obstacle_gain", DGC_PARAM_DOUBLE, &p->cg_obstacle_gain, 0, NULL},
    {"gpp", "cg_max_obstacle_dist", DGC_PARAM_DOUBLE, &p->cg_max_obstacle_dist, 0, NULL},
    {"gpp", "cg_midpoint_anchor_gain", DGC_PARAM_DOUBLE, &p->cg_midpoint_anchor_gain, 0, NULL},
    {"gpp", "max_vel_mph", DGC_PARAM_DOUBLE, &p->max_vel_mph, 0, NULL},
    {"gpp", "forward_accel", DGC_PARAM_DOUBLE, &p->forward_accel, 0, NULL},
    {"gpp", "lateral_accel", DGC_PARAM_DOUBLE, &p->lateral_accel, 0, NULL},
    {"gpp", "use_rndf_perimeter", DGC_PARAM_ONOFF, &p->use_rndf_perimeter, 0, NULL},
    {"gpp", "use_goal_fence", DGC_PARAM_ONOFF, &p->use_goal_fence, 0, NULL},
    {"gpp", "astar_width_buffer", DGC_PARAM_DOUBLE, &p->astar_width_buffer, 0, NULL},
    {"gpp", "astar_length_buffer", DGC_PARAM_DOUBLE, &p->astar_length_buffer, 0, NULL},
    {"gpp", "pathcheck_width_buffer", DGC_PARAM_DOUBLE, &p->pathcheck_width_buffer, 0, NULL},
    {"gpp", "pathcheck_length_buffer", DGC_PARAM_DOUBLE, &p->pathcheck_length_buffer, 0, NULL},
    {"gpp", "pathcheck_length_buffer", DGC_PARAM_DOUBLE, &p->pathcheck_length_buffer, 0, NULL},
    {"gpp", "heuristic_file", DGC_PARAM_FILENAME, &p->heuristic_filename, 0, NULL},
    {"gpp", "use_gridmap", DGC_PARAM_ONOFF, &p->use_gridmap, 0, NULL},
  };
  pint->InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
}

void add_fence(int *num_points, int *max_points, double **x, double **y,
	       unsigned char **point_type, double fence_x, double fence_y,
	       double fence_theta, double fence_offset, double fence_width,
	       double fence_length)
{
  add_obstacle_line(num_points, max_points, x, y, point_type, 
		    fence_x + fence_offset * cos(fence_theta) - 
		    fence_width * sin(fence_theta),
		    fence_y + fence_offset * sin(fence_theta) + 
		    fence_width * cos(fence_theta),
		    fence_x + fence_offset * cos(fence_theta) + 
		    fence_width * sin(fence_theta),
		    fence_y + fence_offset * sin(fence_theta) - 
		    fence_width * cos(fence_theta),
		    0.4, 3);
  add_obstacle_line(num_points, max_points, x, y, point_type, 
		    fence_x + 
		    (fence_offset - fence_length) * cos(fence_theta) - 
		    fence_width * sin(fence_theta),
		    fence_y + 
		    (fence_offset - fence_length) * sin(fence_theta) + 
		    fence_width * cos(fence_theta),
		    fence_x + fence_offset * cos(fence_theta) - 
		    fence_width * sin(fence_theta),
		    fence_y + fence_offset * sin(fence_theta) + 
		    fence_width * cos(fence_theta),
		    0.4, 3);
  add_obstacle_line(num_points, max_points, x, y, point_type, 
		    fence_x +
		    (fence_offset - fence_length) * cos(fence_theta) + 
		    fence_width * sin(fence_theta),
		    fence_y + 
		    (fence_offset - fence_length) * sin(fence_theta) - 
		    fence_width * cos(fence_theta),
		    fence_x + fence_offset * cos(fence_theta) + 
		    fence_width * sin(fence_theta),
		    fence_y + fence_offset * sin(fence_theta) - 
		    fence_width * cos(fence_theta),
		    0.4, 3);
}

void car_rect(double x, double y, double yaw, double width_buffer,
	      double length_buffer, double rectx[4], double recty[4])
{
  double xc, yc, ctheta, stheta, w2, l2;
  double lf;

  w2 = DGC_PASSAT_WIDTH / 2.0 + width_buffer;
  l2 = DGC_PASSAT_LENGTH / 2.0 + length_buffer;

  lf = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST + 
    length_buffer - l2;

  ctheta = cos(yaw);
  stheta = sin(yaw);
  xc = x + lf * ctheta;
  yc = y + lf * stheta;
  rectx[0] = xc + l2 * ctheta - w2 * stheta;
  recty[0] = yc + l2 * stheta + w2 * ctheta;
  rectx[1] = xc + l2 * ctheta + w2 * stheta;
  recty[1] = yc + l2 * stheta - w2 * ctheta;
  rectx[2] = xc - l2 * ctheta + w2 * stheta;
  recty[2] = yc - l2 * stheta - w2 * ctheta;
  rectx[3] = xc - l2 * ctheta - w2 * stheta;
  recty[3] = yc - l2 * stheta + w2 * ctheta;

}

void raw_obstacles_to_obstacle_list(PerceptionObstacles 
				    *obstacles, obstacle_info *obs, 
				    gpp_goal *goal, gpp_goal *last_goal,
				    rndf_perimeter *boundary,
				    double localize_x_offset, 
				    double localize_y_offset,
				    double rectx[4], double recty[4])
{
  static int num_static_points = 0, max_static_points = 0;
  static double *static_x = NULL, *static_y = NULL;
  static unsigned char *static_type = NULL;
  static bucket_grid *static_bgrid = NULL;
  int i;

  obs->num_obstacles = 0;
  obs->obstacle_x = NULL;
  obs->obstacle_y = NULL;

  num_static_points = 0;

  if(obstacles != NULL) 
    for(i = 0; i < obstacles->num_points; i++) {
      if(!point_inside_poly(rectx, recty, 4, obstacles->point[i].x,
			    obstacles->point[i].y))
	add_obstacle_point(&num_static_points, &max_static_points,
			   &static_x, &static_y, &static_type, 
			   obstacles->point[i].x,
			   obstacles->point[i].y,
			   PERCEPTION_MAP_OBSTACLE_HIGH);
    }

  /* add rndf boundary */
  if(gpp->param.use_rndf_perimeter && boundary != NULL)
    for(i = 0; i < boundary->num_points; i++)
      if(boundary->status[i] == POINT_STATUS_NORMAL ||
	 rndf_door_state == DOORS_CLOSED) 
	add_obstacle_point(&num_static_points, &max_static_points,
			   &static_x, &static_y, &static_type,
			   boundary->rndf_point_x[i] - localize_x_offset,
			   boundary->rndf_point_y[i] - localize_y_offset,
			   3);

  if(gpp->param.use_goal_fence) {
    if(goal_fence_activated)
      for(i = 0; i < goal->num_goals(); i++)
	if(goal->goal[i].use_fence)
	  add_fence(&num_static_points, &max_static_points,
		    &static_x, &static_y, &static_type,
		    goal->goal[i].x, goal->goal[i].y,
		    goal->goal[i].theta, 6, 4, 3);
    if(have_previous_goal && start_fence_activated) {
      for(i = 0; i < last_goal->num_goals(); i++)
	if(last_goal->goal[i].use_fence) 
	  add_fence(&num_static_points, &max_static_points,
		    &static_x, &static_y, &static_type,
		    last_goal->goal[i].x, last_goal->goal[i].y,
		    last_goal->goal[i].theta, 6, 4, 3);
    }
  }

  if(static_bgrid == NULL)
    static_bgrid = new bucket_grid(3, obstacle_grid->cols * 
				   obstacle_grid->resolution,
				   obstacle_grid->rows *
				   obstacle_grid->resolution);
  
  obs->bgrid = static_bgrid;

  obs->bgrid->build_grid(static_x, static_y, num_static_points, 
                         obstacle_grid->map_c0 * obstacle_grid->resolution,
                         obstacle_grid->map_r0 * obstacle_grid->resolution);


  obs->num_obstacles = num_static_points;
  if(obs->num_obstacles == 0) {
    obs->obstacle_x = NULL;
    obs->obstacle_y = NULL;
  }
  else {
    obs->obstacle_x = static_x;
    obs->obstacle_y = static_y;
    obs->obstacle_type = static_type;
  }
}

void grid_to_obstacle_list(dgc_grid_p obstacle_grid, obstacle_info *obs, 
			   gpp_goal *goal, gpp_goal *last_goal, 
			   rndf_perimeter *boundary,
			   double localize_x_offset, double localize_y_offset,
			   double rectx[4], double recty[4])
{
  static int num_static_points = 0, max_static_points = 0;
  static double *static_x = NULL, *static_y = NULL;
  static unsigned char *static_type = NULL;
  static bucket_grid *static_bgrid = NULL;
  PerceptionMapGrid *cell = NULL;
  int i, n;

  obs->num_obstacles = 0;
  obs->obstacle_x = NULL;
  obs->obstacle_y = NULL;

  num_static_points = 0;

  if(obstacle_grid != NULL) {
    cell = (PerceptionMapGrid *)obstacle_grid->cell;
    n = obstacle_grid->rows * obstacle_grid->cols;
    for(i = 0; i < n; i++) {
      if(cell->type != PERCEPTION_MAP_OBSTACLE_FREE &&
	 (cell->type == PERCEPTION_MAP_OBSTACLE_HIGH ||
	  cell->type == PERCEPTION_MAP_OBSTACLE_LOW))
	if(!point_inside_poly(rectx, recty, 4, cell->x, cell->y))
	  add_obstacle_point(&num_static_points, &max_static_points,
			     &static_x, &static_y, &static_type, 
			     cell->x, cell->y, cell->type);
      cell++;
    }
  }
  
  /* add rndf boundary */
  if(gpp->param.use_rndf_perimeter && boundary != NULL)
    for(i = 0; i < boundary->num_points; i++)
      if(boundary->status[i] == POINT_STATUS_NORMAL ||
	 rndf_door_state == DOORS_CLOSED) 
	add_obstacle_point(&num_static_points, &max_static_points,
			   &static_x, &static_y, &static_type,
			   boundary->rndf_point_x[i] - 
			   localize_x_offset,
			   boundary->rndf_point_y[i] - 
			   localize_y_offset,
			   3);

  if(gpp->param.use_goal_fence) {
    if(goal_fence_activated)
      for(i = 0; i < goal->num_goals(); i++)
	if(goal->goal[i].use_fence)
	  add_fence(&num_static_points, &max_static_points,
		    &static_x, &static_y, &static_type,
		    goal->goal[i].x, goal->goal[i].y,
		    goal->goal[i].theta, 6, 4, 3);
    if(have_previous_goal && start_fence_activated)
      for(i = 0; i < last_goal->num_goals(); i++)
	if(last_goal->goal[i].use_fence)
	  add_fence(&num_static_points, &max_static_points,
		    &static_x, &static_y, &static_type,
		    last_goal->goal[i].x, last_goal->goal[i].y,
		    last_goal->goal[i].theta, 6, 4, 3);
  }

  if(static_bgrid == NULL)
    static_bgrid = new bucket_grid(3, obstacle_grid->cols * 
				   obstacle_grid->resolution,
				   obstacle_grid->rows *
				   obstacle_grid->resolution);
  
  obs->bgrid = static_bgrid;

  obs->bgrid->build_grid(static_x, static_y, num_static_points, 
                         obstacle_grid->map_c0 * obstacle_grid->resolution,
                         obstacle_grid->map_r0 * obstacle_grid->resolution);


  obs->num_obstacles = num_static_points;
  if(obs->num_obstacles == 0) {
    obs->obstacle_x = NULL;
    obs->obstacle_y = NULL;
  }
  else {
    obs->obstacle_x = static_x;
    obs->obstacle_y = static_y;
    obs->obstacle_type = static_type;
  }
}

void gpp_initialize(gpp_params *p)
{
  gpp = new general_planner(p);

  gpp->add_action(0.0, 0.2, 10.0);
  gpp->add_action(dgc_d2r(p->max_steer_deg) / 
		  DGC_PASSAT_STEERING_RATIO, 0.2, 10.0);
  gpp->add_action(dgc_d2r(-p->max_steer_deg) / 
		  DGC_PASSAT_STEERING_RATIO, 0.2, 10.0);
  if(p->allow_reverse) {
    gpp->add_action(0.0, 0.2, 10.0, true);
    gpp->add_action(dgc_d2r(p->max_steer_deg) / 
		    DGC_PASSAT_STEERING_RATIO, 0.2, 10.0, true);
    gpp->add_action(dgc_d2r(-p->max_steer_deg) /
		    DGC_PASSAT_STEERING_RATIO, 0.2, 10.0, true);
  }
}

void child_wait_for_assignment(void)
{
  /* wait for the assignment condition variable to go true */
  pthread_mutex_lock(&ready_mutex);
  while(!assignment_ready)
    pthread_cond_wait(&ready_cond, &ready_mutex);

  assignment_ready = 0;
  pthread_mutex_unlock(&ready_mutex);

  /* tell the parent we have accepted the assignment */
  pthread_mutex_lock(&accepted_mutex);
  assignment_accepted = 1;
  pthread_cond_signal(&accepted_cond);
  pthread_mutex_unlock(&accepted_mutex);
}

void parent_wait_for_assignment(void)
{
  /* wait until someone accepts it */
  pthread_mutex_lock(&accepted_mutex);
  while(!assignment_accepted)
    pthread_cond_wait(&accepted_cond, &accepted_mutex);
  assignment_accepted = 0;
  pthread_mutex_unlock(&accepted_mutex);
}

void gls_draw_car(vlr::GlsOverlay *gls, double x, double y,
		  double theta, double min_width_buffer, 
		  double min_length_buffer, double max_width_buffer,
		  double max_length_buffer)
{
  double xc, yc, ctheta, stheta, w2, l2, 
    w2_small, l2_small, w2_large, l2_large, lf;

  w2 = DGC_PASSAT_WIDTH / 2.0;
  l2 = DGC_PASSAT_LENGTH / 2.0;
  w2_small = w2 + min_width_buffer;
  l2_small = l2 + min_length_buffer;
  w2_large = w2 + max_width_buffer;
  l2_large = l2 + max_length_buffer;

  lf = DGC_PASSAT_WHEEL_BASE + DGC_PASSAT_FA_TO_BUMPER_DIST + 
    max_length_buffer - l2_large;

  ctheta = cos(theta);
  stheta = sin(theta);
  xc = x + lf * ctheta;
  yc = y + lf * stheta;

  glsColor3f(gls, 1, 1, 1);
  glsBegin(gls, GLS_LINE_LOOP);
  glsVertex2f(gls, 
		  xc + l2 * ctheta - w2 * stheta,
		  yc + l2 * stheta + w2 * ctheta);
  glsVertex2f(gls, 
		  xc + l2 * ctheta + w2 * stheta,
		  yc + l2 * stheta - w2 * ctheta);
  glsVertex2f(gls, 
		 xc - l2 * ctheta + w2 * stheta,
		 yc - l2 * stheta - w2 * ctheta);
  glsVertex2f(gls, 
		  xc - l2 * ctheta - w2 * stheta,
		  yc - l2 * stheta + w2 * ctheta);
  glsEnd(gls);

  glsColor3f(gls, 1, 0, 0);
  glsBegin(gls, GLS_LINE_LOOP);
  glsVertex2f(gls, 
		  xc + l2_small * ctheta - w2_small * stheta,
		  yc + l2_small * stheta + w2_small * ctheta);
  glsVertex2f(gls, 
		  xc + l2_small * ctheta + w2_small * stheta,
		  yc + l2_small * stheta - w2_small * ctheta);
  glsVertex2f(gls, 
		 xc - l2_small * ctheta + w2_small * stheta,
		 yc - l2_small * stheta - w2_small * ctheta);
  glsVertex2f(gls, 
		  xc - l2_small * ctheta - w2_small * stheta,
		  yc - l2_small * stheta + w2_small * ctheta);
  glsEnd(gls);

  glsColor3f(gls, 0, 0, 1);
  glsBegin(gls, GLS_LINE_LOOP);
  glsVertex2f(gls, 
		  xc + l2_large * ctheta - w2_large * stheta,
		  yc + l2_large * stheta + w2_large * ctheta);
  glsVertex2f(gls, 
		  xc + l2_large * ctheta + w2_large * stheta,
		  yc + l2_large * stheta - w2_large * ctheta);
  glsVertex2f(gls, 
		 xc - l2_large * ctheta + w2_large * stheta,
		 yc - l2_large * stheta - w2_large * ctheta);
  glsVertex2f(gls, 
		  xc - l2_large * ctheta - w2_large * stheta,
		  yc - l2_large * stheta + w2_large * ctheta);
  glsEnd(gls);

}

char *goal_status_string(int x)
{
  if(x == GPP_REACHED_GOAL)
    return "G  ";
  else if(x == GPP_REACHED_SUBGOAL)
    return "SG ";
  else
    return "OT ";
}

void gpp_update_plan_velocities(gpp_path *path)
{
  if(gpp != NULL)
    gpp->recompute_velocities(path, &obs);
}

void *gpp_thread(void *ptr)
{
  gpp_goal goal, prev_goal;
  double smooth_x, smooth_y, applanix_yaw, applanix_vel;
  double loc_x_offset, loc_y_offset;
  bool force_replan = false;
  bool safe, improved;
  gpp_path path_copy;
  double ra_x, ra_y;
  double t1, t2;
  gpp_params *p = (gpp_params *)ptr;
  int i, n;
  int ready_to_close;
  int initialized = 0;
  double current_time, last_plan_ts = 0;
  gpp_path temp_path;
  double rectx[4], recty[4];
  rndf_perimeter boundary;
  int no_boundary;

  do {
    child_wait_for_assignment();
    
    /* move the initialization to after first plan request */
    if(!initialized) {
      gpp_initialize(p);
      initialized = 1;
    }

    /* read applanix pose once */
    pthread_mutex_lock(&applanix_mutex);
    smooth_x = applanix_pose_smooth_x;
    smooth_y = applanix_pose_smooth_y;
    applanix_yaw = applanix_pose_yaw;
    applanix_vel = applanix_pose_speed;
    pthread_mutex_unlock(&applanix_mutex);

    pthread_mutex_lock(&localize_mutex);
    loc_x_offset = localize_x_offset;
    loc_y_offset = localize_y_offset;
    pthread_mutex_unlock(&localize_mutex);

    car_rect(smooth_x, smooth_y, applanix_yaw, gpp->width_buffer,
	     gpp->length_buffer, rectx, recty);

    /* if we have a new goal, start fresh */
    pthread_mutex_lock(&goal_mutex);
    goal = planner_goal;
    for(i = 0; i < goal.num_goals(); i++) {
      goal.goal[i].x -= loc_x_offset;
      goal.goal[i].y -= loc_y_offset;
    }
    prev_goal = planner_prev_goal;
    for(i = 0; i < prev_goal.num_goals(); i++) {
      prev_goal.goal[i].x -= loc_x_offset;
      prev_goal.goal[i].y -= loc_y_offset;
    }
    if(new_goal) {
      current_path.waypoint.clear();
      force_replan = true;
      new_goal = 0;
    }
    pthread_mutex_unlock(&goal_mutex);

    /* close the door if we are inside */
    if(rndf_door_state == DOORS_OPEN_INWARD) {
      for(i = 0; i < rndf->num_zones(); i++)
	if(rndf->zone(i)->car_inside(smooth_x + loc_x_offset,
				     smooth_y + loc_y_offset, 
				     applanix_yaw, 1, 4)) {
	  //	  fprintf(stderr, "CLOSING RNDF DOORS : GOING IN\n");
	  rndf_door_state = DOORS_CLOSED;
	}
    }
    if(rndf_door_state == DOORS_OPEN_OUTWARD) {
      ready_to_close = 1;
      for(i = 0; i < rndf->num_zones(); i++)
	if(!rndf->zone(i)->car_outside(smooth_x + loc_x_offset,
				       smooth_y + loc_y_offset, 
				       applanix_yaw, 1, 4))
	  ready_to_close = 0;
      if(ready_to_close) 
	rndf_door_state = DOORS_CLOSED;
    }

    pthread_mutex_lock(&boundary_mutex);
    no_boundary = 0;
    if(current_boundary == NULL)
      no_boundary = 1;
    else
      boundary = *current_boundary;
    pthread_mutex_unlock(&boundary_mutex);
    
    /* update obstacle list */
    if(p->use_gridmap) {
      pthread_mutex_lock(obstacle_grid_mutex);
      grid_to_obstacle_list(obstacle_grid, &obs, &goal, &prev_goal,
			    (no_boundary) ? NULL : &boundary, 
			    loc_x_offset, loc_y_offset, rectx, recty);
      pthread_mutex_unlock(obstacle_grid_mutex);
    }
    else {
      pthread_mutex_lock(obstacles_mutex);
      raw_obstacles_to_obstacle_list(obstacles, &obs, &goal, &prev_goal,
				     (no_boundary) ? NULL : &boundary, 
				     loc_x_offset, loc_y_offset, rectx, recty);
      pthread_mutex_unlock(obstacles_mutex);
    }

    /* clear GLS message */
    gpp_gls->coordinates = GLS_SMOOTH_COORDINATES;
    gpp_gls->origin_x = smooth_x;
    gpp_gls->origin_y = smooth_y;
    gpp_gls->origin_z = 0;
    gls_clear(gpp_gls);

    //    glsTranslatef(gpp_gls, 0, 0, -DGC_PASSAT_HEIGHT);

    /* compute position of the front axle */
    ra_x = smooth_x + (DGC_PASSAT_IMU_TO_FA_DIST - DGC_PASSAT_WHEEL_BASE) *
      cos(applanix_yaw);
    ra_y = smooth_y + (DGC_PASSAT_IMU_TO_FA_DIST - DGC_PASSAT_WHEEL_BASE) *
      sin(applanix_yaw);

    safe = current_path.safe(&obs, gpp->check_width_buffer, 
			     gpp->check_length_buffer, gpp->width_buffer,
			     gpp->length_buffer);

    current_time = dgc_get_time();

    if(current_path.num_waypoints() == 0 || !safe || 
       !current_path.reached_goal || force_replan) {
      if(fabs(applanix_vel) < dgc_mph2ms(0.5))
	current_path.waypoint.clear();

      t1 = dgc_get_time();
      gpp->plan(ra_x, ra_y, applanix_yaw, false, applanix_vel,
		&goal, use_fine, use_smoother, breadth_first,
		&obs, &current_path, &improved, gpp_pop_limit);
      t2 = dgc_get_time();
      last_plan_ts = dgc_get_time();

      planner_message_queue->add_message("PLAN : O %6d : P %6d : %s : %.0f ms", 
					 obs.num_obstacles, gpp->pop_count, 
					 goal_status_string(gpp->plan_status), 
					 (t2 - t1) * 1000.0);

      force_replan = false;
      gpp->reset_planning = false;
    }
    else if(current_time - last_plan_ts > 1.0) {
      temp_path = current_path;
      if(fabs(applanix_vel) < dgc_mph2ms(0.5))
	temp_path.waypoint.clear();

      t1 = dgc_get_time();
      gpp->plan(ra_x, ra_y, applanix_yaw, false, applanix_vel,
		&goal, use_fine, use_smoother, breadth_first, 
		&obs, &temp_path, &improved, gpp_pop_limit);
      t2 = dgc_get_time();
      last_plan_ts = dgc_get_time();
      
      planner_message_queue->add_message("TEST: O %6d : P %6d : %s : %.0f ms", 
					 obs.num_obstacles, gpp->pop_count, 
					 goal_status_string(gpp->plan_status),
					 (t2 - t1) * 1000.0);

      if(improved) 
	current_path = temp_path;
    }
    else {           /* otherwise just smooth */
      t1 = dgc_get_time();
      //      gpp->smooth(ra_x, ra_y, applanix_yaw, applanix_vel, goal.x, goal.y, 
		  //		  goal.theta, use_smoother, &obs, &current_path);
      gpp->smooth(ra_x, ra_y, applanix_yaw, applanix_vel, 0, 0, 0,
		  use_smoother, &obs, &current_path);
      t2 = dgc_get_time();
    }

    gls_draw_path(gpp_gls, &current_path);
    
    /* draw the goal rndf waypoint */
    if(current_path.goal_rndf_wp != NULL) {
      glsColor3f(gpp_gls, 1, 0, 0);
      glsCircle(gpp_gls, 
		    current_path.goal_rndf_wp->utm_x() - loc_x_offset -
		    gpp_gls->origin_x,
		    current_path.goal_rndf_wp->utm_y() - loc_y_offset -
		    gpp_gls->origin_y, 0.5, 10);
      glsBegin(gpp_gls, GLS_LINES);

      glsVertex2f(gpp_gls, 
		      current_path.goal_rndf_wp->utm_x() - loc_x_offset -
		      gpp_gls->origin_x,
		      current_path.goal_rndf_wp->utm_y() - loc_y_offset -
		      gpp_gls->origin_y);
      glsVertex2f(gpp_gls, 
		      current_path.goal_rndf_wp->utm_x() - loc_x_offset -
		      gpp_gls->origin_x + 0.75 *
		      cos(current_path.goal_rndf_wp->heading()),
		      current_path.goal_rndf_wp->utm_y() - loc_y_offset -
		      gpp_gls->origin_y + 0.75 * 
		      sin(current_path.goal_rndf_wp->heading()));

      glsEnd(gpp_gls);
    }
		    
    n = current_path.num_waypoints();
    if(n > 0)
      gls_draw_car(gpp_gls, 
		   current_path.waypoint[n - 1].smooth_x - gpp_gls->origin_x, 
		   current_path.waypoint[n - 1].smooth_y - gpp_gls->origin_y, 
		   current_path.waypoint[n - 1].smooth_theta,
		   gpp->check_width_buffer, gpp->check_length_buffer,
		   gpp->width_buffer, gpp->length_buffer);

#ifdef DRAW_GLS_MAP
    glsColor3f(gpp_gls, 1, 0.5, 0);
    for(int i = 0; i < obs.num_obstacles; i++)
      if(obs.obstacle_type[i] == PERCEPTION_MAP_OBSTACLE_LOW) 
	glsSquare(gpp_gls, obs.obstacle_x[i] - gpp_gls->origin_x, 
		      obs.obstacle_y[i] - gpp_gls->origin_y, 0.15);
    glsColor3f(gpp_gls, 1, 1, 0);
    for(int i = 0; i < obs.num_obstacles; i++)
      if(obs.obstacle_type[i] == PERCEPTION_MAP_OBSTACLE_HIGH)
	glsSquare(gpp_gls, obs.obstacle_x[i] - gpp_gls->origin_x, 
	obs.obstacle_y[i] - gpp_gls->origin_y, 0.15); 
#endif

    glsColor3f(gpp_gls, 0, 1, 1);
    for(int i = 0; i < obs.num_obstacles; i++)
      if(obs.obstacle_type[i] == 3) 
	glsSquare(gpp_gls, obs.obstacle_x[i] - gpp_gls->origin_x, 
		      obs.obstacle_y[i] - gpp_gls->origin_y, 0.15);
    planning_complete = true;
  } while(1);
  return NULL;
}

void gpp_reset_planning(void)
{
  if(gpp != NULL)
    gpp->reset_planning = true;
}

inline bool my_isnormal2(double x)
{
  int i = std::fpclassify(x);
  if(i == FP_NORMAL || i == FP_ZERO)
    return true;
  else 
    return false;
}

inline char *why_not_normal2(double x)
{
  int i = std::fpclassify(x);
  
  if(i == FP_NAN)
    return "NAN";
  else if(i == FP_INFINITE)
    return "INFINITE";
  else if(i == FP_ZERO)
    return "ZERO";
  else if(i == FP_SUBNORMAL)
    return "SUBNORMAL";
  else
    return "NORMAL";
}

void gpp_set_pose(ApplanixPose *applanix_pose)
{
  pthread_mutex_lock(&applanix_mutex);
  applanix_pose_smooth_x = applanix_pose->smooth_x;
  applanix_pose_smooth_y = applanix_pose->smooth_y;
  applanix_pose_yaw = applanix_pose->yaw;
  applanix_pose_speed = applanix_pose->speed;
  received_applanix_pose = 1;
  if(!my_isnormal2(applanix_pose_smooth_x)) {
    fprintf(stderr, "something wrong with applanix %s\n", 
	    why_not_normal2(applanix_pose_smooth_x));
    if(applanix_pose == NULL)
      fprintf(stderr, "applanix pose is NULL\n");
    else
      fprintf(stderr, "applanix smooth x = %f\n", applanix_pose->smooth_x);
    //    dgc_die("This needs to be fixed\n");
  }
  pthread_mutex_unlock(&applanix_mutex);
}

void gpp_pick_goal(gpp_goal *goal)
{
  pthread_mutex_lock(&goal_mutex);
  if(goal->num_goals() == 0)
    dgc_warning("Setting zero goals!\n");
  if(goal_count >= 1) {
    planner_prev_goal = planner_goal;
    have_previous_goal = 1;
  }
  planner_goal = *goal;
  goal_count++;
  new_goal = 1;
  received_goal = 1;
  pthread_mutex_unlock(&goal_mutex);
}

void gpp_breadth_first(bool active)
{
  breadth_first = active;
}

void gpp_use_fine_grid(void)
{
  use_fine = true;
}

void gpp_use_coarse_grid(void)
{
  use_fine = false;
}

void gpp_use_smoother(int use)
{
  use_smoother = use;
}

void gpp_clear_previous_goals(void)
{
  have_previous_goal = 0;
  goal_count = 0;
}

void gpp_set_pop_limit(int pop_limit)
{
  gpp_pop_limit = pop_limit;
}

void gpp_set_goal_fence(bool active)
{
  goal_fence_activated = active;
}

void gpp_set_start_fence(bool active)
{
  start_fence_activated = active;
}

bool gpp_goal_fence_active(void)
{
  return goal_fence_activated;
}

bool gpp_start_fence_active(void)
{
  return start_fence_activated;
}

bool gpp_boundary_active(void)
{
  return (current_boundary != NULL);
}

void gpp_set_uturn_boundary(rndf_waypoint *w1, rndf_waypoint *w2)
{
  pthread_mutex_lock(&boundary_mutex);
  if(custom_boundary != NULL)
    delete custom_boundary;
  custom_boundary = new rndf_perimeter(w1, w2);
  current_boundary = custom_boundary;
  pthread_mutex_unlock(&boundary_mutex);
}

void gpp_set_zone(int zone_num)
{
  int i;

  pthread_mutex_lock(&boundary_mutex);
  if(rndf != NULL && rndf_boundary == NULL) {
    rndf_boundary = new rndf_perimeter*[rndf->num_zones()];
    for(i = 0; i < rndf->num_zones(); i++) 
      rndf_boundary[i] = new rndf_perimeter(rndf->zone(i));
  }

  if(zone_num == -1) {
    fprintf(stderr, "current boundary = NULL\n");
    current_boundary = NULL;
  }
  else
    current_boundary = rndf_boundary[zone_num];
  pthread_mutex_unlock(&boundary_mutex);
}

void gpp_set_door_state(int state)
{
  /*  if(state == DOORS_OPEN_INWARD)
    fprintf(stderr, "state DOORS_OPEN_INWARD\n");
  else if(state == DOORS_OPEN_OUTWARD)
    fprintf(stderr, "State DOORS_OPEN_OUTWARD\n");
  else
  fprintf(stderr, "state DOORS_CLOSED\n");*/
  rndf_door_state = state;
}

void gpp_set_localize_offset(LocalizePose *localize_pose)
{
  pthread_mutex_lock(&localize_mutex);
  localize_x_offset = localize_pose->x_offset;
  localize_y_offset = localize_pose->y_offset;
  received_localize_pose = 1;
  pthread_mutex_unlock(&localize_mutex);
}

void gpp_start_thread(gpp_params *p, rndf_file *rndf_param,
		      dgc_grid_p obstacle_grid_param, 
		      pthread_mutex_t *obstacle_grid_mutex_param,
		      PerceptionObstacles *obstacles_param,
		      pthread_mutex_t *obstacle_mutex_param,
		      status_message_queue *message_queue)
{
  pthread_t thread;

  gpp_gls = vlr::gls_alloc("GPP");

  planner_message_queue = message_queue;
  rndf = rndf_param;
  obstacle_grid = obstacle_grid_param;
  obstacle_grid_mutex = obstacle_grid_mutex_param;
  obstacles = obstacles_param;
  obstacles_mutex = obstacle_mutex_param;
  pthread_create(&thread, NULL, gpp_thread, (void *)p);
}

void gpp_plan_path(void)
{
  /* if we haven't gotten enough data, just return and say were done */
  if(!received_applanix_pose || !received_localize_pose || !received_goal) {
    planning_complete = true;
    return;
  }

  planning_complete = false;

  /* make a new work assignment available */
  pthread_mutex_lock(&ready_mutex);
  assignment_ready = 1;
  pthread_cond_signal(&ready_cond);
  pthread_mutex_unlock(&ready_mutex);
  parent_wait_for_assignment();
}

bool gpp_plan_ready(gpp_path *path_copy)
{
  if(gpp == NULL)
    return false;
  if(planning_complete) {
    *path_copy = current_path;
    return true;
  }
  else
    return false;
}

void gpp_get_plan_info(double *plan_from_x, double *plan_from_y, 
		       double *plan_from_theta)
{
  *plan_from_x = gpp->last_planned_from_x;
  *plan_from_y = gpp->last_planned_from_y;
  *plan_from_theta = gpp->last_planned_from_theta;
}

