#include <roadrunner.h>
#include <gl_support.h>
#include <rndf.h>
#include <gui3D.h>
#include <imagery.h>
#include <gloverlay.h>
#include <rndfgl.h>
#include <lltransform.h>
#include <grid.h>
#include <textures.h>
#include <trajectory.h>
#ifdef HAVE_VIDEOOUT
#include <videoout.h>
#endif
#include <passat_constants.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <passat_interface.h>
#include <can_interface.h>
#include <perception_interface.h>
#include <param_interface.h>
#include <ibeo_interface.h>
#include <ldlrs_interface.h>
#include <gls_interface.h>
#include <planner_interface.h>
#include <error_interface.h>
#include <controller_interface.h>
#include <simulator_interface.h>
#include <estop_interface.h>
#include <radar_interface.h>
#include "project_sensors.h"
#include "clientmap.h"

#include <strings.h>
#include <wordexp.h>

using namespace std;
using namespace dgc;

/* params */

char *imagery_root, *gloverlay_filename, *rndf_filename;

/* vars */

int received_applanix_pose = 0;
dgc_applanix_pose_message applanix_pose;
double applanix_x, applanix_y;
double gps_offset_x, gps_offset_y;
double wheel_angle;
char utmzone[5];

int received_localize_pose = 0;
dgc_localize_pose_message localize_pose;

int received_ibeo1 = 0;
projected_ibeo_p ibeo1_global = NULL;
dgc_transform_t ibeo1_offset;
pthread_mutex_t ibeo1_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_ibeo2 = 0;
projected_ibeo_p ibeo2_global = NULL;
dgc_transform_t ibeo2_offset;
pthread_mutex_t ibeo2_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_ldlrs1 = 0;
projected_scan_p ldlrs1_global = NULL;
dgc_transform_t ldlrs1_offset;
pthread_mutex_t ldlrs1_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_ldlrs2 = 0;
projected_scan_p ldlrs2_global = NULL;
dgc_transform_t ldlrs2_offset;
pthread_mutex_t ldlrs2_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
  int radar_num, id;
  int track_count;
  bool measured, historical;
  float lateral_offset, distance;
  float lateral_offset_var, relative_acceleration, relative_velocity;
  double x, y, v_x, v_y;
} radar_target_t, *radar_target_p;

int received_radar1 = 0;
dgc_transform_t radar1_offset;
radar_target_t radar1_track[64];
int radar1_num_targets = 0;
pthread_mutex_t radar1_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_radar2 = 0;
dgc_transform_t radar2_offset;
radar_target_t radar2_track[64];
int radar2_num_targets = 0;
pthread_mutex_t radar2_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_radar3 = 0;
dgc_transform_t radar3_offset;
radar_target_t radar3_track[64];
int radar3_num_targets = 0;
pthread_mutex_t radar3_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_radar4 = 0;
dgc_transform_t radar4_offset;
radar_target_t radar4_track[64];
int radar4_num_targets = 0;
pthread_mutex_t radar4_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_radar5 = 0;
dgc_transform_t radar5_offset;
radar_target_t radar5_track[64];
int radar5_num_targets = 0;
pthread_mutex_t radar5_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_passat_actuator = 0;
dgc_passat_actuator_message passat_actuator;

int received_passat_signal = 0;
dgc_passat_turnsignal_message passat_signal;
double signal_ts = 0;

int received_map = 0, map_updated = 0;
pthread_mutex_t map_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_gls = 0;
vector <dgc_gls_overlay_message *> gls_cache;
static pthread_mutex_t gls_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_planner_trajectory = 0;
dgc_planner_trajectory_message planner_trajectory;
static pthread_mutex_t planner_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_controller_target = 0;
dgc_controller_target_message controller_target;

int received_simdata = 0;
dgc_simulator_groundtruth_message simdata;
static pthread_mutex_t simulator_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_obstaclelist = 0;
dgc_perception_obstaclelist_message obstaclelist;
static pthread_mutex_t obstaclelist_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_stop_zones = 0;
dgc_perception_stop_zones_message stop_zones;
static pthread_mutex_t stop_zones_mutex = PTHREAD_MUTEX_INITIALIZER;

dgc_perception_obstacles_message obstacles;
int received_obstacles = 0;
static pthread_mutex_t obstacles_mutex = PTHREAD_MUTEX_INITIALIZER;

dgc_planner_goal_message planner_goal;
int received_planner_goal = 0;

dgc_estop_status_message estop;
int received_estop = 0;

rndf_file *rndf = NULL;
int rndf_valid = 0;

dgc_path2D_t vehicle_path, smooth_path;

dgc_grid_p grid = NULL;

/* graphics */

int add_offset = 0;
int use_filter = 1;
int show_ibeo1 = 1;
int show_ibeo2 = 1;
int show_ldlrs1 = 1;
int show_ldlrs2 = 1;
int show_rndf = 1;
int draw_edges = 1;
int draw_rings = 0;
int game_mode = 0;
GLint gloverlay = -1;
int camera_unlocked = 0;
double gloverlay_origin_x, gloverlay_origin_y;

rndf_display_list *rndf_dl = NULL;

dgc_gl_texture_p map_texture = NULL;
dgc_trajectory_p reference_traj = NULL;

#define  MESSAGE_LEVEL_STATUS  0
#define  MESSAGE_LEVEL_ERROR   1

typedef struct {
  char level;
  double timestamp;
  char *string;
} status_message_t;

#define MESSAGE_BUFFER_SIZE 5
status_message_t message_buffer[MESSAGE_BUFFER_SIZE];
int num_messages = 0;
static pthread_mutex_t message_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
  char *name;
  int id;
  char *text;
} gls_text_t;

vector <gls_text_t> gls_text;
static pthread_mutex_t gls_text_mutex = PTHREAD_MUTEX_INITIALIZER;

int record_video = 0;
#ifdef HAVE_VIDEOOUT
dgc_videoout_p vo = NULL;
#endif

double goal_lat = 0, goal_lon = 0, goal_theta = 0;
double goal_x = 0, goal_y = 0;

void keyboard(unsigned char key, int x, int y)
{
  static double last_x = 0, last_y = 0;
  double x2, y2, utm_x, utm_y;
  double lat, lon;

  gui3D_get_2D_position(x, y, &x2, &y2);
  utm_x = x2 + applanix_x;
  utm_y = y2 + applanix_y;
  utmToLatLong(utm_x, utm_y, utmzone, &lat, &lon);

  switch(key) {
  case 27: case 'q': case 'Q':
#ifdef HAVE_VIDEOOUT
    if(record_video)
      dgc_videoout_release(&vo);
#endif
    exit(0);
    break;
  case '0':
    draw_rings = !draw_rings;
    break;
  case '1':
    show_ibeo1 = !show_ibeo1;
    break;
  case '2':
    show_ibeo2 = !show_ibeo2;
    break;
  case '3':
    show_ldlrs1 = !show_ldlrs1;
    break;
  case '4':
    show_ldlrs2 = !show_ldlrs2;
    break;
  case 'o': case 'O':
    add_offset = !add_offset;
    break;
  case 'n': case 'N':
    show_rndf = !show_rndf;
    break;
  case 'g': case 'G':
    if(glutGetModifiers() == GLUT_ACTIVE_ALT) 
      dgc_planner_trigger_command(DGC_PLANNER_ALLOW_MOVEMENT);
    else {
      if(key == 'g') {
	goal_lat = lat;
	goal_lon = lon;
	goal_x = utm_x;
	goal_y = utm_y;
	last_x = utm_x;
	last_y = utm_y;
      }
      else if(key == 'G') {
	goal_theta = atan2(utm_y - last_y, utm_x - last_x);
	//	dgc_planner_goal_command(goal_lat, goal_lon, goal_theta);
      }
    }
    break;
  case 's':
    dgc_planner_goal_command(goal_lat, goal_lon, goal_theta);
    break;
  case 'e': case 'E':
    draw_edges = !draw_edges;
    break;
  case 'c': case 'C':
    gui3D_recenter_2D();
    break;
  case 'p': case 'P':
    if(glutGetModifiers() == GLUT_ACTIVE_ALT && received_estop) {
      if(estop.estop_code == DGC_ESTOP_PAUSE)
	dgc_estop_softstop_command(DGC_ESTOP_RUN);
      else if(estop.estop_code == DGC_ESTOP_RUN)
	dgc_estop_softstop_command(DGC_ESTOP_PAUSE);
    }
    else {
      dgc_path2D_reset(&vehicle_path);
      dgc_path2D_reset(&smooth_path);
    }
    break;
  case 'u': case 'U':
    camera_unlocked = !camera_unlocked;
    break;
  case 'r': case 'R':
    record_video = 1 - record_video;
#ifdef HAVE_VIDEOOUT
    if(record_video) {
      vo  = dgc_videoout_init(dgc_unique_filename("map_view.avi"),
			      60 * gui3D.window_width *
			      gui3D.window_height, gui3D.window_width,
			      gui3D.window_height, 10, CODEC_ID_MPEG4,
			      0, PIX_FMT_YUV420P);
    }
    else
      dgc_videoout_release(&vo);
#endif
    break;  
  case 'f': case 'F':
    use_filter = !use_filter;
    break;
  case 'l': case 'L':
    game_mode = !game_mode;
    break;
  case 'i': case 'I':
    dgc_imagery_cycle_imagery_type();
    break;
  }
  gui3D_forceRedraw();
}

void draw_left_turn_signal(float x, float y, float r)
{
  glColor3f(0, 1, 0);
  glPushMatrix();
  glTranslatef(x - r / 2, y - r / 2, 0);
  glScalef(r / 2, r / 2, 1);
  glBegin(GL_POLYGON);
  glVertex2f(0, 1);
  glVertex2f(-1, 0);
  glVertex2f(0, -1);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex2f(0, 0.4);
  glVertex2f(1, 0.4);
  glVertex2f(1, -0.4);
  glVertex2f(0, -0.4);
  glEnd();
  glPopMatrix();
}

void draw_right_turn_signal(float x, float y, float r)
{
  glColor3f(0, 1, 0);
  glPushMatrix();
  glTranslatef(x - r / 2, y - r / 2, 0);
  glScalef(r / 2, r / 2, 1);
  glBegin(GL_POLYGON);
  glVertex2f(0, 1);
  glVertex2f(1, 0);
  glVertex2f(0, -1);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex2f(0, 0.4);
  glVertex2f(-1, 0.4);
  glVertex2f(-1, -0.4);
  glVertex2f(0, -0.4);
  glEnd();
  glPopMatrix();
}

void draw_pedals(float x, float y, float w, float h, double throttle,
		 double brake)
{
  int i;

  glLineWidth(1.0);
  glColor3f(1, 1, 1);
  glBegin(GL_LINES);
  for(i = 1; i <= 9; i++) {
    glVertex2f(x, y + h * i / 10.0);
    glVertex2f(x + w, y + h * i / 10.0);
    glVertex2f(x + 2 * w, y + h * i / 10.0);
    glVertex2f(x + 3 * w, y + h * i / 10.0);
  }
  glEnd();

  glLineWidth(2.0);
  glColor3f(0, 1, 0);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x + w, y);
  glVertex2f(x + w, y + throttle * h);
  glVertex2f(x, y + throttle * h);
  glEnd();
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex2f(x + 2 * w, y);
  glVertex2f(x + 3 * w, y);
  glVertex2f(x + 3 * w, y + brake / 100.0 * h);
  glVertex2f(x + 2 * w, y + brake / 100.0 * h);
  glEnd();

  glColor3f(1, 1, 1);
  glBegin(GL_LINE_LOOP);
  glVertex2f(x, y);
  glVertex2f(x + w, y);
  glVertex2f(x + w, y + h);
  glVertex2f(x, y + h);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex2f(x + 2 * w, y);
  glVertex2f(x + 3 * w, y);
  glVertex2f(x + 3 * w, y + h);
  glVertex2f(x + 2 * w, y + h);
  glEnd();
  glLineWidth(0.0);
}

void draw_steering_wheel(float x, float y, float r, float steering_angle)
{
  int i;
  double angle;

  glLineWidth(2.0);
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 1.1 * r * cos(angle), y + 1.1 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 0.9 * r * cos(angle), y + 0.9 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + r / 3.0 * cos(angle), y + r / 3.0 * sin(angle));
  }
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(x + r / 3.0 * cos(steering_angle), 
             y + r / 3.0 * sin(steering_angle));
  glVertex2f(x + 0.9 * r * cos(steering_angle), 
             y + 0.9 * r * sin(steering_angle));
  glVertex2f(x - r / 3.0 * cos(steering_angle), 
             y - r / 3.0 * sin(steering_angle));
  glVertex2f(x - 0.9 * r * cos(steering_angle), 
             y - 0.9 * r * sin(steering_angle));
  glEnd();
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex2f(x + 0.9 * r * cos(steering_angle + M_PI_2),
             y + 0.9 * r * sin(steering_angle + M_PI_2));
  glVertex2f(x + 1.1 * r * cos(steering_angle + M_PI_2),
             y + 1.1 * r * sin(steering_angle + M_PI_2));
  glEnd();
  glLineWidth(1.0);
}

void draw_trajectory_parallel(dgc_trajectory_p trajectory, double par_dist,
			      double origin_x, double origin_y)
{
  int i;

  glBegin(GL_LINE_STRIP);
  for(i = 0; i < trajectory->num_waypoints; i++)
    glVertex2f(trajectory->waypoint[i].x - origin_x + par_dist *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y + par_dist *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
  glEnd();
}

void highlight_trajectory(dgc_trajectory_p trajectory, double w,
			  double origin_x, double origin_y)
{
  int i;

  glBegin(GL_QUAD_STRIP);
  for(i = 0; i < trajectory->num_waypoints; i++) {
    glVertex2f(trajectory->waypoint[i].x - origin_x + w *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y + w *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
    glVertex2f(trajectory->waypoint[i].x - origin_x - w *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y - w *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
  }
  glEnd();
}

inline void draw_vehicle_cage(double x, double y, double theta,
			      double w, double l,int id, double v,
			      double range, int draw_flag)
{
  char line[100];

  glLineWidth(2.0);
  glPushMatrix();

  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  /* draw car outline */
  glBegin(GL_LINE_LOOP);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(-l / 2, -w / 2, DGC_PASSAT_HEIGHT);
  glVertex3f(l / 2, 0, DGC_PASSAT_HEIGHT);
  glVertex3f(-l / 2, w / 2, DGC_PASSAT_HEIGHT);
  glVertex3f(l / 2, 0, DGC_PASSAT_HEIGHT);
  glEnd();


  /* draw info flag */
  if(draw_flag) {
    
    double boxh2 = 0.7;
    
    glColor4f(0, 0, 0, 0.4);
    glBegin(GL_POLYGON);
    glVertex2f(-boxh2, -1);
    glVertex2f(-boxh2, 1);
    glVertex2f(boxh2, 1);
    glVertex2f(boxh2, -1);
    glEnd();
    glColor3f(0.7, 0.7, 0.7);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-boxh2, -1);
    glVertex2f(-boxh2, 1);
    glVertex2f(boxh2, 1);
    glVertex2f(boxh2, -1);
    glEnd();

    glRotatef(-90, 0, 0, 1);
    glColor3f(1, 1, 1);

    sprintf(line, "%d", id);
    render_stroke_text_centered_2D(0, 0.4,
				   GLUT_STROKE_ROMAN, 0.3, line);
    sprintf(line, "%.1fmph", v);
    render_stroke_text_centered_2D(0, 0.0,
				   GLUT_STROKE_ROMAN, 0.3, line);
    sprintf(line, "%.1fm", range);
    render_stroke_text_centered_2D(0, -0.4,
				   GLUT_STROKE_ROMAN, 0.3, line);
  }

  glPopMatrix();
  glLineWidth(1.0);
}

void draw_square(float x, float y, float w)
{
  float w2 = w / 2.0;

  glBegin(GL_QUADS);
  glVertex2f(x - w2, y - w2);
  glVertex2f(x + w2, y - w2);
  glVertex2f(x + w2, y + w2);
  glVertex2f(x - w2, y + w2);
  glEnd();
}

void draw_planner_goal(double x, double y, double theta)
{
  glColor4f(0, 1, 0, 0.5);
  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);
  glTranslatef(DGC_PASSAT_WHEEL_BASE - DGC_PASSAT_IMU_TO_FA_DIST, 0, 0);
  draw_passat_outline(0);
  glPopMatrix();
}

void draw_radar_target(radar_target_p target, double origin_x, 
		       double origin_y, float r, float g, float b)
{
  double angle;
  //  char str[100];
  int i;

  if(target->track_count < 3)
    return;

  glColor4f(r, g, b, 0.5);
  glBegin(GL_POLYGON);
  for(i = 0; i < 20; i++) {
    angle = i / 20.0 * 2 * M_PI;
    glVertex2f(target->x - origin_x + 0.5 * cos(angle), 
	       target->y - origin_y + 0.5 * sin(angle));
  }
  glEnd();

  glColor3f(r, g, b);
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 20; i++) {
    angle = i / 20.0 * 2 * M_PI;
    glVertex2f(target->x - origin_x + 0.5 * cos(angle), 
	       target->y - origin_y + 0.5 * sin(angle));
  }
  glEnd();

  glBegin(GL_LINES);
  glVertex2f(target->x - origin_x, target->y - origin_y);
  glVertex2f(target->x + target->v_x - origin_x, 
	     target->y + target->v_y - origin_y);
  glEnd();

  /*
  glColor3f(0, 0, 0);

  sprintf(str, "%d: %d %d (%d %d) %.2f %.2f %.1f mph", target->radar_num,
	  target->id, target->track_count, 
	  target->measured, target->historical, 
	  target->distance, target->lateral_offset,
	  dgc_ms2mph(target->relative_velocity));

  render_stroke_text_centered_2D(target->x - origin_x, 
				 target->y - origin_y + 0.75,
				 GLUT_STROKE_ROMAN, 0.3, str); */
}

void display(void)
{
  double smooth_x_copy, smooth_y_copy, smooth_z_copy;
  double applanix_x_copy, applanix_y_copy, applanix_z_copy, applanix_yaw_copy;
  double applanix_roll_copy, applanix_pitch_copy;
  double origin_x, origin_y, x_offset_copy, y_offset_copy;
  double w, max_width, current_time;
  //  double goal_x, goal_y;
  //  char goal_utmzone[10];
  char str[100];
  int i;

  /* make a copy of coordinates - multi-threaded issue */
  smooth_x_copy = applanix_pose.smooth_x;
  smooth_y_copy = applanix_pose.smooth_y;
  smooth_z_copy = applanix_pose.smooth_z;
  applanix_x_copy = applanix_x;
  applanix_y_copy = applanix_y;
  applanix_z_copy = applanix_pose.altitude;
  applanix_roll_copy = applanix_pose.roll;
  applanix_pitch_copy = applanix_pose.pitch;
  applanix_yaw_copy = applanix_pose.yaw;
  x_offset_copy = localize_pose.x_offset;
  y_offset_copy = localize_pose.y_offset;

  /* pick global coordinate frame */
  if(add_offset) {
    origin_x = smooth_x_copy + localize_pose.x_offset;
    origin_y = smooth_y_copy + localize_pose.y_offset;
  }
  else {
    origin_x = applanix_x_copy;
    origin_y = applanix_y_copy;
  }

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT);

  /* draw the imagery */
  dgc_imagery_draw_2D(imagery_root, gui3D.window_width, gui3D.window_height,
		      gui3D.camera_pose.zoom, origin_x, origin_y,
		      gui3D.camera_pose.x_offset_2D,
		      gui3D.camera_pose.y_offset_2D, utmzone, 1);
  
  /* draw the map */
  if(received_map) {
    /* allocate the map texture in graphics thread (important) */
    if(map_texture == NULL)
      map_texture = dgc_gl_empty_texture_rgba(grid->cols, grid->rows);

    /* update the texture from the rolling grid */
    if(map_updated) {
      pthread_mutex_lock(&map_mutex);
      dgc_clientmap_to_rgba_texture(grid, map_texture);
      pthread_mutex_unlock(&map_mutex);
      dgc_gl_update_texture_rgba(map_texture);
      map_updated = 0;
    }

    /* draw the map texture */
    glPushMatrix();
    glTranslatef(grid->map_c0 * grid->resolution - smooth_x_copy,
                 grid->map_r0 * grid->resolution - smooth_y_copy, 0);
    dgc_gl_draw_texture(map_texture, 0, 0,
			grid->cols * grid->resolution, 
			grid->rows * grid->resolution);
			     
    /* draw map boundary in yellow */
    glColor3f(1, 1, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(0, 0);
    glVertex2f(grid->cols * grid->resolution, 0);
    glVertex2f(grid->cols * grid->resolution, grid->rows * grid->resolution);
    glVertex2f(0, grid->rows * grid->resolution);
    glEnd();
    glPopMatrix();
  }

  /* draw the stop zones */
  if(received_stop_zones) {
    pthread_mutex_lock(&stop_zones_mutex);
    for(i = 0; i < stop_zones.num_zones; i++) {
      if(stop_zones.zone[i].state == ZONE_STATE_FREE)
	glColor4f(0, 1, 0, 0.5);
      else if(stop_zones.zone[i].state == ZONE_STATE_OCCUPIED)
	glColor4f(1, 0, 0, 0.5);
      else
	glColor4f(1, 1, 0, 0.5);
      glPushMatrix();
      glTranslatef(stop_zones.zone[i].utm_x - origin_x, 
		   stop_zones.zone[i].utm_y - origin_y, 0.5);
      glRotatef(dgc_r2d(stop_zones.zone[i].heading), 0, 0, 1);
      glTranslatef(-(stop_zones.zone[i].length / 2.0 - 1.0), 0, 0);
		   
      glBegin(GL_POLYGON);
      glVertex2f(-stop_zones.zone[i].length / 2.0, 
		 -stop_zones.zone[i].width / 2.0);
      glVertex2f(stop_zones.zone[i].length / 2.0, 
		 -stop_zones.zone[i].width / 2.0);
      glVertex2f(stop_zones.zone[i].length / 2.0, 
		 stop_zones.zone[i].width / 2.0);
      glVertex2f(-stop_zones.zone[i].length / 2.0, 
		 stop_zones.zone[i].width / 2.0);
      glEnd();
      glPopMatrix();
    }
    pthread_mutex_unlock(&stop_zones_mutex);
  }

  /* draw the rndf */
  if(rndf_valid && show_rndf) {
    if(rndf_dl == NULL)
      rndf_dl = generate_rndf_display_list(rndf, 1.0);
    draw_rndf_display_list(rndf_dl, 0, 1, 0, 1, 0, 1, origin_x, origin_y);
  }
  glLineWidth(2.0);

  /* draw the GLoverlay */
  if(gloverlay != -1 && draw_edges) {
    glPushMatrix();
    glColor3f(1, 1, 1);
    glTranslatef(gloverlay_origin_x - origin_x, 
                 gloverlay_origin_y - origin_y, 0.0);
    dgc_gloverlay_draw(gloverlay);
    glPopMatrix();
  }

#ifdef blah
  /* draw the reference trajectory */
  if(reference_traj != NULL) {
    glColor4f(1, 0, 0, 0.5);
    highlight_trajectory(reference_traj, 1, origin_x, origin_y);
  }
#endif

  glColor3f(1, 0, 0);
  dgc_path2D_draw(&vehicle_path, -origin_x, -origin_y);
  glColor3f(1, 1, 0);
  dgc_path2D_draw(&smooth_path, -smooth_x_copy, -smooth_y_copy);

  /* draw the GL over IPC stuff */
  if(received_gls) {
    pthread_mutex_lock(&gls_mutex);
    for(i = 0; i < (signed)gls_cache.size(); i++) 
      gls_draw(gls_cache[i], origin_x, origin_y, 0,
               applanix_roll_copy, applanix_pitch_copy, applanix_yaw_copy,
               smooth_x_copy, smooth_y_copy, smooth_z_copy);
    pthread_mutex_unlock(&gls_mutex);
  }

  /* draw the vehicle */
  glPushMatrix();
  glColor3f(0, 0, 0);
  glRotatef(dgc_r2d(applanix_yaw_copy), 0, 0, 1);
  glColor4f(0.25, 0.25, 1, 0.5);
  draw_passat_outline(wheel_angle);

  glColor3f(1, 0, 0);
  draw_circle(0, 0, 0.1);
  glColor3f(0, 1, 0);
  draw_circle(DGC_PASSAT_IMU_TO_FA_DIST - DGC_PASSAT_WHEEL_BASE, 0, 0.1);
  glColor3f(0, 0, 1);
  draw_circle(DGC_PASSAT_IMU_TO_FA_DIST, 0, 0.1);

  /*
  glColor3f(1, 1, 0);
  glTranslatef(DGC_PASSAT_IMU_TO_CG_DIST + .2, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-DGC_PASSAT_LENGTH / 2.0, -DGC_PASSAT_WIDTH / 2.0 - 1, 0);
  glVertex3f(DGC_PASSAT_LENGTH / 2.0, -DGC_PASSAT_WIDTH / 2.0 - 1, 0);
  glVertex3f(DGC_PASSAT_LENGTH / 2.0, DGC_PASSAT_WIDTH / 2.0 + 1, 0);
  glVertex3f(-DGC_PASSAT_LENGTH / 2.0, DGC_PASSAT_WIDTH / 2.0 + 1, 0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(-DGC_PASSAT_LENGTH / 2.0, -DGC_PASSAT_WIDTH / 2.0, 0);
  glVertex3f(DGC_PASSAT_LENGTH / 2.0, -DGC_PASSAT_WIDTH / 2.0, 0);
  glVertex3f(DGC_PASSAT_LENGTH / 2.0, DGC_PASSAT_WIDTH / 2.0, 0);
  glVertex3f(-DGC_PASSAT_LENGTH / 2.0, DGC_PASSAT_WIDTH / 2.0, 0);
  glEnd();
  */
  glPopMatrix();


  /* draw range rings */
  if(draw_rings) {
    glColor3f(1, 1, 0);
    for(i = 1; i <= 10; i++) {
      int j;
      double angle;
      glBegin(GL_LINE_LOOP);
      for(j = 0; j < 100; j++) {
	angle = j / 100.0 * M_PI * 2;
	glVertex2f(cos(angle) * i * 10, sin(angle) * i * 10);
      }
      glEnd();
    }
  }

  /* draw the other vehicles */
  pthread_mutex_lock(&simulator_mutex);
  for(i = 0; i < simdata.num_vehicles; i++) 
    if(i != simdata.our_vehicle_num) {
      glPushMatrix();
      glTranslatef(simdata.vehicle[i].x - origin_x,
		   simdata.vehicle[i].y - origin_y, 0);
      glRotatef(dgc_r2d(simdata.vehicle[i].theta), 0, 0, 1);
      glTranslatef(DGC_PASSAT_LENGTH / 2.0 - DGC_PASSAT_FA_TO_BUMPER_DIST -
		   DGC_PASSAT_IMU_TO_FA_DIST, 0, 0);
      if(i % 4 == 0)
	glColor3f(1, 0, 0);
      else if(i % 4 == 1)
	glColor3f(1, 1, 1);
      else if(i % 4 == 2)
	glColor3f(0.5, 0.5, 0.5);
      else if(i % 4 == 3)
	glColor3f(0, 0.7, 0);
      draw_passat_outline(0);
      
      glColor3f(0, 0, 0);
      draw_circle(0, 0, 0.1);
      
      glPopMatrix();
    }
  pthread_mutex_unlock(&simulator_mutex);

  /* draw the obstaclelist */
  if(0&&received_obstaclelist) {
    pthread_mutex_lock(&obstaclelist_mutex);
    for(i = 0; i < obstaclelist.num_obstacles; i++) {
      if(obstaclelist.obstacle[i].obstacleType == DGC_DYNAMIC_OBSTACLE) 
        glColor3f(0.5, 0.5, 1);
      else
        glColor3f(1, 0, 1);
      draw_vehicle_cage(obstaclelist.obstacle[i].x - smooth_x_copy,
			obstaclelist.obstacle[i].y - smooth_y_copy,
			obstaclelist.obstacle[i].direction,
			obstaclelist.obstacle[i].width,
			obstaclelist.obstacle[i].length,
			obstaclelist.obstacle[i].id,
			dgc_ms2mph(obstaclelist.obstacle[i].velocity), 
                        hypot(obstaclelist.obstacle[i].x - smooth_x_copy,
                              obstaclelist.obstacle[i].y - smooth_y_copy),
                        (obstaclelist.obstacle[i].obstacleType == 
                         DGC_DYNAMIC_OBSTACLE));
    }
    pthread_mutex_unlock(&obstaclelist_mutex);
  }

  if(received_obstacles) {
    pthread_mutex_lock(&obstacles_mutex);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    for(i = 0; i < obstacles.num_points; i++)
      if(obstacles.point[i].type == 0) {
	glVertex2f(obstacles.point[i].x - smooth_x_copy + 0.15,
		   obstacles.point[i].y - smooth_y_copy);
	glVertex2f(obstacles.point[i].x - smooth_x_copy - 0.15,
		   obstacles.point[i].y - smooth_y_copy);
	glVertex2f(obstacles.point[i].x - smooth_x_copy,
		   obstacles.point[i].y - smooth_y_copy - 0.15);
	glVertex2f(obstacles.point[i].x - smooth_x_copy,
		   obstacles.point[i].y - smooth_y_copy + 0.15);
      }
    glColor3f(1, 1, 0);
    for(i = 0; i < obstacles.num_points; i++)
      if(obstacles.point[i].type != 0) {
	glVertex2f(obstacles.point[i].x - smooth_x_copy + 0.15,
		   obstacles.point[i].y - smooth_y_copy);
	glVertex2f(obstacles.point[i].x - smooth_x_copy - 0.15,
		   obstacles.point[i].y - smooth_y_copy);
	glVertex2f(obstacles.point[i].x - smooth_x_copy,
		   obstacles.point[i].y - smooth_y_copy - 0.15);
	glVertex2f(obstacles.point[i].x - smooth_x_copy,
		   obstacles.point[i].y - smooth_y_copy + 0.15);
      }
    glEnd();

    glColor3f(0.5, 0.5, 1);
    for(i = 0; i < obstacles.num_dynamic_obstacles; i++) {
      draw_vehicle_cage(obstacles.dynamic_obstacle[i].x - smooth_x_copy,
			obstacles.dynamic_obstacle[i].y - smooth_y_copy,
			obstacles.dynamic_obstacle[i].direction,
			obstacles.dynamic_obstacle[i].width,
			obstacles.dynamic_obstacle[i].length,
			obstacles.dynamic_obstacle[i].id,
			dgc_ms2mph(obstacles.dynamic_obstacle[i].velocity), 
                        hypot(obstacles.dynamic_obstacle[i].x - smooth_x_copy,
                              obstacles.dynamic_obstacle[i].y - smooth_y_copy),
                        1);
    }

    pthread_mutex_unlock(&obstacles_mutex);
  }

  /* draw the IBEO scan */
  if(received_ibeo1 && show_ibeo1) {
    pthread_mutex_lock(&ibeo1_mutex);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for(i = 0; i < ibeo1_global->num_points; i++) 
      if(ibeo1_global->point[i].status == DGC_IBEO_STATUS_OK &&
	 ibeo1_global->point[i].level < 4) {
	if(ibeo1_global->point[i].level % 4 == 0)
	  glColor3f(1, 0, 0);
	else if(ibeo1_global->point[i].level % 4 == 1)
	  glColor3f(0, 1, 0);
	else if(ibeo1_global->point[i].level % 4 == 2)
	  glColor3f(0, 0, 1);
	else if(ibeo1_global->point[i].level % 4 == 3)
	  glColor3f(1, 1, 0);
	glVertex2f(ibeo1_global->point[i].x - smooth_x_copy,
		   ibeo1_global->point[i].y - smooth_y_copy);
      }
    glEnd();
    glPointSize(1.0);
    pthread_mutex_unlock(&ibeo1_mutex);
  }
  if(received_ibeo2 && show_ibeo2) {
    pthread_mutex_lock(&ibeo2_mutex);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for(i = 0; i < ibeo2_global->num_points; i++) 
      if(ibeo2_global->point[i].status == DGC_IBEO_STATUS_OK &&
	 ibeo2_global->point[i].level < 4) {
	if(ibeo2_global->point[i].level % 4 == 0)
	  glColor3f(1, 0, 0);
	else if(ibeo2_global->point[i].level % 4 == 1)
	  glColor3f(0, 1, 0);
	else if(ibeo2_global->point[i].level % 4 == 2)
	  glColor3f(0, 0, 1);
	else if(ibeo2_global->point[i].level % 4 == 3)
	  glColor3f(1, 1, 0);
	glVertex2f(ibeo2_global->point[i].x - smooth_x_copy,
		   ibeo2_global->point[i].y - smooth_y_copy);
      }
    glEnd();
    glPointSize(1.0);
    pthread_mutex_unlock(&ibeo2_mutex);
  }

  /* draw the LDLRS scans */
  if(received_ldlrs1 && show_ldlrs1) {
    pthread_mutex_lock(&ldlrs1_mutex);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    glColor3f(0, 1, 1);
    for(i = 0; i < ldlrs1_global->num_points; i++) 
      //      if(ldlrs1_global->point[i].useful) 
	glVertex2f(ldlrs1_global->point[i].x - smooth_x_copy,
		   ldlrs1_global->point[i].y - smooth_y_copy);
    glEnd();
    glPointSize(1.0);
    pthread_mutex_unlock(&ldlrs1_mutex);
  }
  if(received_ldlrs2 && show_ldlrs2) {
    pthread_mutex_lock(&ldlrs2_mutex);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    glColor3f(1, 0, 1);
    for(i = 0; i < ldlrs2_global->num_points; i++) 
      //      if(ldlrs2_global->point[i].useful) 
	glVertex2f(ldlrs2_global->point[i].x - smooth_x_copy,
		   ldlrs2_global->point[i].y - smooth_y_copy);
    glEnd();
    glPointSize(1.0);
    pthread_mutex_unlock(&ldlrs2_mutex);
  }

  //  if(received_planner_goal) {
    //    latLongToUtm(planner_goal.goal_lat, planner_goal.goal_lon, &goal_x, &goal_y,
    //		goal_utmzone);
    draw_planner_goal(goal_x - origin_x, goal_y - origin_y,
		      goal_theta);
    //  }

  /* draw the planner path */

  /*
  if(received_planner_trajectory) {
  dgc_planner_waypoint_p pw;
    glColor4f(1, 1, 0, 0.5);
    pthread_mutex_lock(&planner_mutex);
    glBegin(GL_QUAD_STRIP);
    w = DGC_PASSAT_WIDTH / 2.0 + 1;
    for(i = 0; i < planner_trajectory.num_waypoints; i++) {
      pw = &planner_trajectory.waypoint[i];
      glVertex2f(pw->x + w * cos(pw->theta + M_PI / 2.0) - smooth_x_copy,
		 pw->y + w * sin(pw->theta + M_PI / 2.0) - smooth_y_copy);
      glVertex2f(pw->x + w * cos(pw->theta - M_PI / 2.0) - smooth_x_copy,
		 pw->y + w * sin(pw->theta - M_PI / 2.0) - smooth_y_copy);
    }

    glEnd();
    pthread_mutex_unlock(&planner_mutex);
    }*/

  if(reference_traj != NULL) {
    glColor4f(1, 0, 0, 0.5);

    glLineWidth(2.0);

    glBegin(GL_LINE_STRIP);
    for(i = 0; i < reference_traj->num_waypoints; i++)
      glVertex2f(reference_traj->waypoint[i].x - origin_x,
                 reference_traj->waypoint[i].y - origin_y);
    glEnd();

    /*    glColor3f(0, 0, 0);
    glBegin(GL_LINES);
    for(i = 0; i < reference_traj->num_waypoints; i++) {
      glVertex2f(reference_traj->waypoint[i].x - origin_x,
                 reference_traj->waypoint[i].y - origin_y);
      glVertex2f(reference_traj->waypoint[i].x - origin_x +
		 0.1 * cos(reference_traj->waypoint[i].theta),
                 reference_traj->waypoint[i].y - origin_y +
		 0.1 * sin(reference_traj->waypoint[i].theta));
    }
    glEnd();*/

    glLineWidth(1.0);
  }
  
  if(received_planner_trajectory) {
    glLineWidth(2.0);
    glColor3f(1, 1, 0);
    pthread_mutex_lock(&planner_mutex);
    glBegin(GL_LINE_STRIP);
    for(i = 0; i < planner_trajectory.num_waypoints; i++)
      glVertex2f(planner_trajectory.waypoint[i].x - smooth_x_copy,
                 planner_trajectory.waypoint[i].y - smooth_y_copy);
    glEnd();

    glColor3f(0, 0, 0);
    glBegin(GL_LINES);
    for(i = 0; i < planner_trajectory.num_waypoints; i++) {
      glVertex2f(planner_trajectory.waypoint[i].x - smooth_x_copy,
                 planner_trajectory.waypoint[i].y - smooth_y_copy);
      glVertex2f(planner_trajectory.waypoint[i].x - smooth_x_copy +
		 0.1 * cos(planner_trajectory.waypoint[i].theta),
                 planner_trajectory.waypoint[i].y - smooth_y_copy +
		 0.1 * sin(planner_trajectory.waypoint[i].theta));
    }
    glEnd();

    pthread_mutex_unlock(&planner_mutex);
    glLineWidth(1.0);
  }

  if(received_radar1) {
    pthread_mutex_lock(&radar1_mutex);
    for(i = 0; i < radar1_num_targets; i++)
      draw_radar_target(radar1_track + i, smooth_x_copy, smooth_y_copy,
			1, 0, 0);
    pthread_mutex_unlock(&radar1_mutex);
  }
  
  if(received_radar2) {
    pthread_mutex_lock(&radar2_mutex);
    for(i = 0; i < radar2_num_targets; i++)
      draw_radar_target(radar2_track + i, smooth_x_copy, smooth_y_copy,
			0, 1, 0);
    pthread_mutex_unlock(&radar2_mutex);
  }
  
  if(received_radar3) {
    pthread_mutex_lock(&radar3_mutex);
    for(i = 0; i < radar3_num_targets; i++)
      draw_radar_target(radar3_track + i, smooth_x_copy, smooth_y_copy,
			0, 1, 0);
    pthread_mutex_unlock(&radar3_mutex);
  }

  if(received_radar4) {
    pthread_mutex_lock(&radar4_mutex);
    for(i = 0; i < radar4_num_targets; i++)
      draw_radar_target(radar4_track + i, smooth_x_copy, smooth_y_copy,
			0, 0, 1);
    pthread_mutex_unlock(&radar4_mutex);
  }

  if(received_radar5) {
    pthread_mutex_lock(&radar5_mutex);
    for(i = 0; i < radar5_num_targets; i++)
      draw_radar_target(radar5_track + i, smooth_x_copy, smooth_y_copy,
			0, 0, 1);
    pthread_mutex_unlock(&radar5_mutex);
  }

  /* go to 2D to draw overlays */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  pthread_mutex_lock(&message_buffer_mutex);
  max_width = 0;
  for(i = 0; i < num_messages; i++) {
    w = bitmapStringWidth(GLUT_BITMAP_HELVETICA_12, message_buffer[i].string);
    if(w > max_width)
      max_width = w;
  }
  glColor4f(0, 0, 0, 0.4);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(max_width + 10, 0);
  glVertex2f(max_width + 10, 5 + num_messages * 15);
  glVertex2f(0, 5 + num_messages * 15);
  glEnd();
  for(i = 0; i < num_messages; i++) {
    if(message_buffer[i].level == MESSAGE_LEVEL_STATUS)
      glColor3f(1, 1, 1);
    else if(message_buffer[i].level == MESSAGE_LEVEL_ERROR)
      glColor3f(1, 0, 0);
    else
      glColor3f(0, 0, 1);
    renderBitmapString(5, 5 + 15 * (num_messages - i - 1),
                       GLUT_BITMAP_HELVETICA_12, message_buffer[i].string);
  }
  pthread_mutex_unlock(&message_buffer_mutex);

  /* draw text in top left corner */
  pthread_mutex_lock(&gls_text_mutex);
  glColor4f(0, 0, 0, 0.4);
  glBegin(GL_POLYGON);
  glVertex2f(5, gui3D.window_height - 5);
  glVertex2f(235, gui3D.window_height - 5);
  glVertex2f(235, gui3D.window_height - 71 - 20 * gls_text.size());
  glVertex2f(5, gui3D.window_height - 71 - 20 * gls_text.size());
  glEnd();
  glColor3f(0.7, 0.7, 0.7);
  glBegin(GL_LINE_LOOP);
  glVertex2f(5, gui3D.window_height - 5);
  glVertex2f(235, gui3D.window_height - 5);
  glVertex2f(235, gui3D.window_height - 71 - 20 * gls_text.size());
  glVertex2f(5, gui3D.window_height - 71 - 20 * gls_text.size());
  glEnd();
  glColor3f(1, 1, 1);
  sprintf(str, "VEL : %.1f mph", dgc_ms2mph(applanix_pose.speed));
  renderBitmapString(10, gui3D.window_height - 25, GLUT_BITMAP_HELVETICA_18,
		     str);
  sprintf(str, "CTE: %.2f m", controller_target.cross_track_error);
  renderBitmapString(10, gui3D.window_height - 45, GLUT_BITMAP_HELVETICA_18,
		     str);
  sprintf(str, "LOC: %.2f %.2fm (%s)", 
	  gps_offset_x, gps_offset_y, add_offset ? "ON" : "OFF");
  renderBitmapString(10, gui3D.window_height - 65, GLUT_BITMAP_HELVETICA_18,
		     str);

  for(i = 0; i < (int)gls_text.size(); i++) {
    renderBitmapString(10, gui3D.window_height - 85 - 20 * i,
		       GLUT_BITMAP_HELVETICA_18, gls_text[i].text);
  }
  pthread_mutex_unlock(&gls_text_mutex);
 
  if(received_estop) {
    if(estop.estop_code == DGC_ESTOP_DISABLE)
      sprintf(str, "ESTOP : DISABLE");
    else if(estop.estop_code == DGC_ESTOP_PAUSE)
      sprintf(str, "ESTOP : PAUSE");
    else 
      sprintf(str, "ESTOP : RUN");
  }
  else
    sprintf(str, "ESTOP : NONE");
  renderBitmapString(300, gui3D.window_height - 25, GLUT_BITMAP_HELVETICA_18,
		     str);

  /* draw the steering wheel */
  glColor3f(1, 1, 1);
  draw_steering_wheel(gui3D.window_width - 50, 
                      gui3D.window_height - 50, 40.0, 
                      wheel_angle * DGC_PASSAT_STEERING_RATIO);
  if(received_passat_actuator)
    draw_pedals(gui3D.window_width - 130, 
		gui3D.window_height - 95, 10, 90, 
		passat_actuator.throttle_fraction,
		passat_actuator.brake_pressure);
  if(received_passat_signal) {
    current_time = dgc_get_time();
    if(passat_signal.signal == DGC_PASSAT_TURN_SIGNAL_LEFT) {
      if((int)floor((current_time - signal_ts) / 0.5) % 2 == 1)
	draw_left_turn_signal(gui3D.window_width - 90, 
			      gui3D.window_height - 100, 40);
    }
    else if(passat_signal.signal == DGC_PASSAT_TURN_SIGNAL_RIGHT) {
      if((int)floor((current_time - signal_ts) / 0.5) % 2 == 1)
	draw_right_turn_signal(gui3D.window_width - 30, 
			       gui3D.window_height - 100, 40);
    }
  }

  /* draw the rose - always do this last */
  dgc_imagery_draw_compass_rose_right(gui3D.window_width, gui3D.window_height, 
				      gui3D.camera_pose.rotation_2D,
				      gui3D.camera_pose.zoom);

#ifdef HAVE_VIDEOOUT
  /* save to video */
  if(record_video) { 
    dgc_videoout_add_opengl_frame(vo);
    free(vo->screenshot_buffer);
    free(vo->screenshot_buffer2);
    vo->screenshot_buffer = NULL;
    vo->screenshot_buffer2 = NULL;

  }
#endif    
}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_2D_mode();

  /* read the GLoverlay, if available */
  gloverlay = dgc_gloverlay_load(gloverlay_filename, &gloverlay_origin_x,
                                 &gloverlay_origin_y);

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_mainloop();
  return NULL;
}

void can_handler(dgc_can_status_message *can)
{
  wheel_angle = dgc_d2r(can->steering_angle) / DGC_PASSAT_STEERING_RATIO;
}

void rotate_2D(float *x, float *y, double theta)
{
  double x2, y2;

  x2 = cos(theta) * (*x) - sin(theta) * (*y);
  y2 = sin(theta) * (*x) - cos(theta) * (*y);
  *x = x2;
  *y = y2;
}

void applanix_pose_handler(void)
{
  static double last_x = 0, last_y = 0, last_yaw = 0;
  double x, y;

  latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &applanix_x,
              &applanix_y, utmzone);

  if(camera_unlocked && received_applanix_pose) {
    gui3D.camera_pose.x_offset_2D -= (applanix_x - last_x);
    gui3D.camera_pose.y_offset_2D -= (applanix_y - last_y);
  }
  last_x = applanix_x;
  last_y = applanix_y;

  if(game_mode) {
    gui3D.camera_pose.rotation_2D -= applanix_pose.yaw - last_yaw;
    gui3D.camera_pose.x_offset_2D = 
      gui3D.window_width * 3.0 / 8.0 /
      gui3D.camera_pose.zoom * cos(applanix_pose.yaw);
    gui3D.camera_pose.y_offset_2D = 
      gui3D.window_width * 3.0 / 8.0 / 
      gui3D.camera_pose.zoom * sin(applanix_pose.yaw);
  }
  last_yaw = applanix_pose.yaw;

  if(received_localize_pose) {
    x = applanix_pose.smooth_x + localize_pose.x_offset +
      DGC_PASSAT_IMU_TO_FA_DIST * cos(applanix_pose.yaw);
    y = applanix_pose.smooth_y + localize_pose.y_offset +
      DGC_PASSAT_IMU_TO_FA_DIST * sin(applanix_pose.yaw);
    dgc_path2D_add_point(&vehicle_path, x, y);
    dgc_path2D_add_point(&smooth_path, 
			 x - localize_pose.x_offset, 
			 y - localize_pose.y_offset);
  }

  /* 
       *** obsolete interface *** DIRK
       if(!received_applanix_pose)
       dgc_perception_request_map();
  */


  received_applanix_pose = 1;
}

void localize_pose_handler(void)
{
  gps_offset_x = localize_pose.x_offset -
    (applanix_x - applanix_pose.smooth_x);
  gps_offset_y = localize_pose.y_offset -
    (applanix_y - applanix_pose.smooth_y);
  received_localize_pose = 1;
}

void controller_target_handler(void)
{
  received_controller_target = 1;
  gui3D_forceRedraw();
}

void planner_trajectory_handler(void)
{
  received_planner_trajectory = 1;
  gui3D_forceRedraw();
}

/* 
   *** obsolete interface *** DIRK

void static_map_handler(dgc_perception_staticdiff_message *diff)
{
  dgc_client_map_cell_p default_cell = NULL;

  pthread_mutex_lock(&map_mutex);
  if(grid == NULL) {
    default_cell = (dgc_client_map_cell_p)
      calloc(1, sizeof(dgc_client_map_cell_t));
    dgc_test_alloc(default_cell);
    default_cell->value = 0;
    default_cell->unknown = 1;

    grid = dgc_grid_initialize(diff->resolution,
                               diff->total_map_height,
                               diff->total_map_width,
                               sizeof(dgc_client_map_cell_t), default_cell);
    dgc_grid_recenter_grid(grid, applanix_pose.smooth_x,
			   applanix_pose.smooth_y);
  }

  dgc_grid_recenter_grid(grid, applanix_pose.smooth_x,
			 applanix_pose.smooth_y);
  dgc_clientmap_apply_diff(grid, diff);
  received_map = 1;
  map_updated = 1;
  pthread_mutex_unlock(&map_mutex);
  gui3D_forceRedraw();
}
*/

void ibeo1_handler(dgc_ibeo_laser_message *ibeo)
{
  pthread_mutex_lock(&ibeo1_mutex);
  project_ibeo(ibeo, ibeo1_offset, applanix_pose.smooth_x, 
	       applanix_pose.smooth_y, applanix_pose.smooth_z,
	       applanix_pose.roll, applanix_pose.pitch, 
	       applanix_pose.yaw, &ibeo1_global);
  received_ibeo1 = 1;
  pthread_mutex_unlock(&ibeo1_mutex);
  gui3D_forceRedraw();
}

void ibeo2_handler(dgc_ibeo_laser_message *ibeo)
{
  pthread_mutex_lock(&ibeo2_mutex);
  project_ibeo(ibeo, ibeo2_offset, applanix_pose.smooth_x, 
	       applanix_pose.smooth_y, applanix_pose.smooth_z,
	       applanix_pose.roll, applanix_pose.pitch, 
	       applanix_pose.yaw, &ibeo2_global);
  received_ibeo2 = 1;
  pthread_mutex_unlock(&ibeo2_mutex);
  gui3D_forceRedraw();
}

void ldlrs1_handler(dgc_ldlrs_laser_message *ldlrs)
{
  pthread_mutex_lock(&ldlrs1_mutex);
  project_ldlrs(ldlrs, ldlrs1_offset, applanix_pose.smooth_x, 
		applanix_pose.smooth_y, applanix_pose.smooth_z,
		applanix_pose.roll, applanix_pose.pitch, 
		applanix_pose.yaw, &ldlrs1_global);
  received_ldlrs1 = 1;
  pthread_mutex_unlock(&ldlrs1_mutex);
  gui3D_forceRedraw();
}

void ldlrs2_handler(dgc_ldlrs_laser_message *ldlrs)
{
  pthread_mutex_lock(&ldlrs2_mutex);
  project_ldlrs(ldlrs, ldlrs2_offset, applanix_pose.smooth_x, 
		applanix_pose.smooth_y, applanix_pose.smooth_z,
		applanix_pose.roll, applanix_pose.pitch, 
		applanix_pose.yaw, &ldlrs2_global);
  received_ldlrs2 = 1;
  pthread_mutex_unlock(&ldlrs2_mutex);
  gui3D_forceRedraw();
}

void obstaclelist_handler(void)
{
  received_obstaclelist = 1;
}

void obstacles_handler(void)
{
  received_obstacles = 1;
}

void gls_handler(dgc_gls_overlay_message *gls)
{
  int i, found = 0;
  dgc_gls_overlay_message *temp;

  pthread_mutex_lock(&gls_mutex);
  for(i = 0; i < (signed)gls_cache.size(); i++)
    if(strcmp(gls->name, gls_cache[i]->name) == 0) {
      free(gls_cache[i]->byte);
      memcpy(gls_cache[i], gls, sizeof(dgc_gls_overlay_message));
      gls_cache[i]->byte = (unsigned char *)calloc(gls->max_bytes, 1);
      dgc_test_alloc(gls_cache[i]->byte);
      memcpy(gls_cache[i]->byte, gls->byte, gls->num_bytes);
      found = 1;
      break;
    }
  if(!found) {
    temp = dgc_gls_alloc("");
    memcpy(temp, gls, sizeof(dgc_gls_overlay_message));
    temp->byte = (unsigned char *)calloc(gls->max_bytes, 1);
    dgc_test_alloc(temp->byte);
    memcpy(temp->byte, gls->byte, gls->num_bytes);
    gls_cache.push_back(temp);
  }
  pthread_mutex_unlock(&gls_mutex);
  gui3D_forceRedraw();
  received_gls = 1;
}

void gls_text_handler(dgc_gls_text_message *text)
{
  int i, found, which;

  pthread_mutex_lock(&gls_text_mutex);

  found = 0;
  for(i = 0; i < (int)gls_text.size(); i++)
    if(strcmp(text->name, gls_text[i].name) == 0 && 
       gls_text[i].id == text->id) {
      which = i;
      found = 1;
      break;
    }
  if(!found) {
    gls_text.resize(gls_text.size() + 1);
    which = gls_text.size() - 1;
    gls_text[which].name = NULL;
    gls_text[which].text = NULL;
  }

  if(gls_text[which].name == NULL) {
    gls_text[which].name = (char *)calloc(strlen(text->name) + 1, 1);
    dgc_test_alloc(gls_text[which].name);
    strcpy(gls_text[which].name, text->name);
  }

  if(gls_text[which].text != NULL) 
    free(gls_text[which].text);
  if(text->text == NULL)
    gls_text[which].text = (char *)calloc(1, 1);
  else
    gls_text[which].text = (char *)calloc(strlen(text->text) + 1, 1);
  dgc_test_alloc(gls_text[which].text);
  if(text->text == NULL)
    gls_text[which].text[0] = '\0';
  else
    strcpy(gls_text[which].text, text->text);
    
  gls_text[which].id = text->id;

  pthread_mutex_unlock(&gls_text_mutex);
}

void passat_actuator_handler(void)
{
  received_passat_actuator = 1;
}

void passat_signal_handler(void)
{
  received_passat_signal = 1;
  signal_ts = dgc_get_time();
}

void error_status_handler(dgc_error_string_message *status)
{
  int i;

  pthread_mutex_lock(&message_buffer_mutex);
  if(num_messages == MESSAGE_BUFFER_SIZE) {
    free(message_buffer[0].string);
    for(i = 0; i < num_messages - 1; i++)
      message_buffer[i] = message_buffer[i + 1];
    num_messages--;
  }
  message_buffer[num_messages].string = 
    (char *)calloc(strlen(status->string) + 1, 1);
  dgc_test_alloc(message_buffer[num_messages].string);
  strcpy(message_buffer[num_messages].string, status->string);
  message_buffer[num_messages].timestamp = dgc_get_time();
  message_buffer[num_messages].level = MESSAGE_LEVEL_STATUS;
  num_messages++;
  pthread_mutex_unlock(&message_buffer_mutex);
}

void simulator_start_handler(void)
{
  int i;

  pthread_mutex_lock(&message_buffer_mutex);
  for(i = 0; i < num_messages; i++)
    free(message_buffer[i].string);
  num_messages = 0;
  pthread_mutex_unlock(&message_buffer_mutex);

  dgc_path2D_reset(&vehicle_path);
  dgc_path2D_reset(&smooth_path);
  gui3D_forceRedraw();
}

void simulator_handler(void)
{
  received_simdata = 1;
}

void stop_zones_handler(void)
{
  received_stop_zones = 1;
}

void estop_handler(void)
{
  received_estop = 1;
  gui3D_forceRedraw();
}

void planner_goal_handler(void)
{
  received_planner_goal = 1;
  gui3D_forceRedraw();
}

void copy_targets(radar_target_p src, radar_target_p dest, 
		  int *num_src, int num_dest)
{
  memcpy(src, dest, num_dest * sizeof(radar_target_t));
  *num_src = num_dest;
}

void project_radar(dgc_radar_sensor_message *radar, dgc_transform_t offset,
		   dgc_applanix_pose_message *applanix_pose, int radar_num,
		   radar_target_p target, int *num_targets)
{
  double z, roll, pitch, yaw, bearing;
  radar_target_t last_target[64];
  dgc_transform_t t;
  int mark = 0, i, j, last_num_targets = 0;

  copy_targets(last_target, target, &last_num_targets, *num_targets);

  dgc_transform_copy(t, offset);
  dgc_transform_rotate_x(t, applanix_pose->roll);
  dgc_transform_rotate_y(t, applanix_pose->pitch);
  dgc_transform_rotate_z(t, applanix_pose->yaw);

  dgc_transform_get_rotation(offset, &roll, &pitch, &yaw);
  
  if(radar->num_targets > 64)
    dgc_die("Error: too many radar targets.   %d\n", radar->num_targets);

  for(i = 0; i < radar->num_targets; i++) {
    /* skip objects that are moving too fast or too slow */
    if(fabs(radar->target[i].relative_velocity) < dgc_mph2ms(2.0) ||
       fabs(radar->target[i].relative_velocity) > dgc_mph2ms(40.0))
      continue;

    target[mark].x = radar->target[i].distance;
    target[mark].y = radar->target[i].lateral_offset;

    z = 0;
    dgc_transform_point(&target[mark].x, &target[mark].y, &z, t);
    target[mark].x += applanix_pose->smooth_x;
    target[mark].y += applanix_pose->smooth_y;

    target[mark].lateral_offset = radar->target[i].lateral_offset;
    target[mark].distance = radar->target[i].distance;

    target[mark].radar_num = radar_num;
    target[mark].id = radar->target[i].id;
    target[mark].measured = radar->target[i].measured;
    target[mark].historical = radar->target[i].historical;
    target[mark].lateral_offset_var = radar->target[i].lateral_offset_var;
    target[mark].relative_acceleration = radar->target[i].relative_acceleration;
    target[mark].relative_velocity = radar->target[i].relative_velocity;
    
    bearing = atan2(radar->target[i].lateral_offset, radar->target[i].distance);

    target[mark].v_x = radar->target[i].relative_velocity *
      cos(yaw + applanix_pose->yaw + bearing);
    target[mark].v_y = radar->target[i].relative_velocity *
      sin(yaw + applanix_pose->yaw + bearing);

    target[mark].track_count = 1;
    for(j = 0; j < last_num_targets; j++)
      if(last_target[j].id == target[mark].id) {
	target[mark].track_count = last_target[j].track_count + 1;
	break;
      }

    mark++;
  }
  *num_targets = mark;
}

void radar1_handler(dgc_radar_sensor_message *radar1)
{
  if(!received_applanix_pose)
    return;

  pthread_mutex_lock(&radar1_mutex);
  project_radar(radar1, radar1_offset, &applanix_pose, 1, radar1_track,
		&radar1_num_targets);
  received_radar1 = 1;
  pthread_mutex_unlock(&radar1_mutex);
}

void radar2_handler(dgc_radar_sensor_message *radar2)
{
  if(!received_applanix_pose)
    return;
  
  pthread_mutex_lock(&radar2_mutex);
  project_radar(radar2, radar2_offset, &applanix_pose, 2, radar2_track,
		&radar2_num_targets);
  received_radar2 = 1;
  pthread_mutex_unlock(&radar2_mutex);
}

void radar3_handler(dgc_radar_sensor_message *radar3)
{
  if(!received_applanix_pose)
    return;
  
  pthread_mutex_lock(&radar3_mutex);
  project_radar(radar3, radar3_offset, &applanix_pose, 3, radar3_track,
		&radar3_num_targets);
  received_radar3 = 1;
  pthread_mutex_unlock(&radar3_mutex);
}

void radar4_handler(dgc_radar_sensor_message *radar4)
{
  if(!received_applanix_pose)
    return;
  
  pthread_mutex_lock(&radar4_mutex);
  project_radar(radar4, radar4_offset, &applanix_pose, 4, radar4_track,
		&radar4_num_targets);
  received_radar4 = 1;
  pthread_mutex_unlock(&radar4_mutex);
}

void radar5_handler(dgc_radar_sensor_message *radar5)
{
  if(!received_applanix_pose)
    return;
  
  pthread_mutex_lock(&radar5_mutex);
  project_radar(radar5, radar5_offset, &applanix_pose, 5, radar5_track,
		&radar5_num_targets);
  received_radar5 = 1;
  pthread_mutex_unlock(&radar5_mutex);
}

void read_parameters(int argc, char **argv)
{
  dgc_param_t params[] = {
    {"imagery", "root", DGC_PARAM_FILE, &imagery_root, 0, NULL},
    {"transform", "ibeo_laser1", DGC_PARAM_TRANSFORM, &ibeo1_offset, 0, NULL},
    {"transform", "ibeo_laser2", DGC_PARAM_TRANSFORM, &ibeo2_offset, 0, NULL},
    {"transform", "ldlrs_laser1", DGC_PARAM_TRANSFORM, &ldlrs1_offset, 0, NULL},
    {"transform", "ldlrs_laser2", DGC_PARAM_TRANSFORM, &ldlrs2_offset, 0, NULL},
    {"transform", "radar1", DGC_PARAM_TRANSFORM, &radar1_offset, 0, NULL},
    {"transform", "radar2", DGC_PARAM_TRANSFORM, &radar2_offset, 0, NULL},
    {"transform", "radar3", DGC_PARAM_TRANSFORM, &radar3_offset, 0, NULL},
    {"transform", "radar4", DGC_PARAM_TRANSFORM, &radar4_offset, 0, NULL},
    {"transform", "radar5", DGC_PARAM_TRANSFORM, &radar5_offset, 0, NULL},
    {"imagery", "gloverlay", DGC_PARAM_FILE, &gloverlay_filename, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILE, &rndf_filename, 0, NULL},
  };
  dgc_param_install_params(argc, argv, params, sizeof(params) / 
                           sizeof(params[0]));
}

int main(int argc, char **argv)
{
  char temp_filename[200];
  pthread_t thread;
  param_struct_t param;

  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  strcpy(utmzone, "10S");

  dgc_path2D_new(&vehicle_path, 1000);
  dgc_path2D_new(&smooth_path, 1000);

  /* read the reference trajectory, if available */
  //  reference_traj = dgc_trajectory_read(argv[1]);

  read_parameters(argc, argv);

  /* load the RNDF file, if available */
  /*  if(strcmp(rndf_filename + strlen(rndf_filename) - 10, ".srndf.txt") == 0) {
    strcpy(temp_filename, rndf_filename);
    strcpy(temp_filename + strlen(temp_filename) - 10, ".txt");
  }
  else*/
    strcpy(temp_filename, rndf_filename);

  rndf = new rndf_file;
  if(rndf->load(temp_filename) == 0) {
    rndf_valid = 1;
    rndf->mark_yield_exits();
    rndf->mark_merge_exits();
  }

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  dgc_applanix_subscribe_pose_message(&applanix_pose,
                                      (dgc_handler_t)applanix_pose_handler,
                                      DGC_SUBSCRIBE_ALL, NULL);
  dgc_localize_subscribe_pose_message(&localize_pose,
                                      (dgc_handler_t)localize_pose_handler,
                                      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_can_subscribe_status_message(NULL, (dgc_handler_t)can_handler, 
				   DGC_SUBSCRIBE_LATEST, NULL);
  /* 
       *** obsolete interface *** DIRK
        dgc_perception_subscribe_staticdiff_message(NULL, (dgc_handler_t)
                                                    static_map_handler,
						    DGC_SUBSCRIBE_ALL, NULL);
  */
  dgc_perception_subscribe_obstaclelist_message(&obstaclelist, (dgc_handler_t)
						obstaclelist_handler,
						DGC_SUBSCRIBE_ALL, 
						&obstaclelist_mutex);
  dgc_perception_subscribe_stop_zones_message(&stop_zones, (dgc_handler_t)
					      stop_zones_handler,
					      DGC_SUBSCRIBE_ALL, 
					      &stop_zones_mutex);
  dgc_perception_subscribe_obstacles_message(&obstacles, (dgc_handler_t)
					     obstacles_handler,
					     DGC_SUBSCRIBE_LATEST,
					     &obstacles_mutex);

  dgc_radar_subscribe_sensor1_message(NULL, (dgc_handler_t)radar1_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_radar_subscribe_sensor2_message(NULL, (dgc_handler_t)radar2_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_radar_subscribe_sensor3_message(NULL, (dgc_handler_t)radar3_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_radar_subscribe_sensor4_message(NULL, (dgc_handler_t)radar4_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_radar_subscribe_sensor5_message(NULL, (dgc_handler_t)radar5_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);

  dgc_ibeo_subscribe_laser1_message(NULL, (dgc_handler_t)ibeo1_handler,
  				    DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ibeo_subscribe_laser2_message(NULL, (dgc_handler_t)ibeo2_handler,
				    DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ldlrs_subscribe_laser1_message(NULL, (dgc_handler_t)ldlrs1_handler,
				     DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ldlrs_subscribe_laser2_message(NULL, (dgc_handler_t)ldlrs2_handler,
				    DGC_SUBSCRIBE_LATEST, NULL);
  dgc_gls_subscribe_overlay_message(NULL, (dgc_handler_t)
                                    gls_handler, DGC_SUBSCRIBE_ALL, 
                                    &gls_mutex);
  dgc_gls_subscribe_text_message(NULL, (dgc_handler_t)
				 gls_text_handler, DGC_SUBSCRIBE_ALL,
				 NULL);
  dgc_error_subscribe_status_message(NULL, (dgc_handler_t)
                                     error_status_handler,
                                     DGC_SUBSCRIBE_ALL,
                                     NULL);
  dgc_simulator_subscribe_groundtruth_message(&simdata, 
                                              (dgc_handler_t)simulator_handler,
                                              DGC_SUBSCRIBE_LATEST,
                                              &simulator_mutex);
  dgc_simulator_subscribe_start_message(NULL,
					(dgc_handler_t)simulator_start_handler,
					DGC_SUBSCRIBE_ALL, NULL);
  dgc_controller_subscribe_target_message(&controller_target,
                                          (dgc_handler_t)
                                          controller_target_handler,
                                          DGC_SUBSCRIBE_LATEST, NULL);
  dgc_passat_subscribe_actuator_message(&passat_actuator,
					(dgc_handler_t)
					passat_actuator_handler,
					DGC_SUBSCRIBE_LATEST, NULL);
  dgc_estop_subscribe_status_message(&estop, (dgc_handler_t)estop_handler,
				     DGC_SUBSCRIBE_LATEST, NULL);
  dgc_passat_subscribe_turnsignal_message(&passat_signal, (dgc_handler_t)
					  passat_signal_handler,
					  DGC_SUBSCRIBE_LATEST, NULL);
  dgc_planner_subscribe_trajectory_message(&planner_trajectory, (dgc_handler_t)
                                           planner_trajectory_handler,
                                           DGC_SUBSCRIBE_LATEST, 
                                           &planner_mutex);
  dgc_planner_subscribe_goal_message(&planner_goal, (dgc_handler_t)
				     planner_goal_handler,
				     DGC_SUBSCRIBE_LATEST, NULL);
				    
  dgc_ipc_dispatch();
}
