#ifndef PERCEPTION_VIEW_VIEW_H_
#define PERCEPTION_VIEW_VIEW_H_

#include <roadrunner.h>
#ifdef HAVE_VIDEOOUT
#include <videoout.h>
#endif
#include <applanix_interface.h>
#include <param_interface.h>
#include <passatmodel.h>
#include <lltransform.h>
#include <gui3D.h>
#include <velocore.h>
#include <imagery.h>
#include <lltransform.h>
#include <aw_roadNetwork.h>
#include <grid.h>
#include <rndfgl.h>
#include <passat_constants.h>
#include <passat_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
#include <ldlrs_interface.h>
#include <laser_interface.h>
#include <can_interface.h>
#include <radar_interface.h>
#include <estop_interface.h>
#include <error_interface.h>
#include <planner_interface.h>
#include <controller_interface.h>
#include <simulator_interface.h>
#include <ghostcar_interface.h>
#include <trajectory_points_interface.h>
#include <gls_interface.h>
#include <passat_constants.h>
#include <facelist.h>
#include <pointlist.h>
#include <project.h>
#include <skybox.h>
#include <stdint.h>

#ifdef SIMULATOR_FX
#include <vehiclemodels.h>
#endif

#ifdef USE_QT
#include "qt.h"
extern int __gui3d_use_qt;
extern QApplication  *win  ;
extern QTGui         *qgui;
void   qgui3D_initialize(int argc, char **argv, int window_x, int window_y,
			 int window_width, int window_height, double fps);
void   qgui3D_mainloop( void );
#endif


#define MAX_NUM_SCANS            10000
#define MAX_NUM_POINTS_PER_BEAM  20000
#define NUM_LASER_BEAMS             64
#define NAME_LENGTH                 80
#define MAX_SENSOR_INFO            100
#define APPLANIX_HISTORY_LENGTH    100
#define GLS_REMOVE_TIME              5.0
#define SIM_STATUS_WARNING_TIME     10.0
#define RADAR_MARKER_SIZE            1.0
#define NUM_SIM_CAR_COLORS           6
#define MAX_NUM_SIM_CARS           300
#define MAX_FILL_SIZE              2.0
#define MAX_INTENSITY              128.0
#define MIN_DIST_BETWEEN_SCANS     0.1
#define NUM_MESH_POLYGONS          40000
#define NUM_MESH_POINTS            50000


extern float camera_rot_angle;

#define IPC_GRID_MAP            0

#define PITCH_MODE              1
#define YAW_MODE                2

#ifndef MAX
#define MAX(x,y) (x > y ? x : y)
#endif

#ifndef MIN
#define MIN(x,y) (x < y ? x : y)
#endif

#ifdef HAVE_VIDEOOUT
extern dgc_videoout_p vo;
#endif

#define NUM_LRR2_RADARS         4
#define NUM_LRR3_RADARS		2
#define NUM_RADARS		(NUM_LRR2_RADARS + NUM_LRR3_RADARS)
#define RADAR3_INDEX		2		// RADAR3 is an LRR3. This value is used in the radar for-loops.
#define RADAR6_INDEX		5               // RADAR6 is an LRR3. This value is used in the radar for-loops.


typedef struct {
  float   x, y, z;
  char    i, v;
} velodyne_pt_t, * velodyne_pt_p;

typedef struct {
  double  forward_accel_warning_time;
  double  lateral_accel_warning_time;
  double  plan_warning_time;
  double  collision_warning_time;
  char    name[20];
  int     model_nr;
} sim_status_t;

typedef struct {

  double    r;
  double    g;
  double    b;

} dgc_rgb_t, *dgc_rgb_p;

typedef struct {

  double    lat;
  double    lon;
  double    theta;
  double    utm_x;
  double    utm_y;
  char      utm_zone[10];
  int       goal_set;

} dgc_planner_goal_t, *dgc_planner_goal_p;

#define NUM_COLORS  8
extern dgc_rgb_t                    crgb[NUM_COLORS];

using namespace dgc;

typedef struct {

  double    h;
  double    s;
  double    v;

} dgc_hsv_t, *dgc_hsv_p;


typedef struct applanix_elem_t {

  applanix_elem_t                * next;
  applanix_elem_t                * prev;
  ApplanixPose                     data;

} *applanix_elem_p;

typedef struct {

  applanix_elem_p                  current;

} applanix_history_t, *applanix_history_p;

#define MAX_MESSAGE_LEN  128

typedef struct {
  double         timestamp;
  char           string[MAX_MESSAGE_LEN];
} status_message_t;

typedef struct {
  int            num_points;
  vlr::dgc_point3D_p  point;
  float         *range;
  float         *intensity;
} projected_scan_t, *projected_scan_p;

typedef struct {

  int                    size;
  projected_scan_p       scan;
  projected_scan_p       last_scan;
  dgc_facelist_t         fl;
  dgc_pointlist_t        pl;
  int                    firsttime;
  float                  last_x;
  float                  last_y;

} dgc_laser_mesh_data_t, *dgc_laser_mesh_data_p;

typedef struct {

  double                 lon;
  double                 lat;
  double                 theta;

} car_place_t;

#define GOAL_NOT_SET    0
#define GOAL_DO_SET     1
#define GOAL_XY_SET     2
#define GOAL_ALL_SET    3

extern car_place_t                             goal_place;
extern int                                     goal_place_active;
extern double                                  scene_x;
extern double                                  scene_y;

extern dgc_laser_mesh_data_t                   lms1_mesh;
extern dgc_laser_mesh_data_t                   lms3_mesh;


extern IpcInterface                            *ipc;
extern ParamInterface                          *pint;

extern ApplanixPose                            app_car_pose;
extern dgc_planner_goal_t                 planner_goal;


#define MESSAGE_BUFFER_SIZE       8
extern status_message_t message_buffer[MESSAGE_BUFFER_SIZE];
extern status_message_t error_buffer[MESSAGE_BUFFER_SIZE];

extern vlr::PerceptionStopZones                     stop_zones;
extern dgc_velodyne_config_p                   veloconfig;
extern std::vector <vlr::GlsOverlay *>      gls_cache;

#define ACTION_NOOP                    0
#define ACTION_READ_TRANSFORMATIONS    1
#define ACTION_RESET_MAP               2
#define ACTION_REQUEST_MAP             3
#define ACTION_SUBSCRIBE               5
#define ACTION_ESTOP_RUN               6
#define ACTION_ESTOP_PAUSE             7
#define ACTION_SEND_GOAL               8
#define ACTION_GHOST_CAR_SYNC          9

#define MAX_GLS_SOURCES                20
#define MAX_GLS_NAME_LENGTH            50

extern int                          action;

extern int                          use_ipc_grid;
extern int                          use_velo_shm;

extern int                          show_beams;
extern int                          show_simulator;
extern int                          camera_lock;
extern int                          camera_limit;
extern int                          show_inverse;
extern int                          show_info;
extern int                          show_estop;
extern int                          show_fancy_estop;
extern int                          show_cte;
extern int                          show_gmeter;
extern int                          show_all_sensors;
extern int                          show_sensor_info;
extern int                          show_sensor_info_layout;
extern int                          show_coordinates;
extern int                          show_gls;
extern int                          show_help;
extern int                          show_status;
extern int                          show_gls_select;
extern int                          show_trajectory;
extern int                          show_goal;
extern int                          show_goal_place;
extern int                          show_planner_goal;
extern int                          show_grid;
extern int                          show_ipc_grid_map;
extern int                          show_rndf;
extern int                          show_colors;
extern int                          change_colors;
extern int                          show_car;
extern int                          show_ghost_car;
extern int                          show_no_car;
extern int                          show_stickers;
extern int                          show_car_nr;
extern int                          show_car_marker;
extern int                          show_localize;
extern int                          show_inside_view;
extern int                          show_actuators;
extern int                          show_canact;
extern int                          show_radar;
extern int                          show_all_radars;
extern int                          show_radar_cal;
extern int                          plain_mode;
extern int                          show_point_size;
extern int                          show_distances;
extern int                          show_obstacles;
extern int                          show_frame_classifications;
extern int                          show_track_classifications;
extern int                          show_dynamic;
extern int                          show_velodyne;
extern int                          show_lines;
extern int                          show_intensity;
extern int                          show_lower_block;
extern int                          show_upper_block;
extern int                          show_imagery;
extern int                          show_flat_imagery;
extern int                          show_ldlrs1;
extern int                          show_ldlrs2;
extern int                          show_lms1;
extern int                          show_lms3;
extern float                        clock_offset_sec;
extern int                          show_clock;
extern float                        tri_angle_yaw;
extern float                        tri_angle_pitch;
extern int                          sep_colors;
extern int                          color_num;
extern int                          record_video;
extern int                          mark_single_beam;
extern int                          marked_beam;
extern int                          follow_mode;
extern short                        beam_active[NUM_LASER_BEAMS];
extern double                       velodyne_range_multiplier;

extern unsigned char                ldlrs1_mask[];
extern int                          ldlrs1_mask_size;
extern unsigned char                ldlrs2_mask[];
extern int                          ldlrs2_mask_size;

extern vlr::rndf::RoadNetwork* rn;
extern int                          rndf_valid;
extern char*                        rndf_filename;
extern vlr::rndf_display_list*      rndf_dl;
extern int                          rndf_generate_display_lists;

extern double                       map_resolution;
extern int                          map_size_x;
extern int                          map_size_y;

extern double                       global_x;
extern double                       global_y;

extern double                       ghost_car_x;
extern double                       ghost_car_y;

extern double                       stop_zones_dist_before_line;
extern double                       stop_zones_dist_behind_line;
extern double                       stop_zones_width;
extern double                       stop_zones_side_shift;

extern EstopStatus                  estop;
extern LdlrsLaser                   ldlrs1, ldlrs2;
extern LaserLaser                   lms1, lms3;
extern CanStatus                    can;
extern RadarSensor                  radar_lrr2[NUM_LRR2_RADARS];
extern RadarLRR3Sensor              radar_lrr3[NUM_LRR3_RADARS];
//extern RadarSensor                  radar[NUM_RADARS];
extern dgc_grid_p                   ipc_grid;
extern dgc_transform_t              ldlrs1_offset, ldlrs2_offset;
extern dgc_transform_t              lms1_offset, lms3_offset;
extern dgc_transform_t              radar_lrr2_offset[NUM_LRR2_RADARS];
extern dgc_transform_t              radar_lrr3_offset[NUM_LRR3_RADARS];
extern dgc_velodyne_scan_p          scans;

extern uint32_t                     velo_shm_key;
extern int                          num_scans;
extern char                       * imagery_root;
extern double                       perception_hz;
extern double                       applanix_hz;
extern double                       ldlrs1_hz;
extern double                       ldlrs2_hz;
extern double                       lms1_hz;
extern double                       lms3_hz;
extern double                       can_hz;
extern double                       gls_hz;
extern double                       gls_s_hz[MAX_GLS_SOURCES];
extern int                          num_simcars;
extern int                          simcars_required;

extern int                          occupied_map_cells;
extern char                         utmzone[5];

extern int                          mouse_pos_x;
extern int                          mouse_pos_y;

extern double                                gps_offset_x;
extern double                                gps_offset_y;
extern double                                applanix_x;
extern double                                applanix_y;
extern ApplanixRms                           applanix_rms;
extern ApplanixDmi                           applanix_dmi;
extern ApplanixGps                           applanix_gps;
extern vlr::PerceptionMapDiff                     diff;
extern vlr::PerceptionObstacles                   obstacles;
extern PlannerTrajectory                     planner_trajectory;
extern vlr::TrajectoryPoints2D                    trajectory_points;


extern PlannerMdfGoal                        planner_mdf_goal;
extern LocalizePose                          loc_pose;
extern PassatActuator                        passat_act;
extern PassatTurnSignal                      passat_signal;
extern ControllerTarget                      controller_target;
extern SimulatorGroundTruth                  simulator_groundtruth;
//extern SimulatorTag                          simulator_tags;
extern GhostcarPose                          ghost_car;

extern pthread_mutex_t                    ldlrs1_mutex;
extern pthread_mutex_t                    ldlrs2_mutex;
extern pthread_mutex_t                    lms1_mutex;
extern pthread_mutex_t                    lms3_mutex;
extern pthread_mutex_t                    radar_mutex;
extern pthread_mutex_t                    stop_zone_mutex;
extern pthread_mutex_t                    trafficlight_mutex;
extern pthread_mutex_t                    obstacles_mutex;
extern pthread_mutex_t                    dynamic_mutex;
extern pthread_mutex_t                    traj_mutex;
extern pthread_mutex_t                    traj2D_mutex;
extern pthread_mutex_t                    goal_mutex;
extern pthread_mutex_t                    gls_mutex;
extern pthread_mutex_t                    simulator_mutex;
extern pthread_mutex_t                    ghost_car_mutex;
extern pthread_mutex_t                    message_buffer_mutex;
extern pthread_mutex_t                    rndf_mutex;
extern pthread_mutex_t                    camera_mutex;
extern pthread_mutex_t                    map_mutex;
extern pthread_mutex_t                    pose_mutex;

extern char   *sensor_info[MAX_SENSOR_INFO];
extern char   *gls_select[MAX_GLS_SOURCES+6];
extern char    gls_enable[MAX_GLS_SOURCES];
extern double  gls_size[MAX_GLS_SOURCES];

extern vlr::dgc_passatwagonmodel_t*            ghost;
extern vlr::dgc_passatwagonmodel_t*            passat;
extern vlr::dgc_passatwagonmodel_t*            sim_passat[NUM_SIM_CAR_COLORS];
extern sim_status_t                            sim_car_status[MAX_NUM_SIM_CARS];
#ifdef SIMULATOR_FX
extern dgc_vehicle_gl_model               vehicle_gl_models;
#endif

extern vlr::rndf_display_list                * rndf_dl;

void       generate_rndf( void );

dgc_rgb_t  dgc_val_to_rgb( double val );
float      pts_dist( velodyne_pt_t pt1, velodyne_pt_t pt2 );
double     pose_dist( dgc_pose_p pt1, dgc_pose_p pt2 );
char *     createTimeString( double time );
char *     createDateString( double time );
void       myglVertex3f( float x, float y, float z );
int        cycle_colors( int offset );

void       keyboard( unsigned char key,
		     __attribute__ ((unused)) int x,
		     __attribute__ ((unused)) int y );

void       mouse( int button, int state, int x, int y );
void       motion( int x, int y );

dgc_rgb_t  dgc_hsv_to_rgb( dgc_hsv_t color );

void       viewer_ipc_init( int argc, char *argv[] );

void       ipc_check_subscription( void );
void       ipc_subscribe_radar( int on );
void       ipc_subscribe_can( int on );
void       ipc_subscribe_estop( int on );
void       ipc_subscribe_localize( int on );
void       ipc_subscribe_ldlrs1( int on );
void       ipc_subscribe_ldlrs2( int on );
void       ipc_subscribe_lms1( int on );
void       ipc_subscribe_lms3( int on );
void       ipc_subscribe_obstacles( int on );
void       ipc_subscribe_map_diff( int on );
void       ipc_subscribe_map( int on );
void       ipc_subscribe_map_reset( int on );
void       ipc_subscribe_stop_zones( int on );

void       ipc_subscribe_trajectory( int on );
void       ipc_subscribe_goal( int on );
void       ipc_subscribe_planner_goal( int on );
void       ipc_subscribe_applanix( int on );
void       ipc_subscribe_applanix_rms( int on );
void       ipc_subscribe_passat_actuators( int on );
void       ipc_subscribe_gls( int on );

void       draw_msg( const char *msg, ... );
void       draw_info_window( ApplanixPose *pose );
void       draw_applanix_window( ApplanixPose *pose );
void       draw_grid( ApplanixPose  *pose,
		      double center_x, double center_y );
void       draw_distances(ApplanixPose  *pose);
void       draw_stop_zones(vlr::rndf::RoadNetwork& rn, double origin_x, double origin_y);
void       draw_cube( float x, float y, float z1, float z2, float w,
		      dgc_rgb_t rgb1, dgc_rgb_t rgb2, float occ );
void       draw_vehicle_cage( double x, double y, double theta,
			      double w, double l, int id, double v,
			      double range, int draw_flag );
void       draw_cte( float error );
void       draw_marker( float x, float y, float z, float size );
void       draw_disc( float x, float y, float z, float size );
void       draw_pedestrian( float x, float y, float z1, float height, float size, 
                            float direction, float magnitude, dgc_rgb_t rgb, float occ);

void       draw_bicyclist( float x, float y, float z, float scale, float direction, 
			   float magnitude, dgc_rgb_t rgb, float occ);
void       draw_obstacle_frame( float x, float y, float z, float length, float width, 
                            float direction, float magnitude, dgc_rgb_t rgb);


void       draw_cage( double l, double w, double h );
void       draw_car_box( double l, double w );
void       draw_empty_box( double l, double w );
void       draw_arrow( float x, float y, float z, float size );
void       draw_narrow_arrow( float x, float y, float z, float size );
void       draw_cross_hair( float x, float y, float z, float size );
void       draw_star( float x, float y, float z, float size );
void       draw_radar_spot( float x, float y, float z, float size, float height,
			    dgc_rgb_t rgb1, dgc_rgb_t rgb2, float occ );
void       draw_radar_cal( float x, float y, float z, float size,
			   ApplanixPose *pose,
			   int radar_num, RadarTarget target );
void       draw_radar_lrr3_cal( float x, float y, float z, float size,
			   ApplanixPose *pose,
			   int radar_num, RadarLRR3Target target );
void       draw_bitmap_text(float x, float y, void *font, char *string);
void       draw_velodyne( int num_scans, dgc_velodyne_scan_p scans,
			  ApplanixPose *cur_pose );
void       draw_grid_map( dgc_grid_p map, ApplanixPose *pose,
			  char *data, short offset );
void       draw_ldlrs( LdlrsLaser *ldlrs,
		       ApplanixPose *cur_pose,
		       dgc_transform_t offset, pthread_mutex_t *mutex );
void       draw_lms( LaserLaser *laser,
		     ApplanixPose *cur_pose,
		     dgc_transform_t offset,
		     dgc_laser_mesh_data_p mesh, pthread_mutex_t *mutex );
void       draw_lms( LaserLaser  *lms,
		     ApplanixPose *cur_pose,
		     dgc_transform_t offset, pthread_mutex_t *mutex );
void       draw_pedals(float x, float y, float w, float h, double throttle, double brake);
void       draw_steering_wheel(float x, float y, float r, float steering_angle);
void       draw_actuators( float steering_angle, float throttle, float brake,
			   char signal, float speed, int p_cte, float cte,
			   int offset_x, int offset_y, float t );
void       draw_help( void );
void       draw_g_meter( float x, float y, float size, float x_acc, float y_acc );
void       draw_stickered_passat(vlr::dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle, double velodyne_angle, int draw_body );
void       draw_left_turn_signal(float x, float y, float r);
void       draw_left_turn_signal_outline(float x, float y, float r);
void       draw_right_turn_signal(float x, float y, float r);
void       draw_right_turn_signal_outline(float x, float y, float r);
void       draw_accident_sign_3D(double x, double y, double z,
				 double r, double heading, double blend);
void       draw_death_sign_3D(double x, double y, double z,
			      double r, double heading, double blend);
void       draw_slippery_sign_3D(double x, double y, double z,
				 double r, double heading, double blend);
void       draw_warning_sign_3D(double x, double y, double z,
				double r, double heading, double blend);

void       init_mesh_data( dgc_laser_mesh_data_p mesh );
void       check_mesh_data( dgc_laser_mesh_data_p mesh, int size );
void       laser_scan_copy(projected_scan_p dest, projected_scan_p src);
void       laser_add_points(projected_scan_p scan, dgc_pointlist_p pointlist);
void       laser_add_polygons( projected_scan_p old_scan, projected_scan_p new_scan, dgc_facelist_p facelist );

char *     numf( float v, char *unit );
char *     _numf( float v, char *unit );
void      display( void );
void       display_redraw( void );
void       display_check_camera( ApplanixPose *pose );

dgc_pose_t dgc_applanix_pose_difference( ApplanixPose a,
					 ApplanixPose b );

ApplanixPose *  applanix_history_pose( applanix_elem_p elem );
ApplanixPose *  applanix_current_pose( applanix_history_p history );
applanix_elem_p              applanix_history_elem( applanix_history_p history );
applanix_elem_p              applanix_history_next( applanix_elem_p elem );
applanix_elem_p              applanix_history_prev( applanix_elem_p elem );

ApplanixPose *  applanix_current_pose( void );
ApplanixPose *  applanix_pose( double timestamp );
void                         applanix_history_init( int size );
void                         applanix_history_add( ApplanixPose *msg );

#endif
