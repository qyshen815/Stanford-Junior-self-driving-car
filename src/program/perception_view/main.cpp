#include <IL/il.h>

#ifdef HAVE_VIDEOOUT
#include <videoout.h>
#endif

#include <global.h>
#include <stdint.h>
#include <velodyne_shm_interface.h>

#include "view.h"


using namespace vlr::rndf;
using namespace vlr;

IpcInterface                 *ipc                = NULL;
ParamInterface               *pint               = NULL;

int                          show_beams          = 0;
int                          camera_lock         = 1;
int                          camera_limit        = 1;
int                          action              = ACTION_NOOP;
int                          show_inverse        = 0;
int                          show_help           = 0;
int                          show_status         = 1;
int                          show_flat_imagery   = 1;
int                          show_gls_select     = 0;
int                          show_grid           = 0;
int                          show_gls            = 0;
int                          show_shm_grid_map   = 0;
int                          show_ipc_grid_map   = 0;
int                          show_coordinates    = 0;
int                          show_rndf           = 1;
int                          show_car            = 1;
int                          show_ghost_car      = 1;
int                          show_no_car         = 0;
int                          show_stickers       = 0;
int                          show_car_nr         = 0;
int                          show_car_marker     = 0;
int                          show_localize       = 0;
int                          show_sky            = 0;
int                          show_inside_view    = 0;
int                          show_actuators      = 0;
int                          show_canact         = 0;
int                          show_info           = 0;
int                          show_cte            = 0;
int                          show_gmeter         = 0;
int                          show_clock          = 0;
int                          show_all_sensors    = 0;
int                          show_sensor_info    = 0;
int                          show_sensor_info_layout = 2;
int                          show_radar          = 0;
int                          show_radar_cal      = 0;
int                          show_all_radars     = 1;
int                          show_simulator      = 0;
int                          show_colors         = 0;
int                          change_colors       = 0;
int                          plain_mode          = 0;
int                          show_distances      = 0;
int                          show_point_size     = 0;
int                          show_obstacles      = 1;
int                          show_track_classifications = 1;
int                          show_frame_classifications = 0;
int                          show_dynamic        = 1;
int                          show_velodyne       = 0;
int                          show_lines          = 0;
int                          show_intensity      = 0;
int                          show_lower_block    = 1;
int                          show_upper_block    = 1;
int                          show_imagery        = 1;
int                          show_trajectory     = 1;
int                          show_goal           = 0;
int                          show_planner_goal   = 1;
int                          show_ldlrs1         = 0;
int                          show_ldlrs2         = 0;
int                          show_lms1           = 0;
int                          show_lms3           = 0;
float                        tri_angle_yaw       = 0.0;
float                        tri_angle_pitch     = 0.0;
int                          sep_colors          = 0;
int                          color_num           = 0;
int                          record_video        = 0;
int                          mark_single_beam    = 0;
int                          marked_beam         = 0;
int                          follow_mode         = 0;
int                          num_simcars         = 0;
int                          simcars_required    = 0;
double                       stop_zones_dist_before_line = 0;
double                       stop_zones_dist_behind_line = 0;
double                       stop_zones_width            = 0;
double                       stop_zones_side_shift       = 0;

int                          show_estop           = 1;
int                          show_fancy_estop     = 0;
float                        clock_offset_sec     = 391.0;

short                        beam_active[NUM_LASER_BEAMS] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

dgc_rgb_t sim_car_color[NUM_SIM_CAR_COLORS] = { { 1.0, 0.0, 0.0 },
						{ 0.0, 1.0, 0.0 },
						{ 1.0, 1.0, 0.0 },
						{ 0.0, 0.5, 0.5 },
						{ 1.0, 1.0, 1.0 },
						{ 0.1, 0.1, 0.1 } };
sim_status_t          sim_car_status[MAX_NUM_SIM_CARS];

#ifdef HAVE_VIDEOOUT
dgc_videoout_p               vo = NULL;
int                          vo_initialized = 0;
#endif

dgc_grid_p                   ipc_grid;
int                          use_ipc_grid         = TRUE;
char                         default_grid_cell    = PERCEPTION_MAP_OBSTACLE_FREE;

VelodyneInterface                 *velo_interface = NULL;
int                          use_velo_shm         = TRUE;
int                          use_velo_ipc         = TRUE;
char                         utmzone[5]           = "10S ";
uint32_t                     velo_shm_key         = 0;

car_place_t                             goal_place = { 0, 0, 0 };
int                                     show_goal_place = FALSE;
int                                     goal_place_active = GOAL_NOT_SET;

double                                  global_x = 0.0;
double                                  global_y = 0.0;

double                                  ghost_car_x = 0.0;
double                                  ghost_car_y = 0.0;

double                                  map_resolution;
int                                     map_size_x;
int                                     map_size_y;

status_message_t message_buffer[MESSAGE_BUFFER_SIZE];
status_message_t error_buffer[MESSAGE_BUFFER_SIZE];

dgc_laser_mesh_data_t                   lms1_mesh;
dgc_laser_mesh_data_t                   lms3_mesh;

EstopStatus                             estop = { 0, 0, "" };
LaserLaser                              lms1, lms3;
LdlrsLaser                              ldlrs1, ldlrs2;
CanStatus                               can;
RadarSensor                             radar_lrr2[NUM_LRR2_RADARS];
RadarLRR3Sensor                         radar_lrr3[NUM_LRR3_RADARS];
//RadarSensor                             radar[NUM_RADARS];

std::vector <GlsOverlay *>              gls_cache;

char   *gls_select[MAX_GLS_SOURCES+6];
char    gls_enable[MAX_GLS_SOURCES];
double  gls_size[MAX_GLS_SOURCES];

char   *sensor_info[MAX_SENSOR_INFO];

double                                applanix_x = 0.0;
double                                applanix_y = 0.0;
double                                gps_offset_x = 0.0;
double                                gps_offset_y = 0.0;
ApplanixRms                           applanix_rms;
ApplanixDmi                           applanix_dmi;
ApplanixGps                           applanix_gps;
PerceptionObstacles                   obstacles;
PlannerTrajectory                     planner_trajectory;
LocalizePose                          loc_pose;
PassatActuator                        passat_act;
PassatTurnSignal                      passat_signal;
ControllerTarget                      controller_target;
GhostcarPose                          ghost_car;
SimulatorGroundTruth                  simulator_groundtruth;
//SimulatorTag                          simulator_tags;
PlannerMdfGoal                        planner_mdf_goal = { -1, -1, -1, -1, -1, -1, 0.0, "" };

dgc_planner_goal_t                 planner_goal = { 0.0, 0.0, 0.0, 0.0, 0.0, "", 0 };

pthread_mutex_t                    simulator_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    ghost_car_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    ldlrs1_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    ldlrs2_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    lms1_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    lms3_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    radar_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    stop_zone_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    obstacles_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    traj_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    gls_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    message_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    rndf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    dynamic_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    camera_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    goal_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                    map_mutex = PTHREAD_MUTEX_INITIALIZER;

dgc_transform_t                    ldlrs1_offset, ldlrs2_offset;
dgc_transform_t                    lms1_offset, lms3_offset;
dgc_transform_t                    radar_lrr2_offset[NUM_LRR2_RADARS];
dgc_transform_t                    radar_lrr3_offset[NUM_LRR3_RADARS];

double                             applanix_hz = -10.0;
double                             ldlrs1_hz = -10.0;
double                             ldlrs2_hz = -10.0;
double                             lms1_hz = -10.0;
double                             lms3_hz = -10.0;
double                             perception_hz = -10.0;
double                             can_hz = -10.0;
double                             gls_hz = -10.0;
double                             gls_s_hz[MAX_GLS_SOURCES];

int                          occupied_map_cells = -1;

dgc_velodyne_config_p        veloconfig = NULL;
dgc_velodyne_scan_p          scans = NULL;
int                          num_scans = 0;

char                       * imagery_root = NULL;

RoadNetwork*                 rn = NULL;
int                          rndf_valid = 0;
char                       * rndf_filename = NULL;
int                          rndf_generate_display_lists  = FALSE;


void                         timer(int);

dgc_passatwagonmodel_t*       passat = NULL;
dgc_passatwagonmodel_t*       ghost = NULL;
dgc_passatwagonmodel_t*       sim_passat[NUM_SIM_CAR_COLORS];
#ifdef SIMULATOR_FX
dgc_vehicle_gl_model         vehicle_gl_models;
#endif

rndf_display_list          * rndf_dl = NULL;

double darpa_x_offset = 0, darpa_y_offset = 0;
double google_x_offset = 0, google_y_offset = 0;

uint32_t img_id = 0;

/************************************************************************
 *
 *   GRAPHICS THREAD
 *
 ************************************************************************/

void
init_graphics(void)
{
  int              i;

  /*
    void gui3D_setCameraParams(double zoom_sensitivity,
                               double rotate_sensitivity,
			       double move_sensitivity,
			       double min_zoom_range,
			       double camera_fov,
			       double min_clip_range,
			       double max_clip_range);
  */
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 4.0, 50, 0.25, 15000);

#ifdef SIMULATOR_FX
  printf("# INFO: Initializing GL models..\n");
  vehicle_gl_models.init();
#endif

  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_COLOR);

  passat = passatwagonmodel_load( 0.0, 0.0, 0.5, 1.0 );
  ghost  = passatwagonmodel_load( 1.0, 0.0, 0.0, 0.3 );

  fprintf( stderr, "# INFO: allocate simulator cars " );
  for (i=0; i<NUM_SIM_CAR_COLORS; i++) {
    fprintf( stderr, "." );
    sim_passat[i] = passatwagonmodel_load( sim_car_color[i].r,
					   sim_car_color[i].g,
					   sim_car_color[i].b,
					   1.0 );
  }
  for (i=0; i<MAX_NUM_SIM_CARS; i++) {
    snprintf( sim_car_status[i].name, 20, "simulator %02d", i+1 );
    sim_car_status[i].model_nr                   = 0;
    sim_car_status[i].forward_accel_warning_time = 0.0;
    sim_car_status[i].lateral_accel_warning_time = 0.0;
    sim_car_status[i].plan_warning_time          = 0.0;
    sim_car_status[i].collision_warning_time     = 0.0;
  }
  fprintf( stderr, "\n" );

  if(rndf_valid)
    rndf_dl = generate_rndf_display_list(*rn, 0.5, true);


}

void *
graphics_thread(void *ptr)
{
  param_struct_p   param = (param_struct_p)ptr;

#ifdef USE_QT
  fprintf( stderr, "# INFO: initialize QT GUI interface\n" );
  gui3D_set_initFunc(init_graphics);
  qgui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 15);
#else
  fprintf( stderr, "# INFO: initialize GLUT GUI interface\n" );
  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 15);
  init_graphics();
#endif

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_set_mouseFunc(mouse);
  gui3D_set_passiveMotionFunc(motion);

#ifdef USE_QT
  qgui3D_mainloop();
#else
  gui3D_mainloop();
#endif

  return NULL;
}


/************************************************************************
 *
 *   TIMER
 *
 ************************************************************************/

void action_timer(void)
{
  int   i;
  char  str[NAME_LENGTH];
  int mark_lrr2 = 0, mark_lrr3 = 0;

  if (action == ACTION_NOOP)
    return;

  switch (action) {
  case ACTION_READ_TRANSFORMATIONS:
    pint->GetTransform("transform", "ldlrs_laser1", &ldlrs1_offset );
    pint->GetTransform("transform", "ldlrs_laser2", &ldlrs2_offset );
    pint->GetTransform("transform", "sick_laser1", &lms1_offset );
    pint->GetTransform("transform", "sick_laser3", &lms3_offset );
    for (i=0; i<NUM_RADARS; i++) {
      snprintf( str, NAME_LENGTH, "radar%d_offset", i+1);
      if (i != RADAR3_INDEX && i != RADAR6_INDEX){
        pint->GetTransform("transform", str, &(radar_lrr2_offset[mark_lrr2]) );
        mark_lrr2++;
      }
      else {
        pint->GetTransform("transform", str, &(radar_lrr3_offset[mark_lrr3]) );
        mark_lrr3++;
      }
    }
    break;
  case ACTION_RESET_MAP:
    PerceptionMapResetCommand(ipc);
    break;
  case ACTION_REQUEST_MAP:
    PerceptionMapRequestCommand(ipc);
    break;
  case ACTION_SUBSCRIBE:
    ipc_check_subscription();
    break;
  case ACTION_ESTOP_PAUSE:
    EstopSoftstopCommand( ipc, DGC_ESTOP_PAUSE );
    break;
  case ACTION_ESTOP_RUN:
    EstopSoftstopCommand( ipc, DGC_ESTOP_RUN );
    break;
  case ACTION_SEND_GOAL:
    PlannerGoalCommand( ipc, goal_place.lat, goal_place.lon, goal_place.theta );
    break;
  case ACTION_GHOST_CAR_SYNC:
    GhostcarSyncCommand( ipc, applanix_current_pose()->latitude,
			 applanix_current_pose()->longitude );
    break;
  default:
    break;
  }
  action = ACTION_NOOP;
  display_redraw();
}

void *
timer_thread(void *)
{

  static double   last_time = 0;
  double          current_time = dgc_get_time();

  while (1) {

    if (use_velo_shm && show_velodyne) {
      while (velo_interface->ScanDataWaiting()) {
	num_scans =
	  velo_interface->ReadCurrentScans(scans, MAX_NUM_SCANS);
      }
    }

    if (current_time-last_time>0.1) {
      pthread_mutex_lock(&gls_mutex);
      for(int i = 0; i < (signed)gls_cache.size(); i++) {
	if (current_time-gls_cache[i]->timestamp>GLS_REMOVE_TIME) {
	  free(gls_cache[i]->byte);
	  gls_cache.erase(gls_cache.begin()+i);
	}
      }
      pthread_mutex_unlock(&gls_mutex);
    }

    usleep(1000);
  }

  return NULL;
}

/************************************************************************
 *
 *   MAIN LOOP
 *
 ************************************************************************/

int
main(int argc, char **argv)
{
  int                  i;
  pthread_t            thread;
  param_struct_t       param;

  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-no-shm")==0)){
      use_velo_shm = FALSE;
    }
  }

  /************************************************************************/
    // Init IL structures
  ilInit();
  ilGenImages(1, &img_id);
  ilBindImage(img_id);

  /************************************************************************/

  dgc_velodyne_get_config(&veloconfig);

  scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS *
				      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);

  /************************************************************************/

  viewer_ipc_init( argc, argv );

  fprintf( stderr, "# INFO: imagery root = %s\n", imagery_root );

  /************************************************************************/

  if (use_velo_shm) {
    /* create connection to velodyne interface */
    velo_interface = new VelodyneShmInterface;
    if(velo_shm_key != 0)
      velo_interface->SetKey(velo_shm_key);
    if(velo_interface->CreateClient() < 0) {
      fprintf( stderr, "# INFO: disable velodyne support ...\n" );
      use_velo_shm = FALSE;
    }
  }

  if (use_ipc_grid) {
    /* allocate rolling grid */
    fprintf( stderr,
	     "# INFO: allocate grid map (%dx%d) with resolution %.2f\n",
	     (int) (map_size_x/map_resolution),
	     (int) (map_size_y/map_resolution),
	     map_resolution );
    ipc_grid = dgc_grid_initialize( map_resolution,
				    (int) (map_size_x/map_resolution),
				    (int) (map_size_y/map_resolution),
				    sizeof(char),
				    &default_grid_cell );
    if (ipc_grid==NULL) {
      fprintf( stderr, "# WARNING: allocation of grid map failed\n" );
      use_ipc_grid = FALSE;
    }
  }

  init_mesh_data( &lms1_mesh );
  init_mesh_data( &lms3_mesh );

  /************************************************************************/
  /* load the RNDF file, if available */

  rn = new RoadNetwork;
  if(rn->loadRNDF(rndf_filename)) {
    rndf_valid = 1;
    rndf_generate_display_lists = 1;
//    fprintf(stderr, "# INFO: Marking yield and merge exits ... ");  // TODO: ...
//    rndf->mark_yield_exits();
//    rndf->mark_merge_exits();
//    fprintf(stderr, "done.\n");
  }
  else
    fprintf(stderr, "# WARNING: Invalid RNDF %s\n", rndf_filename);

  /************************************************************************/
  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, timer_thread, &param);

  /* ipc thread */
  ipc->AddTimer( 0.10, action_timer);

  /************************************************************************/
  /* ipc thread */
  ipc->Dispatch();

  return 0;
}
