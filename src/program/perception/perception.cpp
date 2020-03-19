#include "perception.h"
#include "utils.h"
#include <ipc_std_interface.h>
#include <velodyne_shm_interface.h>
#include <IL/il.h>
#include <track_manager.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace dgc;
using namespace vlr;
using namespace vlr::rndf;
using namespace track_manager;

#define MIN_NUM_POINTS 75
#define MIN_NUM_FRAMES 10
#define MAX_NUM_FRAMES 10000
#define MAX_TIME_NOT_SEEN 5.0 // After not seeing a track for this number of seconds, call it complete.
#define TIMEOUT 60 // After not seeing any new data for this number of seconds, terminate.

pthread_mutex_t                 shutdown_mutex = PTHREAD_MUTEX_INITIALIZER;
int                             shutdown = 3;

IpcInterface *ipc = NULL;

bool                            data_ready_to_publish = false;
pthread_mutex_t                 applanix_mutex                = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 localize_mutex                = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 integration_mutex             = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 publish_mutex                 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 ldlrs_mutex[NUM_LDLRS_LASERS] = { PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER };
pthread_mutex_t                 radar_mutex[NUM_RADARS]       = { PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t                 fsm_mutex                     = PTHREAD_MUTEX_INITIALIZER;

LdlrsLaser                      ldlrs[NUM_LDLRS_LASERS];
double                          ldlrs_ts[NUM_LDLRS_LASERS];

RadarSensor                     radar_lrr2[NUM_LRR2_RADARS];
RadarLRR3Sensor                 radar_lrr3[NUM_LRR3_RADARS];

vlr::GlsOverlay               * gls = NULL;
unsigned short                  counter                            = 0;
int                             velodyne_ctr                       = 0;

dgc_perception_map_cells_t     obstacles_s_publish;
dgc_perception_map_cell_p     obstacles_s_cell_pointer;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked_publish;
dgc_grid_t                      grid_publish;
double                          velodyne_ts  = 0;
double                          last_velodyne_ts  = 0;
dgc_velodyne_data_p             velodyne;
dgc_velodyne_data_p             last_velodyne; // double buffer to keep velodyne data around for an extra frame

double                          scan_resolution                    = 0;
LocalizePose                    localize_pos = { 0, 0.0, 0.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "" };

PerceptionObstacles             msg;
PlannerFsmState                 fsm_state;

grid_stat_t                     grid_stat;
dgc_grid_p                      grid;

VelodyneInterface              *velo_interface = NULL;
uint32_t                        velo_shm_key = 0;

dgc_perception_map_cell_p       default_map_cell     = NULL;
dgc_perception_map_cell_p       default_terrain_cell = NULL;
short                           default_z_cell       = std::numeric_limits<short>::max();

double                          publish_interval;
double                          timer_interval;

char                          * rndf_filename = NULL;
dgc_global_pose_t               global_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"}; // This is actually in smooth coordinates!  Growl.
dgc_pose_t                      g_smooth;

uint32_t img_id;

RoadNetwork* road_network = NULL;
int rndf_valid = 0;

void rndf_load_file(char *filename) {
  /* load the RNDF file, if available */
  fprintf(stderr, "# INFO: load rndf file\n");
  road_network = new RoadNetwork;

//  rndf = new rndf_file;
  if (road_network->loadRNDF(filename))
    rndf_valid = 1;
}

/****************************************************************************/

perception_settings_t           settings;

MultiBooster* booster = NULL;
ClassifierPipeline* g_classifier_pipeline = NULL;

/****************************************************************************/

double get_z() {
  return velodyne->scans[0].robot.z;
}

void
perception_publish( void )
{
  static int do_publishing = FALSE;

  if (!do_publishing) {
    do_publishing = TRUE;
    pthread_mutex_lock(&publish_mutex);
    {
      printf("(3)Tracks: %d\n", (int)obstacles_tracked.size());
      perception_publish_obstacles( ipc, &grid_publish, &obstacles_s_publish, obstacles_tracked_publish, counter );

      if (settings.gls_output) {
        glsSend(ipc, gls);
        gls_clear(gls);
      }

      //fprintf( stderr, "#INFO: [OBSTACLES: %05d]\n", obstacles_s->num);

      obstacles_s->num = 0;
    }
    pthread_mutex_unlock(&publish_mutex);
    do_publishing = FALSE;
  }
}

/******************************************************************
 * APPLANIX handler
 ******************************************************************/
void
applanix_handler( ApplanixPose *pose )
{
  pthread_mutex_lock(&applanix_mutex);
  pthread_mutex_lock(&localize_mutex);
  applanix_history_add( pose );

  g_smooth.x = pose->smooth_x;
  g_smooth.y = pose->smooth_y;
  g_smooth.z = pose->smooth_z;
  g_smooth.roll = pose->roll;
  g_smooth.pitch = pose->pitch;
  g_smooth.yaw = pose->yaw;
  
  global_pos.x =  pose->smooth_x + localize_pos.x_offset;
  global_pos.y =  pose->smooth_y + localize_pos.y_offset;
  global_pos.z =  pose->smooth_z;
  global_pos.roll = pose->roll;
  global_pos.pitch = pose->pitch;
  global_pos.yaw = pose->yaw;
  pthread_mutex_unlock(&localize_mutex);
  pthread_mutex_unlock(&applanix_mutex);
}

/******************************************************************
 * SENSOR_DATA handler
 ******************************************************************/
void
ldlrs1_handler( void )
{
  pthread_mutex_lock(&ldlrs_mutex[0]);
  ldlrs_ts[0] = dgc_get_time();
  pthread_mutex_unlock(&ldlrs_mutex[0]);
}

void
ldlrs2_handler( void )
{
  pthread_mutex_lock(&ldlrs_mutex[1]);
  ldlrs_ts[1] = dgc_get_time();
  pthread_mutex_unlock(&ldlrs_mutex[1]);
}

/************************************************************
 * Track management
 ************************************************************/

map<int, boost::shared_ptr<Track> > g_track_map;
string g_track_manager_filename;

//! Returns true if track should be saved.
bool trackValid(const Track& track) {
  if(track.frames_.size() < MIN_NUM_FRAMES)
    return false;

  bool enough_points = false;
  for(size_t i = 0; i < track.frames_.size(); ++i) {
    if(track.frames_[i]->cloud_->get_points_size() > MIN_NUM_POINTS) {
      enough_points = true;
      break;
    }
  }
  if(!enough_points)
    return false;

  return true;
}

//! Returns true if track should be deleted.
bool trackComplete(const Track& track) {
  if(track.frames_.size() >= MAX_NUM_FRAMES)
    return true;
  if(track.frames_.back()->timestamp_ + MAX_TIME_NOT_SEEN < velodyne->scans->timestamp)
    return true;

  return false;
}

void processCompletedTracks(bool force = false) {
  map<int, boost::shared_ptr<Track> >::iterator it;
  for(it = g_track_map.begin(); it != g_track_map.end(); ) {
    if(!force && !trackComplete(*it->second)) {
      ++it;
      continue;
    }
    
    if(trackValid(*it->second))
      streamTrack(g_track_manager_filename, *it->second);
    
    g_track_map.erase(it++);
  }
}
 
void terminatePerception(int param) {
  pthread_mutex_lock(&shutdown_mutex);
  shutdown = 2;
  pthread_mutex_unlock(&shutdown_mutex);
  // while(true) {
  //   pthread_mutex_lock(&shutdown_mutex);
  //   if(shutdown == 0) {
  //     pthread_mutex_unlock(&shutdown_mutex);
  //     break;
  //   }
  //   pthread_mutex_unlock(&shutdown_mutex);
  //   usleep(100);
  // }
    
  cout << "Terminating: streaming all valid tracks to disk... ";
  cout.flush();
  processCompletedTracks(true);
  cout << "done." << endl;
  //exit(0);
}


//! Add new tracks, update existing tracks, stream and / or throw out completed tracks.
bool updateTrackManager() {
  bool new_tracks = false;

  // -- Add new tracks and append new frames to old tracks.
  for(size_t i = 0; i < obstacles_tracked.size(); ++i) {
    if(obstacles_tracked[i]->getPoints().empty())
      continue;

    int id = obstacles_tracked[i]->id;
    double timestamp = obstacles_tracked[i]->timestamp_observation_;
//     cout << setprecision(16)
// 	 << "obstacles_tracked[i]->time_: "
// 	 << obstacles_tracked[i]->time_
// 	 << endl
// 	 << "obstacles_tracked[i]->timestamp_observation_: "
// 	 << obstacles_tracked[i]->timestamp_observation_
// 	 << endl
// 	 << "obstacles_tracked[i]->lastObservation_->time_: "
// 	 << obstacles_tracked[i]->lastObservation_->time_
// 	 << endl
// 	 << "obstacles_tracked[i]->time_ - obstacles_tracked[i]->lastObservation_->time_: "
// 	 << obstacles_tracked[i]->time_ - obstacles_tracked[i]->lastObservation_->time_
// 	 << endl
// 	 << "obstacles_tracked[i]->time_ - obstacles_tracked[i]->timestamp_observation_: "
// 	 << obstacles_tracked[i]->time_ - obstacles_tracked[i]->timestamp_observation_
// 	 << endl;
    

    // Make a new track if necessary.
    if(g_track_map.count(id) == 0)  {
      vector< boost::shared_ptr<Frame> > no_frames;
      g_track_map[id] = boost::shared_ptr<Track>(new Track("unlabeled", velodyne_offset, no_frames));
      g_track_map[id]->reserve(1000);
    }

    // Add a new frame if there is a new cluster.
    if(g_track_map[id]->frames_.empty() ||
       fabs(g_track_map[id]->frames_.back()->timestamp_ - timestamp) > 0.01) {
      boost::shared_ptr<sensor_msgs::PointCloud> cloud = dgcToRos(obstacles_tracked[i]->getPoints());

//       dgc::ApplanixPose* pose = applanix_pose(timestamp);
//       dgc_pose_t robot;
//       robot.x = pose->smooth_x;
//       robot.y = pose->smooth_y;
//       robot.z = pose->smooth_z;
//       robot.roll = pose->roll;
//       robot.pitch = pose->pitch;
//       robot.yaw = pose->yaw;
//       cout << "-----" << endl;
//       cout << "xyz of robot at beginning of spin: " << robot.x << " " << robot.y << " " << robot.z << endl;
      dgc_pose_t& r2 = obstacles_tracked[i]->lastObservation_->robot_pose_when_observed_;
//      cout << "xyz of robot_pose_when_observed_: " << r2.x << " " << r2.y << " " << r2.z << endl;
      
      pthread_mutex_lock(&applanix_mutex);
      pthread_mutex_lock(&localize_mutex);
      g_track_map[id]->insertFrame(cloud, timestamp, r2);
      pthread_mutex_unlock(&localize_mutex);
      pthread_mutex_unlock(&applanix_mutex);
      
      new_tracks = true;
    }
  }
  
  processCompletedTracks();
  return new_tracks;
}


/******************************************************************
 * TIMER  handler
 ******************************************************************/
void*
perception_thread(void*)
{

  perception_init();
  double last_new_tracks = dgc_get_time();
  while(true) {
    double t1 = Time::current();
    bool new_data = false;
    while(velo_interface->ScanDataWaiting()) {
      velodyne->num_scans =
        velo_interface->ReadScans(velodyne->scans, velodyne->allocated_scans);

      if (velodyne->num_scans>0) {
        velodyne->preprocessed = FALSE;
        velodyne_ts = dgc_get_time();
        velodyne_ctr++;
        new_data = true;
      }
    }

    if (new_data) {
      integrate_sensors(velodyne);
      //perception_publish();
      data_ready_to_publish = true;
      dgc_velodyne_data_p tmp = velodyne;
      velodyne = last_velodyne;
      last_velodyne = tmp;
      last_velodyne_ts = velodyne_ts;
    }

    // -- Horrible hack: dump out the tracks as we get them.
    if(getenv("EXTRACT_TRACKS")) { 
      double start = dgc_get_time();
      bool new_tracks = updateTrackManager();
      cout << "updating tracks took " << dgc_get_time() - start << " seconds." << endl;
      if(new_tracks)
	last_new_tracks = dgc_get_time();
      if(!new_tracks && (dgc_get_time() - last_new_tracks) > TIMEOUT)
	terminatePerception(1);
    }
    
    double t2 = Time::current();
    double extra = 1.0 / 10 - (t2 - t1);
    if(extra > 0) {
      usleep((int) rint(extra * 1e6));
    }
    else if (extra < 0) {
      printf("SLOW %f\n", t2 - t1);
    }
    pthread_mutex_lock(&shutdown_mutex);
    if(shutdown == 2) {
      printf("Perception shutting down\n");
      shutdown = 0;
      pthread_mutex_unlock(&shutdown_mutex);
      return NULL;
    }
    pthread_mutex_unlock(&shutdown_mutex);
  }
  return NULL;
}

/******************************************************************
 * READ parameters
 ******************************************************************/
void
frequency_change_handler(void)
{
  publish_interval = (1.0 / (double) settings.rate_in_hz);
  timer_interval   = (1.0 / (double) (2.0*settings.rate_in_hz) );
}

void
scan_resolution_handler(void)
{
  settings.virtual_scan_resolution = dgc_d2r(scan_resolution);
}

void
rndf_change_handler(void)
{
  //fprintf( stderr, "# INFO: rndf changed. Quit program.\n" );
  exit(0);
}


void
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
      {"velodyne", "shm_key", DGC_PARAM_INT, &velo_shm_key, 0, NULL},
      {"perception", "hz",   DGC_PARAM_DOUBLE,                 &settings.rate_in_hz,  1, ParamCB(frequency_change_handler)},
      {"perception", "max_sensor_delay",   DGC_PARAM_DOUBLE,   &settings.max_sensor_delay,  1, NULL },
      {"perception", "clear_sensor_data",   DGC_PARAM_ONOFF,   &settings.clear_sensor_data,  1, NULL },
      {"perception", "extract_dynamic",   DGC_PARAM_ONOFF,     &settings.extract_dynamic,       1, NULL},
      {"perception", "overpass_height",   DGC_PARAM_DOUBLE,    &settings.overpass_height,      1, NULL},

      {"perception", "num_threads",   DGC_PARAM_INT,           &settings.num_threads,      1, NULL},

      {"perception", "map_resolution",   DGC_PARAM_DOUBLE,     &settings.map_resolution,  0, NULL },
      {"perception", "map_size_x",   DGC_PARAM_DOUBLE,         &settings.map_size_x,  0, NULL },
      {"perception", "map_size_y",   DGC_PARAM_DOUBLE,         &settings.map_size_y,  0, NULL },
      {"perception", "map_cell_threshold",   DGC_PARAM_DOUBLE, &settings.map_cell_threshold,  1, NULL },
      {"perception", "map_cell_min_hits",   DGC_PARAM_INT,     &settings.map_cell_min_hits,  1, NULL },
      {"perception", "map_cell_increase",   DGC_PARAM_INT,     &settings.map_cell_increase,  1, NULL },
      {"perception", "map_ray_tracing" , DGC_PARAM_ONOFF,      &settings.map_ray_tracing,      1, NULL},
      {"perception", "z_resolution",   DGC_PARAM_DOUBLE,       &settings.z_resolution,  0, NULL },
      {"perception", "z_obstacle_height",   DGC_PARAM_DOUBLE,  &settings.z_obstacle_height,  0, NULL },

      {"perception", "gls_output" , DGC_PARAM_ONOFF,     &settings.gls_output,      1, NULL},
      {"perception", "show_virtual_scan" , DGC_PARAM_ONOFF,    &settings.show_virtual_scan,      1, NULL},
      {"perception", "show_ray_tracing" , DGC_PARAM_ONOFF,     &settings.show_ray_tracing,      1, NULL},

      {"rndf",      "rndf_file",    DGC_PARAM_FILENAME,    &rndf_filename, 1, ParamCB(rndf_change_handler) },


      {"perception", "use_ldlrs1",  DGC_PARAM_ONOFF,     &settings.use_ldlrs[0],      1, NULL},
      {"perception", "use_ldlrs2" , DGC_PARAM_ONOFF,     &settings.use_ldlrs[1],      1, NULL},
      {"perception", "ldlrs_min_distance" , DGC_PARAM_DOUBLE,   &settings.ldlrs_min_dist,  1, NULL},
      {"perception", "ldlrs_max_distance" , DGC_PARAM_DOUBLE,   &settings.ldlrs_max_dist,  1, NULL},
      {"transform", "ldlrs_laser1", DGC_PARAM_TRANSFORM, &ldlrs_offset[0],   0, NULL},
      {"transform", "ldlrs_laser2", DGC_PARAM_TRANSFORM, &ldlrs_offset[1],   0, NULL},

      {"transform", "radar1", DGC_PARAM_TRANSFORM, &radar_offset[0],   0, NULL},
      {"transform", "radar2", DGC_PARAM_TRANSFORM, &radar_offset[1],   0, NULL},
      {"transform", "radar3", DGC_PARAM_TRANSFORM, &radar_offset[2],   0, NULL},
      {"transform", "radar4", DGC_PARAM_TRANSFORM, &radar_offset[3],   0, NULL},
      {"transform", "radar5", DGC_PARAM_TRANSFORM, &radar_offset[4],   0, NULL},
      {"transform", "radar6", DGC_PARAM_TRANSFORM, &radar_offset[5],   0, NULL},

      {"perception", "use_velodyne" , DGC_PARAM_ONOFF,     &settings.use_velodyne,      1, NULL},
      {"perception", "velodyne_threshold_factor", DGC_PARAM_DOUBLE,   &settings.velodyne_threshold_factor, 1, NULL },
      {"perception", "velodyne_max_range", DGC_PARAM_DOUBLE,   &settings.velodyne_max_range, 1, NULL },
      {"perception", "velodyne_min_beam_diff", DGC_PARAM_DOUBLE,   &settings.velodyne_min_beam_diff, 0, NULL },
      {"perception", "velodyne_sync",   DGC_PARAM_ONOFF,    &settings.velodyne_sync,  1, NULL },
      {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &velodyne_offset,    0, NULL},
      {"velodyne", "cal_file", DGC_PARAM_FILENAME, &settings.velodyne_cal, 0, NULL},


      {"perception", "virtual_scan_resolution", DGC_PARAM_DOUBLE,   &scan_resolution, 1, ParamCB(scan_resolution_handler) },

      { "segmentation", "min_points", DGC_PARAM_INT, &settings.segmentation_settings.min_points, 1, NULL },
      { "segmentation", "max_points", DGC_PARAM_INT, &settings.segmentation_settings.max_points, 1, NULL },
      { "segmentation", "min_height", DGC_PARAM_DOUBLE, &settings.segmentation_settings.min_height, 1, NULL },
      { "segmentation", "kernel_size", DGC_PARAM_INT, &settings.segmentation_settings.kernel_size, 1, NULL },
      { "segmentation", "gls_output", DGC_PARAM_ONOFF, &settings.segmentation_settings.gls_output, 1, NULL },


      { "tracker", "filter_rndf_max_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_distance, 1, NULL },
      { "tracker", "filter_rndf_max_pedestrian_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_pedestrian_distance, 1, NULL },
      { "tracker", "filter_rndf", DGC_PARAM_ONOFF, &settings.tracker_settings.filter_rndf, 1, NULL },
      { "tracker", "merge_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.merge_dist, 1, NULL },
      { "tracker", "lateral_merge_dist", DGC_PARAM_DOUBLE, &settings.tracker_settings.lateral_merge_dist, 1, NULL },
      { "tracker", "pedestrian_classifier", DGC_PARAM_FILENAME, &settings.tracker_settings.classifier_filename, 1, NULL },
      /*{ "tracker", "default_loc_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_loc_stddev, 1, NULL },
      { "tracker", "min_car_width", DGC_PARAM_DOUBLE, &settings.kf_settings.min_car_width, 1, NULL },
      { "tracker", "min_car_length", DGC_PARAM_DOUBLE, &settings.kf_settings.min_car_length, 1, NULL },
      { "tracker", "default_vel_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_vel_stddev, 1, NULL },
      { "tracker", "transition_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.transition_stddev, 1, NULL },
      { "tracker", "velodyne_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_stddev, 1, NULL },
//      { "tracker", "velodyne_max_range", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_max_range, 1, NULL },
      { "tracker", "radar_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.radar_stddev, 1, NULL },
      { "tracker", "radar_max_range", DGC_PARAM_DOUBLE, &settings.kf_settings.radar_max_range, 1, NULL },
      { "tracker", "max_dist_correspondence", DGC_PARAM_DOUBLE, &settings.kf_settings.max_dist_correspondence, 1, NULL },
      { "tracker", "confidence_increment_obs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_obs, 1, NULL },
      { "tracker", "confidence_increment_unobs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_unobs, 1, NULL },

      { "tracker", "confidence_decay", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_decay, 1, NULL },
      { "tracker", "confidence_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_max, 1, NULL },
      { "tracker", "confidence_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_min, 1, NULL },
      { "tracker", "confidence_initial_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_min, 1, NULL },
      { "tracker", "confidence_initial_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_max, 1, NULL },
      { "tracker", "confidence_track_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_track_min, 1, NULL },
      { "tracker", "confidence_publish_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_publish_min, 1, NULL },*/
      { "tracker", "correspondence_threshold", DGC_PARAM_DOUBLE, &settings.kf_settings.correspondence_threshold, 1, NULL },
      { "tracker", "pruning_threshold", DGC_PARAM_DOUBLE, &settings.kf_settings.pruning_threshold, 1, NULL },
      { "tracker", "measurement_variance", DGC_PARAM_DOUBLE, &settings.kf_settings.measurement_variance, 1, NULL },
      { "tracker", "position_variance", DGC_PARAM_DOUBLE, &settings.kf_settings.position_variance, 1, NULL },
      { "tracker", "velocity_variance", DGC_PARAM_DOUBLE, &settings.kf_settings.velocity_variance, 1, NULL },
      { "tracker", "initial_position_variance", DGC_PARAM_DOUBLE, &settings.kf_settings.initial_position_variance, 1, NULL },
      { "tracker", "initial_velocity_variance", DGC_PARAM_DOUBLE, &settings.kf_settings.initial_velocity_variance, 1, NULL }

  };

  pint->InstallParams(argc, argv,
      params, sizeof(params)/sizeof(params[0]));

  //settings.kf_settings.velodyne_max_range = settings.velodyne_max_range;
  settings.publish_interval = (1.0 / (double) settings.rate_in_hz);
  timer_interval   = (1.0 / (double) (2.0*settings.rate_in_hz) );
  settings.virtual_scan_resolution = dgc_d2r(scan_resolution);
}

/******************************************************************
 *
 *    MAIN
 *
 ******************************************************************/
int
main(int argc, char **argv)
{
  // -- Set up track saving.
  if(getenv("EXTRACT_TRACKS")) { 
    signal(SIGINT, terminatePerception);
    assert(argc == 2);
    g_track_manager_filename = argv[1];
    //assert(!boost::filesystem::exists(g_track_manager_filename));
    TrackManager tm;
    tm.save(g_track_manager_filename);
  }

  
  ParamInterface *pint;

  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->Connect("perception") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  settings.velodyne_cal = NULL;

  read_parameters(pint, argc, argv);

  perception_register_ipc_messages(ipc);

  // initialize DevIl used for map publishing
  ilInit();
  ilGenImages(1, &img_id);
  ilBindImage(img_id);

  // -- Check for a classifier.
  if(dgc_file_exists(settings.tracker_settings.classifier_filename)) {
    booster = new MultiBooster(settings.tracker_settings.classifier_filename);
  }

  if(booster) { 
    //cout << booster->status(false) << endl;
    
    // -- Translate it to use the features that we have currently.
    assert(booster->class_map_.isPermutation(getClassNames()));
    NameMapping feature_map(getDescriptorNames());
    if(!booster->feature_map_.isPermutation(feature_map)) { 
      vector<string> not_in_code;
      vector<string> not_in_classifier;
      booster->feature_map_.diff(feature_map, &not_in_code, &not_in_classifier);
      cout << "Descriptors in classifier " << settings.tracker_settings.classifier_filename
	   << " but not present in code: " << endl;
      for(size_t i = 0; i < not_in_code.size(); ++i)
	cout << i << ":   " << not_in_code[i] << endl;
      cout << endl << endl;

      cout << "Descriptors in code but not in classifier: " << endl;
      for(size_t i = 0; i < not_in_classifier.size(); ++i)
	cout << i << ":   " << not_in_classifier[i] << endl;
      cout << endl << endl;
      return 1;
    }
    booster->applyNewMappings(NameMapping(getClassNames()), NameMapping(getDescriptorNames()));

    // -- Construct the classifier pipeline.
    int num_threads = settings.num_threads;
    if(getenv("NUM_THREADS"))
      num_threads = atoi(getenv("NUM_THREADS"));
    g_classifier_pipeline = new ClassifierPipeline(booster, num_threads);
    printf("Loaded %s, using %d threads for pipeline classification.\n", settings.tracker_settings.classifier_filename, num_threads);
    
  }
  else {
    printf("#WARNING: no classifier found at %s\n", settings.tracker_settings.classifier_filename);
  }
  
  /* allocate rolling grid */
  default_map_cell =
    (dgc_perception_map_cell_p)calloc(1, sizeof(dgc_perception_map_cell_t));
  dgc_test_alloc(default_map_cell);

  default_map_cell->max       = -FLT_MAX;
  default_map_cell->min       = FLT_MAX;
  default_map_cell->hits      = 0;
  default_map_cell->seen      = 0;
  default_map_cell->last_min  = 0;
  default_map_cell->last_max  = 0;
  default_map_cell->last_observed  = 0;
  default_map_cell->last_obstacle  = 0;
  default_map_cell->last_dynamic   = 0;
  default_map_cell->last_mod  = 0;
  default_map_cell->region    = 0;
  default_map_cell->obstacle  = PERCEPTION_MAP_OBSTACLE_FREE;
  default_map_cell->street    = 1;

  default_terrain_cell =
        (dgc_perception_map_cell_p)calloc(1, sizeof(dgc_perception_map_cell_t));
  dgc_test_alloc(default_terrain_cell);
  *default_terrain_cell = *default_map_cell;

  grid_stat.mapsize.x    = (int) (settings.map_size_x/settings.map_resolution);
  grid_stat.mapsize.y    = (int) (settings.map_size_y/settings.map_resolution);
  grid_stat.resolution   = settings.map_resolution;
  grid_stat.center.x     = 0;
  grid_stat.center.y     = 0;
  grid_stat.z_resolution = settings.z_resolution;

  fprintf( stderr, "# INFO: initialize grid map (%.1fm x %.1fm - %.2fm resolution)\n",
      settings.map_size_x, settings.map_size_y, settings.map_resolution );

  grid =
    dgc_grid_initialize( grid_stat.resolution,
        grid_stat.mapsize.x,
        grid_stat.mapsize.y,
        sizeof(dgc_perception_map_cell_t),
        default_map_cell );


  /* create connection to velodyne interface */
  velo_interface = new VelodyneShmInterface;
  if(velo_shm_key != 0)
    velo_interface->SetKey(velo_shm_key);
  if(velo_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to velodyne interface.\n");

  applanix_history_init( APPLANIX_HISTORY_LENGTH );

  /*********************************************************************
   *
   *   IPC
   *
   *********************************************************************/

  ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL,
      &applanix_mutex);

  ipc->Subscribe(LocalizePoseID, &localize_pos, DGC_SUBSCRIBE_ALL, &localize_mutex);

  ipc->Subscribe(LdlrsLaser1ID, &ldlrs[0], ldlrs1_handler, DGC_SUBSCRIBE_ALL,
      &ldlrs_mutex[0]);
  ipc->Subscribe(LdlrsLaser2ID, &ldlrs[1], ldlrs2_handler, DGC_SUBSCRIBE_ALL,
      &ldlrs_mutex[1]);

  ipc->Subscribe(RadarSensor1ID, &radar_lrr2[0], DGC_SUBSCRIBE_LATEST, &radar_mutex[0]);
  ipc->Subscribe(RadarSensor2ID, &radar_lrr2[1], DGC_SUBSCRIBE_LATEST, &radar_mutex[1]);
  ipc->Subscribe(RadarLRR3Sensor3ID, &radar_lrr3[0], DGC_SUBSCRIBE_LATEST, &radar_mutex[2]);
  ipc->Subscribe(RadarSensor4ID, &radar_lrr2[2], DGC_SUBSCRIBE_LATEST, &radar_mutex[3]);
  ipc->Subscribe(RadarSensor5ID, &radar_lrr2[3], DGC_SUBSCRIBE_LATEST, &radar_mutex[4]);
  ipc->Subscribe(RadarLRR3Sensor6ID, &radar_lrr3[1], DGC_SUBSCRIBE_LATEST, &radar_mutex[5]);

  ipc->Subscribe(PlannerFsmStateID, &fsm_state, DGC_SUBSCRIBE_LATEST,
      &fsm_mutex);

  gls = vlr::gls_alloc("PERCEPTION");

  rndf_load_file( rndf_filename );

  velodyne = (dgc_velodyne_data_p) malloc(sizeof(dgc_velodyne_data_t));
  last_velodyne = (dgc_velodyne_data_p) malloc(sizeof(dgc_velodyne_data_t));

  velodyne_init( velodyne );
  velodyne_init( last_velodyne );

  dgc_transform_copy(velodyne->config->offset, velodyne_offset);
  dgc_transform_copy(last_velodyne->config->offset, velodyne_offset);

  pthread_t perception_thread_id;
  pthread_attr_t def_thread_attr;
  pthread_attr_init(&def_thread_attr);
  pthread_attr_setdetachstate(&def_thread_attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&perception_thread_id, &def_thread_attr, perception_thread, NULL);
  //double start = Time::current();
  while (ipc->Sleep(0.005)>= 0 ) {
    static double time = 0.0;
    if(data_ready_to_publish) {
      if(time != 0.0) {
        printf("Publishing again in %f seconds\n", Time::current() - time);
      }
      time = Time::current();
      perception_publish();
      data_ready_to_publish = false;
    }

    pthread_mutex_lock(&shutdown_mutex);
    if(shutdown == 0) {
      pthread_mutex_unlock(&shutdown_mutex);
      return 0;
    }
    pthread_mutex_unlock(&shutdown_mutex);
    

    // if(start + 300 < time) {
    //   pthread_mutex_lock(&shutdown_mutex);
    //   shutdown = 2;
    //   printf("Setting shutdown\n");
    //   pthread_mutex_unlock(&shutdown_mutex);
    //   while(true) {
    //     usleep(10000);
    //     pthread_mutex_lock(&shutdown_mutex);
    //     if(shutdown == 0) {
    //       pthread_mutex_unlock(&shutdown_mutex);
    //       return 0;
    //     }
    //     pthread_mutex_unlock(&shutdown_mutex);
    //   }  
    //    }
  }
  return 0;
}
