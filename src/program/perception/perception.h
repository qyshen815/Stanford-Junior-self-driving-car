#ifndef DGC_PERCEPTION_H
#define DGC_PERCEPTION_H

#define USE_GRID_SEGMENTER

#ifndef USE_GRID_SEGMENTER
  #define  USE_LASER_SEGMENTER
#else
  #undef  USE_LASER_SEGMENTER
#endif

#undef  USE_LASER_SEGMENTER

#include <roadrunner.h>
#include <applanix_interface.h>

#include <heartbeat_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
#include <radar_interface.h>
#include <velocore.h>
#include <velo_support.h>
#include <param_interface.h>
#include <ldlrs_interface.h>
#include <planner_interface.h>
#include <grid.h>
#include <limits.h>
#include <passat_constants.h>
#include <gls_interface.h>
#include <aw_roadNetwork.h>
#include <stdint.h>

#include <tr1/memory>

#include "applanix_history.h"
#include "obstacle.h"
#include "tracked_obstacle.h"

#include "perception_defines.h"
#include "perception_types.h"

#include "multibooster_support.h"

  /************************************************************************
   *
   * VARIABLES
   *
   ************************************************************************/

  extern pthread_mutex_t                 shutdown_mutex;
  extern int                             shutdown;
  extern pthread_mutex_t                 applanix_mutex;
  extern pthread_mutex_t                 localize_mutex;
  extern pthread_mutex_t                 integration_mutex;
  extern pthread_mutex_t                 tracking_mutex;
  extern pthread_mutex_t                 publish_mutex;
  extern pthread_mutex_t                 ldlrs_mutex[NUM_LDLRS_LASERS];
  extern pthread_mutex_t                 velodyne_mutex;
  extern pthread_mutex_t                 radar_mutex[NUM_RADARS];
  extern pthread_mutex_t                 fsm_mutex;
  extern pthread_mutex_t                 virtual_scan_mutex;
  extern pthread_cond_t                  virtual_tracking_cond;

  extern bool                            data_ready_to_publish;
  extern vlr::GlsOverlay                 *gls;
  
  extern dgc_transform_t                 ldlrs_offset[NUM_LDLRS_LASERS];

  extern dgc::RadarSensor                radar_lrr2[NUM_LRR2_RADARS];
  extern dgc::RadarLRR3Sensor            radar_lrr3[NUM_LRR3_RADARS];
  extern dgc_transform_t                 radar_offset[NUM_RADARS];

  extern vlr::PerceptionObstacleList     dynamic_msg;
  extern pthread_mutex_t                        dynamic_msg_mutex;
  extern pthread_mutex_t                        dynamic_semaphore_mutex;

  extern dgc::LocalizePose               localize_pos;

  extern dgc::LdlrsLaser                 ldlrs[NUM_LDLRS_LASERS];
  extern double                          ldlrs_ts[NUM_LDLRS_LASERS];

  extern dgc_transform_t                 velodyne_offset;
  extern int                             velodyne_ctr;
  extern double                          velodyne_ts;
  extern double                          last_velodyne_ts;
  extern vlr::PerceptionScan             virtual_scan[NUM_VIRTUAL_SCANS];
  extern dgc::PlannerFsmState            fsm_state;
  
  extern vlr::PerceptionObstacles        msg;
  extern dgc_grid_p                      grid;
  extern dgc_grid_t                      grid_publish;
  extern dgc_perception_map_cell_p       default_map_cell;
  extern dgc_perception_map_cell_p       default_terrain_cell;
  extern short                           default_z_cell;
  extern grid_stat_t                     grid_stat;
  extern int                             send_map_reset;

  extern dgc_grid_p                      terrain_grid;
  extern dgc_grid_p                      z_grid;

  extern vlr::rndf::RoadNetwork        * road_network;
  extern int                             rndf_valid;

  extern std::vector<std::tr1::shared_ptr<dgc::Obstacle> > obstacles_segmented;
  extern std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> >  obstacles_predicted;
  extern std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> >  obstacles_tracked;
  extern std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> >  obstacles_tracked_publish;

  extern perception_settings_t           settings;

  extern laser_scan_p                    lscan;
  extern laser_scan_p                    nscan;

  extern dgc_perception_map_cells_p      obstacles_s;
  extern dgc_perception_map_cell_p     obstacles_s_cell_pointer;
  extern dgc_perception_map_cells_t      obstacles_s_publish;
  extern dgc_perception_map_cells_p      map_s;

  extern std::vector<dgc_perception_map_region_p>   regions;

  extern int                             publish_ready;
  extern int                             dynamic_msg_ready;
  extern unsigned short                  counter;
  extern int                             virtual_scan_counter;

  extern MultiBooster* booster;
  extern ClassifierPipeline* g_classifier_pipeline;

  /************************************************************************
   *
   * FUNCTIONS
   *
   ************************************************************************/
  extern dgc_global_pose_t               global_pos;
  extern char                          * rndf_filename;
  
  void rndf_load_file( char *filename );
  void rndf_stop_zones_init( void );
  void rndf_stop_zones_check( void );

  void perception_init( void );

  void velodyne_init( dgc_velodyne_data_p velodyne );

  void integrate_sensors( dgc_velodyne_data_p velo );
  
  void integrate_velodyne( dgc_velodyne_data_p velodyne, 
			   unsigned short counter );

  void generate_clusters(int counter, int num_scans, dgc_velodyne_scan_p scans,
      dgc_velodyne_config_p config, double ground_threshold, std::vector<laser_cluster_p>& clusters);

  void test_clusters(int num_scans, dgc_velodyne_scan_p scans,
      dgc_velodyne_config_p config, double ground_threshold, std::vector<laser_cluster_p>& clusters,
      double x, double y);

  void integrate_ldlrs( dgc::LdlrsLaser *ldlrs, dgc_transform_t offset,
			unsigned short counter );

  unsigned short counter_diff( unsigned short last, unsigned short now );


  void perception_publish_stop_zones( dgc::IpcInterface *ipc);

  void perception_register_ipc_messages(dgc::IpcInterface *ipc);

  void perception_publish_dynamic( vlr::PerceptionObstacleList *msg );

  void perception_publish_obstacles( dgc::IpcInterface *ipc, dgc_grid_p grid, 
				     dgc_perception_map_cells_p points, std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> > obstacles,
				     unsigned short counter );
  
  void perception_publish_map_diff( dgc::IpcInterface *ipc, dgc_grid_p grid, 
				    dgc_perception_map_cells_p obstacles,
				    dgc_perception_map_cells_p points,
				    unsigned short counter );

  void perception_publish_map( dgc::IpcInterface *ipc, dgc_grid_p grid );

  void perception_publish_map_rle( dgc::IpcInterface *ipc, dgc_grid_p grid );

  void perception_publish_terrain_map( dgc_grid_p grid );

  dgc::ApplanixPose         *  applanix_history_pose( applanix_elem_p elem );
  applanix_elem_p              applanix_history_elem( applanix_history_p history );
  applanix_elem_p              applanix_history_next( applanix_elem_p elem );
  applanix_elem_p              applanix_history_prev( applanix_elem_p elem );
  
  dgc::ApplanixPose         *  applanix_current_pose( void );
  dgc::ApplanixPose         *  applanix_pose( double timestamp );
  void                         applanix_history_init( int size );
  void                         applanix_history_add( dgc::ApplanixPose *msg );

  void  write_map( dgc_grid_p map );
  void  grid_line( ivec2_t start, ivec2_t end, grid_line_p line );

  void  free_space_ray_tracing( unsigned short counter );

  void  set_cell_min( dgc_perception_map_cell_p cell,  float z, unsigned short counter );
  void  set_cell_max( dgc_perception_map_cell_p cell,  float z, unsigned short counter );
  void  sync_with_terrain( dgc_perception_map_cell_p t, dgc_perception_map_cell_p c );

  void * integration_thread( void * ptr __attribute__ ((unused)) );
  void * tracking_thread( void * ptr __attribute__ ((unused)) );

  void   dgc_transform_integrate_pose( dgc_transform_t t, dgc_pose_t pose );

//  void   perception_allocate_virtual_scan( vlr::PerceptionScan *scan,
//					   float resolution );

//  void   perception_compute_virtual_scan( vlr::PerceptionScan *scan,
//					  dgc_pose_t pose,
//					  dgc_transform_t offset,
//					  unsigned short counter );

//  void   perception_extract_dynamic_points( void );

  void   perception_track_frame( double timestamp );

  double pose_dist( dgc::ApplanixPose *p1, 
		    dgc::ApplanixPose *p2 );

  double get_z();

  void display_time(char* label, double time);

  char* obstacle_type2str(dgc_obstacle_type type);

#endif
