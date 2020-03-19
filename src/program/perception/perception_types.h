#ifndef DGC_PERCEPTION_TYPES_H
#define DGC_PERCEPTION_TYPES_H

#include <applanix_interface.h>
#include <perception_interface.h>
#include <velocore.h>
#include "applanix_history.h"
#include "perception_defines.h"
#include "obstacle_types.h"

typedef struct {
  int    len;
  int  * val;
} stop_zone_history_t;

typedef struct {
  double                                x;
  double                                y;
} point2d_t;

typedef struct {
  double                                x;
  double                                y;
  double                                z;
  double                                intensity;
} point3d_t;

typedef struct {
  int                                   x;
  int                                   y;
} ipoint2d_t;

typedef struct {
  point2d_t                             center;
  ipoint2d_t                            mapsize;
  float                                 resolution;
  float                                 z_resolution;
} grid_stat_t;

typedef struct {
  double                                min_x;
  double                                max_x;
  double                                min_y;
  double                                max_y;
} bounding_box2d_t;

typedef struct {
  float             x1;
  float             x2;
  float             y1;
  float             y2;
  float             min_angle;
  float             max_angle;
} restricted_2d_area_t;

typedef struct {
  int                    num_scans;
  dgc_velodyne_scan_p    scans;
  int                    allocated_scans;
  dgc_velodyne_config_p  config;
  int                    preprocessed;
} dgc_velodyne_data_t, *dgc_velodyne_data_p;


typedef struct {
  unsigned short          hits;
  unsigned short          seen;

  unsigned char           obstacle;
  float                   min;
  float                   max;
  unsigned char           street;

  /* region in segmentation */
  unsigned short          region;

  unsigned short          last_obstacle; // counter when cell was last observed as an obstacle
  unsigned short          last_observed; // counter when cell was last observed
  unsigned short          last_dynamic;  // counter when cell was last observed as dynamic

  /* counter compare modified for the map diff list */
  unsigned short          last_mod;

  unsigned short          last_min;
  unsigned short          last_max;

  /* timestamp_sum / num_timestamps = average timestamp for this cell. */
  double                  num_timestamps;
  double                  timestamp_sum;

} dgc_perception_map_cell_t, *dgc_perception_map_cell_p;

typedef struct {

  float                   min;
  float                   max;
  unsigned short          last_min;
  unsigned short          last_max;

} terrain_map_cell_t, *terrain_map_cell_p;

// a grid region
typedef struct {
  int id;
  int num_points;
  std::vector<dgc_perception_map_cell_p> cells;
} dgc_perception_map_region_t, *dgc_perception_map_region_p;

typedef struct {
  double                 x;
  double                 y;
  double                 z;
  double                 yaw;
  double                 roll;
  double                 pitch;
  char                   utmzone[5];
} dgc_global_pose_t;

typedef struct {
  int                            num;
  dgc_perception_map_cell_p    * cell;
  double                         timestamp;
} dgc_perception_map_cells_t, *dgc_perception_map_cells_p;

typedef struct {
  dgc_velodyne_scan_p        scan;
  dgc_velodyne_point_p       point;
  short*                     z_cell;
  int                        encoder;
  unsigned char              obstacle;
  unsigned char              valid;
} laser_point_t, *laser_point_p;

typedef struct {
  int                      num_points;
  laser_point_t          * laser_point;
  dgc_pose_t             * robot;
} laser_scan_t, *laser_scan_p;

typedef std::vector<dgc_velodyne_point_p> laser_cluster_t;
typedef laser_cluster_t* laser_cluster_p;

typedef struct {
  int         x;
  int         y;
} ivec2_t, *ivec2_p;

typedef struct {
  int         max;
  int         numgrids;
  ivec2_p     grid;
} grid_line_t, *grid_line_p;

typedef enum {
  SENSOR_RADAR1,
  SENSOR_RADAR2,
  SENSOR_RADAR3,
  SENSOR_RADAR4,
  SENSOR_RADAR5,
  SENSOR_RADAR6,
  SENSOR_VELODYNE
} dgc_sensor_type;

typedef enum {
  TRACKING_STATE_INITIALIZED,
  TRACKING_STATE_TRACKING,
  TRACKING_STATE_LOST,
  TRACKING_STATE_SEGMENTED,
  TRACKING_STATE_PREDICTED,
  TRACKING_STATE_FILTERED
} dgc_tracking_state;

typedef struct {
  int kernel_size;                    // kernel size for segmentation
  int min_points;                     // min number of points
  int max_points;                     // max number of points
  double min_height;                  // minimum height
  int gls_output;                     // send visualization to perception_view
} segmentation_settings_t, *segmentation_settings_p;

typedef struct {
  /*REMOVE: double default_loc_stddev;
  double default_vel_stddev;
  double transition_stddev;
  double velodyne_stddev;
  double velodyne_max_range;
  double radar_stddev;
  double radar_max_range;
  double max_dist_correspondence;
  double confidence_increment_obs;
  double confidence_increment_unobs;
  double confidence_decay;
  double confidence_max;
  double confidence_min;
  double confidence_initial_max;
  double confidence_initial_min;
  double confidence_track_min;
  double confidence_publish_min;
  double min_car_width;
  double min_car_length;*/
  double correspondence_threshold;
  double pruning_threshold;
  double measurement_variance;
  double position_variance;
  double velocity_variance;
  double initial_position_variance;
  double initial_velocity_variance;
} tracker_kf_settings_t, *tracker_kf_settings_p;

typedef struct {
  double merge_dist;
  double lateral_merge_dist;
  double filter_rndf_max_distance;
  double filter_rndf_max_pedestrian_distance;
  int filter_rndf;
//  double filter_graph_scaler;
//  int    predictive_clustering;
//  double max_car_width;
//  double max_car_length;
  char* classifier_filename;
} tracker_settings_t, *tracker_settings_p;

typedef struct {
  int         gls_output;
  int         show_virtual_scan;
  double      virtual_scan_resolution;

  int         show_ray_tracing;
  int         extract_dynamic;

  int         clear_sensor_data;
  double      max_sensor_delay;

  double      map_resolution;
  double      map_size_x;
  double      map_size_y;
  double      map_cell_threshold;
  int         map_cell_min_hits;
  int         map_cell_increase;
  int         map_ray_tracing;

  double      z_resolution;
  double      z_obstacle_height;

  int         use_ldlrs[NUM_LDLRS_LASERS];
  double      ldlrs_min_dist;
  double      ldlrs_max_dist;

  int         use_velodyne;
  double      velodyne_threshold_factor;
  double      velodyne_max_range;
  double      velodyne_min_beam_diff;
  int         velodyne_sync;
  char       *velodyne_cal;

  double      overpass_height;

  double      rate_in_hz;
  double      publish_interval;

  int         num_threads;

  int         rerun;

  tracker_settings_t        tracker_settings;
  tracker_kf_settings_t     kf_settings;
  segmentation_settings_t   segmentation_settings;
} perception_settings_t, *perception_settings_p;

#endif
