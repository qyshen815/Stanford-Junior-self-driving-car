#ifndef DGC_PERCEPTION_MESSAGES_H
#define DGC_PERCEPTION_MESSAGES_H

#include <ipc_interface.h>
#include <global.h>
#include <stdint.h>

namespace vlr {

/*************************************************************************/

typedef struct {
  int segment, lane, waypoint;
  char detected_left_boundary, detected_right_boundary;
  double x, y;
  float width;
  float x_var, y_var, xy_var;
} PerceptionWaypointDiff;

typedef struct {
  int num_changes;
  PerceptionWaypointDiff *change;
  double timestamp;
  char host[10];
} PerceptionRndfDiff;

#define    DGC_PERCEPTION_RNDFDIFF_NAME      "dgc_perception_rndfdiff"
#define    DGC_PERCEPTION_RNDFDIFF_FMT       "{int,<{int,int,int,char,char,double,double,float,float,float,float}:1>,double,[char:10]}"

const dgc::IpcMessageID PerceptionRndfDiffID = { DGC_PERCEPTION_RNDFDIFF_NAME,
					    DGC_PERCEPTION_RNDFDIFF_FMT };

/*************************************************************************
 *
 *    DYNAMIC MESSAGE: obstacles seen by the tracker
 *
 *************************************************************************/

#define     DGC_STATIC_OBSTACLE      1
#define     DGC_DYNAMIC_OBSTACLE     2

typedef struct {
  float                              x;
  float                              y;
} ObstaclePoint;

#define  DGC_DYNAMIC_OBSTACLE_POINT_FMT    "{float,float}"

typedef struct {
  int                                id;
  char                               obstacleType;
  char                               obstacleTypeThisFrame;
  int                                classifiedThisFrame;
  char                               confidence;
  char                               turn_signal;
  double                             x;
  double                             y;
  float                              direction;
  float                              width;
  float                              length;
  float                              velocity;
  float                              x_var;
  float                              y_var;
  float                              xy_cov;
} PerceptionDynamicObstacle;

#define    DGC_DYNAMIC_OBSTACLE_FMT    "{int,char,char,int,char,char,double,double,float,float,float,float,float,float,float }"

typedef struct {
  int                                num_obstacles;
  PerceptionDynamicObstacle         *obstacle;
  double                             timestamp;
  char                               host[10];
} PerceptionObstacleList;

#define    DGC_PERCEPTION_OBSTACLELIST_NAME       "dgc_perception_obstaclelist"
#define    DGC_PERCEPTION_OBSTACLELIST_FMT        "{int,<" DGC_DYNAMIC_OBSTACLE_FMT ":1>,double,[char:10]}"

const dgc::IpcMessageID PerceptionObstacleListID =
  { DGC_PERCEPTION_OBSTACLELIST_NAME,
    DGC_PERCEPTION_OBSTACLELIST_FMT };

/*************************************************************************
 *
 *    OBSTACLE MESSAGE: current visible obstacles
 *
 *************************************************************************/

typedef struct {
  float                              x;
  float                              y;
  float                              z_min;
  float                              z_max;
  char                               type;
} PerceptionObstaclePoint;

#define  DGC_PERCEPTION_OBSTACLE_POINT_FMT    "{float,float,float,float,char}"

/*************************************************************************/

typedef struct {
  int                                num_points;
  PerceptionObstaclePoint           *point;
  unsigned short                     counter;
  int                                num_dynamic_obstacles;
  PerceptionDynamicObstacle         *dynamic_obstacle;
  double                             timestamp;
  char                               host[10];
} PerceptionObstacles;

#define  DGC_PERCEPTION_OBSTACLES_NAME   "dgc_perception_obstacles"
#define  DGC_PERCEPTION_OBSTACLES_FMT    "{int,<" DGC_PERCEPTION_OBSTACLE_POINT_FMT ":1>,short,int,<" DGC_DYNAMIC_OBSTACLE_FMT ":4>,double, [char:10]}"

const dgc::IpcMessageID PerceptionObstaclesID = { DGC_PERCEPTION_OBSTACLES_NAME,
					     DGC_PERCEPTION_OBSTACLES_FMT };

/*************************************************************************
 *
 *    SCAN MESSAGE: message with the combined scan
 *
 *************************************************************************/

typedef struct {
  float                   dist;
  float                   angle;
  float                   x;
  float                   y;
} PerceptionScanPoint;

#define  DGC_PERCEPTION_SCAN_PTS_FMT    "{float,float,float,float}"

#define  DGC_POSE_FMT  "{double,double,double,double,double,double}"

typedef struct {
  double                  x_offset;
  double                  y_offset;
} PerceptionLocalizeOffset;

#define  DGC_PERCEPTION_LOCALIZE_OFFSET_FMT    "{double,double}"


typedef struct {
  dgc_pose_t                         pose;
  PerceptionLocalizeOffset           localize;
} PerceptionRobotPose;

#define  DGC_PERCEPTION_ROBOT_POSE_FMT    "{ " DGC_POSE_FMT "," DGC_PERCEPTION_LOCALIZE_OFFSET_FMT " }"

typedef struct {
  float                       origin_x;
  float                       origin_y;
  float                       resolution;
  int                         num;
  PerceptionScanPoint        *p;
  int                         counter;
  PerceptionRobotPose         robot;
  double                      timestamp;
  char                        host[10];
} PerceptionScan;

#define  DGC_PERCEPTION_SCAN_NAME   "dgc_perception_scan"
#define  DGC_PERCEPTION_SCAN_FMT    "{float,float,float,int, <" DGC_PERCEPTION_SCAN_PTS_FMT ":4>, int, " DGC_PERCEPTION_ROBOT_POSE_FMT ", double, [char:10] }"

const dgc::IpcMessageID PerceptionScanID = { DGC_PERCEPTION_SCAN_NAME,
					DGC_PERCEPTION_SCAN_FMT };


/*************************************************************************
 *
 *    MAP MESSAGES: messages related to the grid map
 *
 *************************************************************************/

typedef struct {
  float                              x;
  float                              y;
  char                               type;
} PerceptionMapGrid;

#define  DGC_PERCEPTION_MAP_GRID_FMT    "{float,float,char}"

typedef struct {
  int                                num_points;
  PerceptionMapGrid                 *grid;
  unsigned short                     counter;
  float                              center_x;
  float                              center_y;
  float                              resolution;
  int                                mapsize_x; /* grid size of map in pixel */
  int                                mapsize_y; /* grid size of map in pixel */
  short                              new_map; /* the map should be cleared, if
						 this value is not zero */

  double                             timestamp;
  char                               host[10];
} PerceptionMapDiff;

#define  DGC_PERCEPTION_MAP_DIFF_NAME   "dgc_perception_map_diff"
#define  DGC_PERCEPTION_MAP_DIFF_FMT    "{int,<" DGC_PERCEPTION_MAP_GRID_FMT ":1>,short,float,float,float,int,int,short,double, [char:10]}"

const dgc::IpcMessageID PerceptionMapDiffID = { DGC_PERCEPTION_MAP_DIFF_NAME,
					   DGC_PERCEPTION_MAP_DIFF_FMT };

/*************************************************************************/

typedef struct {
  double                             timestamp;
  char                               host[10];
} PerceptionMapReset;

#define  DGC_PERCEPTION_MAP_RESET_NAME   "dgc_perception_map_reset"
#define  DGC_PERCEPTION_MAP_RESET_FMT    "{double, [char:10]}"

const dgc::IpcMessageID PerceptionMapResetID = { DGC_PERCEPTION_MAP_RESET_NAME,
					    DGC_PERCEPTION_MAP_RESET_FMT };

/*************************************************************************/

typedef struct {
  double                             timestamp;
  char                               host[10];
} PerceptionMapRequest;

#define  DGC_PERCEPTION_MAP_REQUEST_NAME   "dgc_perception_map_request"
#define  DGC_PERCEPTION_MAP_REQUEST_FMT    "{double, [char:10]}"

const dgc::IpcMessageID PerceptionMapRequestID = { DGC_PERCEPTION_MAP_REQUEST_NAME,
					      DGC_PERCEPTION_MAP_REQUEST_FMT };

/*************************************************************************/

typedef struct {
  double                             timestamp;
  char                               host[10];
} PerceptionTerrainMapRequest;

#define  DGC_PERCEPTION_TERRAIN_MAP_REQUEST_NAME   "dgc_perception_terrain_map_request"
#define  DGC_PERCEPTION_TERRAIN_MAP_REQUEST_FMT    "{double, [char:10]}"

/*************************************************************************
 * full map as PNG data (compressed as PNG)
 *************************************************************************/

typedef struct {
  double                   resolution;     /* map resolution in meters   */
  int                      rows;           /* rows of grid               */
  int                      cols;           /* columns of grid            */
  int                      map_r0;         /* grid coordinates of lower  */
  int                      map_c0;         /*   left corner of map       */
  int                      array_r0;       /* position of lower left     */
  int                      array_c0;       /*   corner in array          */
  int                      non_free;       /* # of non-free cells in map */
  int                      len;
  unsigned char          * data;
  double                   timestamp;
  char                     host[10];
} PerceptionMap;

#define  DGC_PERCEPTION_MAP_NAME   "dgc_perception_map"
#define  DGC_PERCEPTION_MAP_FMT    "{double,int,int,int,int,int,int,int,int,<char:9>, double, [char:10]}"

const dgc::IpcMessageID PerceptionMapID = { DGC_PERCEPTION_MAP_NAME,
				       DGC_PERCEPTION_MAP_FMT };

/*************************************************************************
 * full map, rle compressed
 *************************************************************************/

typedef struct {
  double   resolution;
  int width, height;
  double   fl_corner_x, fl_corner_y; // front left corner from bird's eye perspective
  int compressed_size;
  unsigned char* data;
  double   timestamp;
  char     host[10];
} PerceptionMapRLE;

#define  PERCEPTION_MAP_RLE_NAME   "perception_map_rle"
#define  PERCEPTION_MAP_RLE_FMT    "{double,int,int,double,double,int,<char:6>,double,[char:10]}"

const dgc::IpcMessageID PerceptionMapRLEID = { PERCEPTION_MAP_RLE_NAME, PERCEPTION_MAP_RLE_FMT };

/*************************************************************************
 * full map as PNG data (compressed as PNG)
 *************************************************************************/

typedef struct {
  double                   resolution;     /* map resolution in meters   */
  int                      rows;           /* rows of grid               */
  int                      cols;           /* columns of grid            */
  int                      map_r0;         /* grid coordinates of lower  */
  int                      map_c0;         /*   left corner of map       */
  int                      array_r0;       /* position of lower left     */
  int                      array_c0;       /*   corner in array          */
  int                      unknown;        /* # of unknown cells in map  */
  int                      len;
  unsigned char          * data;
  float                    offset;         /* z offset */
  double                   timestamp;
  char                     host[10];
} PerceptionTerrainMap;

#define  DGC_PERCEPTION_TERRAIN_MAP_NAME   "dgc_perception_terrain_map"
#define  DGC_PERCEPTION_TERRAIN_MAP_FMT    "{double,int,int,int,int,int,int,int,int,<char:9>, float, double, [char:10]}"

const dgc::IpcMessageID PerceptionTerrainMapID = { DGC_PERCEPTION_TERRAIN_MAP_NAME,
					      DGC_PERCEPTION_TERRAIN_MAP_FMT };

/*************************************************************************
 *
 *  STOP ZONE MESSAGES
 *
 *************************************************************************
 this message gives information about the state of the stop zones close
 (100m) to the car. The states are:  unknown, occupied, and free          */

#define ZONE_STATE_UNKNOWN          0
#define ZONE_STATE_OCCUPIED         1
#define ZONE_STATE_FREE             2
#define ZONE_STATE_OCCLUDED         3
#define ZONE_STATE_OBSTACLE_MOVING  4
#define ZONE_STATE_OBSTACLE_STOPPED 5

typedef struct {
  int                           state;
  int                           segment;
  int                           lane;
  int                           waypoint;
  float                         heading;
  double                        utm_x;
  double                        utm_y;
  float                         width;
  float                         length;
  int                           hits;

  //new elements for moving obstacles
  int                           obstacle_id;
  double                        obstacle_vx;
  double                        obstacle_vy;
  double                        obstacle_dist;
} PerceptionStopZone;

#define  DGC_PERCEPTION_STOP_ZONE_FMT  "{int,int,int,int,float,double,double,float,float,int,int,double,double,double}"

typedef struct {
  int                           num_zones;
  PerceptionStopZone           *zone;
  double                        timestamp;
  char                          host[10];
} PerceptionStopZones;

#define  DGC_PERCEPTION_STOP_ZONES_NAME   "dgc_perception_stop_zones"
#define  DGC_PERCEPTION_STOP_ZONES_FMT    "{int,<" DGC_PERCEPTION_STOP_ZONE_FMT ":1>,double, [char:10]}"

const dgc::IpcMessageID PerceptionStopZonesID = { DGC_PERCEPTION_STOP_ZONES_NAME,
					     DGC_PERCEPTION_STOP_ZONES_FMT };

} // namespace vlr

#endif
