#ifndef DGC_GHOSTCAR_MESSAGES_H
#define DGC_GHOSTCAR_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  int    vehicle_id;
  int    vehicle_type;
  double lat, lon;
  float  altitude;
  float  yaw, pitch, roll;
  float  a_x, a_y, a_z;
  float  v_n, v_e, v_u;
  float  wheel_angle, speed;
  int    gear;
  float  throttle, brake;
  double timestamp;
  char   host[10];
} GhostcarPose;

#define    DGC_GHOSTCAR_POSE_NAME    "dgc_ghostcar_pose"
#define    DGC_GHOSTCAR_POSE_FMT     "{int,int,double,double,float,float,float,float,float,float,float,float,float,float,float,float,int,float,float,double,[char:10]}"

const IpcMessageID GhostcarPoseID = { DGC_GHOSTCAR_POSE_NAME, 
				      DGC_GHOSTCAR_POSE_FMT };

typedef struct {
  int vehicle_id;
  double lat, lon;
  double timestamp;
  char host[10];
} GhostcarSync;

#define    DGC_GHOSTCAR_SYNC_NAME    "dgc_ghostcar_sync"
#define    DGC_GHOSTCAR_SYNC_FMT     "{int,double,double,double,[char:10]}"

const IpcMessageID GhostcarSyncID = { DGC_GHOSTCAR_SYNC_NAME, 
				      DGC_GHOSTCAR_SYNC_FMT };

}

#endif
