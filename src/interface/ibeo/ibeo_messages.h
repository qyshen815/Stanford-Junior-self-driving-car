#ifndef DGC_IBEO_MESSAGES_H
#define DGC_IBEO_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define     DGC_IBEO_STATUS_OK        0
#define     DGC_IBEO_STATUS_INVALID   1
#define     DGC_IBEO_STATUS_RAIN      2
#define     DGC_IBEO_STATUS_GROUND    3
#define     DGC_IBEO_STATUS_DIRT      4

typedef struct {
  char level, status;
  float x, y, z;
} IbeoLaserPoint;

typedef struct {
  float start_angle, end_angle;
  int scan_counter;
  int num_points;
  IbeoLaserPoint *point;
  double hardware_timestamp;
  double timestamp;
  char host[10];
} IbeoLaser;

#define  DGC_IBEO_LASER1_NAME   "dgc_ibeo_laser1"
#define  DGC_IBEO_LASER1_FMT    "{float,float,int,int,<{char,char,float,float,float}:4>,double,double,[char:10]}"

const IpcMessageID IbeoLaser1ID = { DGC_IBEO_LASER1_NAME, 
				    DGC_IBEO_LASER1_FMT };

#define  DGC_IBEO_LASER2_NAME   "dgc_ibeo_laser2"
#define  DGC_IBEO_LASER2_FMT    "{float,float,int,int,<{char,char,float,float,float}:4>,double,double,[char:10]}"

const IpcMessageID IbeoLaser2ID = { DGC_IBEO_LASER2_NAME, 
				    DGC_IBEO_LASER2_FMT };

#define  DGC_IBEO_LASER3_NAME   "dgc_ibeo_laser3"
#define  DGC_IBEO_LASER3_FMT    "{float,float,int,int,<{char,char,float,float,float}:4>,double,double,[char:10]}"

const IpcMessageID IbeoLaser3ID = { DGC_IBEO_LASER3_NAME, 
				    DGC_IBEO_LASER3_FMT };

}

#endif
