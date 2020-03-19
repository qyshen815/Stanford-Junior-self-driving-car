#ifndef DGC_LDLRS_MESSAGES_H
#define DGC_LDLRS_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  int scan_count;
  float angular_resolution;
  float start_angle, end_angle;
  int num_range;
  float *range;
  int num_intensity;
  short int *intensity;
  int sector_start_ts, sector_end_ts;
  double timestamp;
  char host[10];
} LdlrsLaser;

#define  DGC_LDLRS_LASER1_NAME   "dgc_ldlrs_laser1"
#define  DGC_LDLRS_LASER1_FMT    "{int,float,float,float,int,<float:5>,int,<short:7>,int,int,double,[char:10]}"

const IpcMessageID LdlrsLaser1ID = { DGC_LDLRS_LASER1_NAME, 
				     DGC_LDLRS_LASER1_FMT };

#define  DGC_LDLRS_LASER2_NAME   "dgc_ldlrs_laser2"
#define  DGC_LDLRS_LASER2_FMT    "{int,float,float,float,int,<float:5>,int,<short:7>,int,int,double,[char:10]}"

const IpcMessageID LdlrsLaser2ID = { DGC_LDLRS_LASER2_NAME, 
				     DGC_LDLRS_LASER2_FMT };

}

#endif
