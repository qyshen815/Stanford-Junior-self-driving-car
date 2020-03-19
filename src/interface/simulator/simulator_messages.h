#ifndef DGC_SIMULATOR_MESSAGES_H
#define DGC_SIMULATOR_MESSAGES_H

namespace dgc {

typedef struct {
  double x, y, theta, alpha;
  float v, forward_accel, lateral_accel;
  char plan_warning, collision_warning;
  char forward_accel_warning, lateral_accel_warning;
} SimulatorVehiclePose;

typedef struct {
  int num_vehicles;
  SimulatorVehiclePose *vehicle;
  int our_vehicle_num;
  double timestamp;
  char host[10];
} SimulatorGroundTruth;

#define  DGC_SIMULATOR_GROUNDTRUTH_NAME   "dgc_simulator_groundtruth"
#define  DGC_SIMULATOR_GROUNDTRUTH_FMT    "{int,<{double,double,double,double,float,float,float,char,char,char,char}:1>,int,double,[char:10]}"

const IpcMessageID SimulatorGroundTruthID = { DGC_SIMULATOR_GROUNDTRUTH_NAME, 
					      DGC_SIMULATOR_GROUNDTRUTH_FMT };

typedef struct {
  double timestamp;
  char host[10];
} SimulatorStart;

#define    DGC_SIMULATOR_START_NAME    "dgc_simulator_start"
#define    DGC_SIMULATOR_START_FMT     "{double,[char:10]}"

const IpcMessageID SimulatorStartID = { DGC_SIMULATOR_START_NAME, 
					DGC_SIMULATOR_START_FMT };

typedef char dgc_car_name[20];

typedef struct {
  int num_vehicles;
  dgc_car_name *tag;
  double timestamp;
  char host[10];
} SimulatorTag;

#define    DGC_SIMULATOR_TAG_NAME    "dgc_simulator_tag"
#define    DGC_SIMULATOR_TAG_FMT     "{int,<[char:20]:1>,double,[char:10]}"

const IpcMessageID SimulatorTagID = { DGC_SIMULATOR_TAG_NAME, 
				      DGC_SIMULATOR_TAG_FMT };
}

#endif
