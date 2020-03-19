#ifndef DGC_RADAR_MESSAGES_H
#define DGC_RADAR_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define LRR2_OPENING_HALF_ANGLE 	8.0	/* in deg */
#define LRR3_OPENING_HALF_ANGLE 	15.0	/* in deg */
#define LRR2_MAX_DIST			200.0	/* in m */
#define LRR3_MAX_DIST			200.0   /* in m */

typedef struct {
  int id;
  char measured, historical;
  float distance, lateral_offset, lateral_offset_var;
  float relative_acceleration, relative_velocity;
} RadarTarget;

typedef struct {
  int measurement_number;
  int num_targets;
  RadarTarget *target;
  char sensor_dirty, hw_failure, sgu_failure, sgu_comsurveillance;
  char cu_io1_received, cu_io2_received, cu_request_received;
  int scu_temperature;
  double timestamp;
  char host[10];
} RadarSensor;

typedef struct {
  int id;
  char measured, historical, valid, moving_state;
  float long_distance, long_relative_velocity, long_relative_acceleration;
  float long_distance_std, long_velocity_std, long_acceleration_std;
  float lateral_distance, lateral_relative_velocity, lateral_distance_std;
  float prob_exist, prob_obstacle;
} RadarLRR3Target;

#define DGC_LRR3_RADAR_TARGET_FMT       "{int,char,char,char,char,float,float,float,float,float,float,float,float,float,float,float}"

typedef struct {
  int measurement_number;
  int num_targets;
  RadarLRR3Target *target;

  float vehicle_yaw_rate, vehicle_velocity;
  float vehicle_acceleration, vehicle_slip_angle;
  float curvature;

  char acc_target_id, acc_stationary_id;
  float acc_distance, acc_course_offset;
  float acc_relative_acceleration, acc_relative_velocity;

  char pss_moving_id, pss_stationary_id;
  float pss_moving_distance, pss_moving_lateral_offset;
  float pss_stationary_distance, pss_stationary_lateral_offset;

  char sensor_dirty, hw_failure, sgu_failure;
  int scu_temperature;
  float horz_missalign_angle;

  char cu_request, cu_handle;

  double timestamp;
  char host[10];
} RadarLRR3Sensor;

#define     DGC_RADAR_SENSOR1_NAME     "dgc_radar_sensor1"
#define     DGC_RADAR_SENSOR1_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor1ID = { DGC_RADAR_SENSOR1_NAME, 
				      DGC_RADAR_SENSOR1_FMT };

#define     DGC_RADAR_SENSOR2_NAME     "dgc_radar_sensor2"
#define     DGC_RADAR_SENSOR2_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor2ID = { DGC_RADAR_SENSOR2_NAME, 
				      DGC_RADAR_SENSOR2_FMT };

#define     DGC_RADAR_SENSOR3_NAME     "dgc_radar_sensor3"
#define     DGC_RADAR_SENSOR3_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor3ID = { DGC_RADAR_SENSOR3_NAME, 
				      DGC_RADAR_SENSOR3_FMT };

#define     DGC_RADAR_SENSOR4_NAME     "dgc_radar_sensor4"
#define     DGC_RADAR_SENSOR4_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor4ID = { DGC_RADAR_SENSOR4_NAME, 
				      DGC_RADAR_SENSOR4_FMT };

#define     DGC_RADAR_SENSOR5_NAME     "dgc_radar_sensor5"
#define     DGC_RADAR_SENSOR5_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor5ID = { DGC_RADAR_SENSOR5_NAME, 
				      DGC_RADAR_SENSOR5_FMT };

#define     DGC_RADAR_SENSOR6_NAME     "dgc_radar_sensor6"
#define     DGC_RADAR_SENSOR6_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor6ID = { DGC_RADAR_SENSOR6_NAME, 
				      DGC_RADAR_SENSOR6_FMT };

#define     DGC_RADAR_SENSOR7_NAME     "dgc_radar_sensor7"
#define     DGC_RADAR_SENSOR7_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor7ID = { DGC_RADAR_SENSOR7_NAME, 
				      DGC_RADAR_SENSOR7_FMT };

#define     DGC_RADAR_SENSOR8_NAME     "dgc_radar_sensor8"
#define     DGC_RADAR_SENSOR8_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor8ID = { DGC_RADAR_SENSOR8_NAME, 
				      DGC_RADAR_SENSOR8_FMT };

#define     DGC_RADAR_SENSOR9_NAME     "dgc_radar_sensor9"
#define     DGC_RADAR_SENSOR9_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor9ID = { DGC_RADAR_SENSOR9_NAME, 
				      DGC_RADAR_SENSOR9_FMT };

#define     DGC_RADAR_SENSOR10_NAME     "dgc_radar_sensor10"
#define     DGC_RADAR_SENSOR10_FMT      "{int,int,<{int,char,char,float,float,float,float,float}:2>,char,char,char,char,char,char,char,int,double,[char:10]}"

const IpcMessageID RadarSensor10ID = { DGC_RADAR_SENSOR10_NAME, 
				      DGC_RADAR_SENSOR10_FMT };

#define     DGC_LRR3_RADAR_SENSOR1_NAME     "dgc_LRR3_radar_sensor1"
#define     DGC_LRR3_RADAR_SENSOR1_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor1ID = { DGC_LRR3_RADAR_SENSOR1_NAME,
                                          DGC_LRR3_RADAR_SENSOR1_FMT };

#define     DGC_LRR3_RADAR_SENSOR2_NAME     "dgc_LRR3_radar_sensor2"
#define     DGC_LRR3_RADAR_SENSOR2_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor2ID = { DGC_LRR3_RADAR_SENSOR2_NAME,
                                          DGC_LRR3_RADAR_SENSOR2_FMT };

#define     DGC_LRR3_RADAR_SENSOR3_NAME     "dgc_LRR3_radar_sensor3"
#define     DGC_LRR3_RADAR_SENSOR3_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor3ID = { DGC_LRR3_RADAR_SENSOR3_NAME,
                                          DGC_LRR3_RADAR_SENSOR3_FMT };

#define     DGC_LRR3_RADAR_SENSOR4_NAME     "dgc_LRR3_radar_sensor4"
#define     DGC_LRR3_RADAR_SENSOR4_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor4ID = { DGC_LRR3_RADAR_SENSOR4_NAME,
                                          DGC_LRR3_RADAR_SENSOR4_FMT };

#define     DGC_LRR3_RADAR_SENSOR5_NAME     "dgc_LRR3_radar_sensor5"
#define     DGC_LRR3_RADAR_SENSOR5_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor5ID = { DGC_LRR3_RADAR_SENSOR5_NAME,
                                          DGC_LRR3_RADAR_SENSOR5_FMT };

#define     DGC_LRR3_RADAR_SENSOR6_NAME     "dgc_LRR3_radar_sensor6"
#define     DGC_LRR3_RADAR_SENSOR6_FMT      "{int,int,<"DGC_LRR3_RADAR_TARGET_FMT":2>,float,float,float,float,float,char,char,float,float,float,float,char,char,float,float,float,float,char,char,char,int,float,char,char,double,[char:10]}"

const IpcMessageID RadarLRR3Sensor6ID = { DGC_LRR3_RADAR_SENSOR6_NAME,
                                          DGC_LRR3_RADAR_SENSOR6_FMT };

}

#endif
