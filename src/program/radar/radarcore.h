#ifndef DGC_RADARCORE_H
#define DGC_RADARCORE_H

#include <roadrunner.h>
#include "kvasercan.h"
#include "can.h"

namespace dgc {

#define     BOSCH_LRR3_RADAR_OBJECT_STARTER_CAN_ID    	0x3F2
#define     BOSCH_LRR3_RADAR_FIRST_A_TARGET_CAN_ID     	0x3F3
#define     BOSCH_LRR3_RADAR_FIRST_B_TARGET_CAN_ID     	0x3F4
#define     BOSCH_LRR3_RADAR_LAST_A_TARGET_CAN_ID      	0x431
#define     BOSCH_LRR3_RADAR_LAST_B_TARGET_CAN_ID      	0x432
#define     BOSCH_LRR3_RADAR_OBJECT_ENDER_CAN_ID       	0x433
#define     BOSCH_LRR3_RADAR_ACC_TARGET_CAN_ID       	0x3C0
#define     BOSCH_LRR3_RADAR_PSS_COLLISION_CAN_ID      	0x3C1
#define     BOSCH_LRR3_RADAR_COLLISION_CAN_ID       	0x3C2
#define     BOSCH_LRR3_RADAR_ACC_SGU_INFO_CAN_ID       	0x0B9
#define     BOSCH_LRR3_RADAR_VELOCITY_CAN_ID            0x0CE
#define     BOSCH_LRR3_RADAR_YAWRATE_CAN_ID             0x2B2
#define     BOSCH_LRR3_RADAR_DRIVING_STATUS_CAN_ID      0x1A0

#define     MAX_TARGETS                         	32
#define     DEBUG					0

extern      int 	use_can;

typedef struct {
  int id, measured, historical, valid;
  double long_distance, long_velocity, long_acceleration;
  double long_distance_std, long_velocity_std, long_acceleration_std;
  double lateral_distance, lateral_velocity, lateral_distance_std;
  double prob_exist, prob_obstacle;
  int moving_state;

} dgc_bosch_lrr3_radar_target_t, *dgc_bosch_lrr3_radar_target_p;

typedef struct {
  KvaserCan *can_device;

  int targets_ready, num_objects, measurement_number;

  /* Object Starter Data */
  double vehicle_yaw_rate, vehicle_velocity;
  double vehicle_acceleration, vehicle_slip_angle;

  /* Radar Object Data */
  dgc_bosch_lrr3_radar_target_t target[MAX_TARGETS];

  /* Object Ender Data */
  double sync_time, curvature;
  
  /* ACC Target Object data*/
  int acc_target_id, acc_stationary_id;
  double acc_distance, acc_course_offset;
  double acc_acceleration, acc_velocity;

  /* PSS Collision Object data */
  int pss_moving_id, pss_stationary_id;
  double pss_moving_distance, pss_moving_lateral_offset;
  double pss_stationary_distance, pss_stationary_lateral_offset;
  
  /* SGU Information */
  int sensor_dirty, hw_failure, sgu_failure, scu_temperature;
  double cycle_time, horz_missalign_angle;

  /* Collision Unavoidable Message */
  int cu_request, cu_handle;

} dgc_bosch_lrr3_radar_t, *dgc_bosch_lrr3_radar_p;

typedef struct{
  double vWheelFL, vWheelFR, vWheelRL, vWheelRR;  	// in km/m, as per LRR3 spec
  double yaw_rate;					// in deg/s, as per LRR3 spec
} dgc_bosch_lrr3_pose_t, *dgc_bosch_lrr3_pose_p;

dgc_bosch_lrr3_radar_p 
dgc_bosch_radar_connect(int radar_num);

void
dgc_bosch_radar_send_motion_data(dgc_bosch_lrr3_radar_p radar,
                                 int received_vehicle_state,
				 dgc_bosch_lrr3_pose_p pose_to_radar);
void
dgc_bosch_radar_process(dgc_bosch_lrr3_radar_p radar);

void
dgc_bosch_radar_disconnect(dgc_bosch_lrr3_radar_p *radar);

}

#endif
