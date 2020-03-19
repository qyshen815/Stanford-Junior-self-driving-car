#ifndef DGC_CAN_MESSAGES_H
#define DGC_CAN_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define CAN_CLUTCH_OPEN                   0
#define CAN_CLUTCH_CONTROLLED_SLIP        1
#define CAN_CLUTCH_CLOSED                 2
#define CAN_CLUTCH_ERROR                  3

#define CAN_TARGET_GEAR_PARK_NEUTRAL      0
#define CAN_TARGET_GEAR_1ST               1
#define CAN_TARGET_GEAR_2ND               2
#define CAN_TARGET_GEAR_3RD               3
#define CAN_TARGET_GEAR_4TH               4
#define CAN_TARGET_GEAR_5TH               5
#define CAN_TARGET_GEAR_1M                6
#define CAN_TARGET_GEAR_REVERSE           7
#define CAN_TARGET_GEAR_6TH               8
#define CAN_TARGET_GEAR_7TH               9
#define CAN_TARGET_GEAR_ERROR             15

#define CAN_GEAR_POSITION_INTERMEDIATE    0
#define CAN_GEAR_POSITION_1               1
#define CAN_GEAR_POSITION_2               2
#define CAN_GEAR_POSITION_3               3
#define CAN_GEAR_POSITION_4               4
#define CAN_GEAR_POSITION_D               5
#define CAN_GEAR_POSITION_N               6
#define CAN_GEAR_POSITION_R               7
#define CAN_GEAR_POSITION_P               8
#define CAN_GEAR_POSITION_RSP             9
#define CAN_GEAR_POSITION_Z1              10
#define CAN_GEAR_POSITION_Z2              11
#define CAN_GEAR_POSITION_S               12
#define CAN_GEAR_POSITION_L               13
#define CAN_GEAR_POSITION_TIPTRONIC       14
#define CAN_GEAR_POSITION_ERROR           15

  /** CAN bus telemetry message from car */

typedef struct {
  float throttle_position;          /**< Throttle percentage - 0 to 100 */
  float steering_angle;             /**< Steering wheel angle in degees */
  float steering_rate;              /**< Steering wheel velocity in deg/sec */
  float engine_rpm;                 /**< Engine RPM */
  char parking_brake;
  char target_gear;                 /**< Internal gear */
  char gear_position;               /**< Position of the gear shift lever */
  float wheel_speed_fl;             /**< Front left wheel speed in km/hr */
  float wheel_speed_fr;             /**< Front right wheel speed in km/hr */
  float wheel_speed_rl;             /**< Rear left wheel speed in km/hr */
  float wheel_speed_rr;             /**< Rear right wheel speed in km/hr */
  float wheel_height_fl;            /**< Suspension encoder height in mm */
  float wheel_height_fr;
  float wheel_height_rl;
  float wheel_height_rr;
  float brake_pressure;             /**< Brake pressure in unknown units */
  char esp_status;                  /**< 1 = active, 0 = inactive */
  char abs_status;                  /**< 1 = active, 0 = inactive */
  char throttle_error, rpm_error, wheel_height_fl_error,
    wheel_height_fr_error, wheel_height_rl_error, wheel_height_rr_error;
  char steering_status;             /**< see CAN docu, lots of things... */
  float avg_wheel_revolutions;      /**< 0..65278 rotations/sec */
  int distance_pulses_front_axle;   /**< 0..2047 */
  float yaw_rate;		    /**< 0..100 deg/sec; 200 indicates invalid yaw_rate */ 
  char backing_up_light;            /**< 1 = on; 0 = off; */
  float wheel_impulses_fl;          /**< Front left wheel impulses 0-1000; 1021=initValue; 1022=underVoltage; 1023=overVoltage; 1030=QualificationBitImpulsesInvalid */
  int wheel_direction_fl;	    /**< Front left wheel -1=backwards; 0=QualificationBitDirectionInvalid; 1=forward; */
  float wheel_impulses_fr; 
  int wheel_direction_fr;
  float wheel_impulses_rl;
  int wheel_direction_rl; 
  float wheel_impulses_rr; 
  int wheel_direction_rr;
  int wheel_direction_rr_added;     /**< Wheel direction rear right, added from sensor/uC from VW Germany */
  double steer_angleCalculated;     /**< -614,25..614,25 deg;   1000 indicates invalid angleCalculated   */ 
  double steer_handTorque;          /**< -15Nm..15Nm;   1000 indicates invalid handTorque */
  char   steer_statusEPS;     /**< Status of EPS according Lastenheft PLA/EPS */
  double timestamp;
  char host[10];
} CanStatus;
 
#define      DGC_CAN_STATUS_NAME      "dgc_can_status"
#define      DGC_CAN_STATUS_FMT       "{float,float,float,float,char,char,char,float,float,float,float,float,float,float,float,float,char,char,char,char,char,char,char,char,char,float,int,float,char,float,int,float,int,float,int,float,int,int,double,double,char,double,[char:10]}"

const IpcMessageID CanStatusID = { DGC_CAN_STATUS_NAME, DGC_CAN_STATUS_FMT };

}

#endif
