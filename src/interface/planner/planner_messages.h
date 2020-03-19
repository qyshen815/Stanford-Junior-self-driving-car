#ifndef DGC_PLANNER_MESSAGES_H
#define DGC_PLANNER_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

  /** planner trajectory waypoint */

typedef struct {
  double time;
  double x;                          /**< waypoint x position, in meters */
  double y;                          /**< waypoint y position, in meters */
  float theta;                       /**< heading of trajectory, in radians */
  float v;                           /**< velocity of vehicle, in m/s    */
  float yaw_rate;                    /**< yaw rate of vehicle, in radians/s */
} PlannerWaypoint;

  /** planner trajectory */

typedef struct {
  int num_waypoints;                 /**< number of waypoints in trajectory */
  PlannerWaypoint *waypoint;   /**< list of waypoints */
  char stop_in_view;
  float stop_dist;
  char obstacle_in_view, in_safety_zone;
  float obstacle_dist, obstacle_vel;
  char reverse;
  int time_indexed;
  double timestamp;
  char host[10];
} PlannerTrajectory;
  
#define    DGC_PLANNER_TRAJECTORY_NAME    "dgc_planner_trajectory"
#define    DGC_PLANNER_TRAJECTORY_FMT     "{int,<{double,double,double,float,float,float}:1>,char,float,char,char,float,float,char,int,double,[char:10]}"

const IpcMessageID PlannerTrajectoryID = { DGC_PLANNER_TRAJECTORY_NAME, 
					   DGC_PLANNER_TRAJECTORY_FMT };

typedef struct {
  int num_waypoints;                 /**< number of waypoints in trajectory */
  PlannerWaypoint *waypoint;   /**< list of waypoints */
  double timestamp;
  char host[10];
} PlannerFulltraj;

#define    DGC_PLANNER_FULLTRAJ_NAME    "dgc_planner_fulltraj"
#define    DGC_PLANNER_FULLTRAJ_FMT     "{int,<{double,double,float,float,float}:1>,double,[char:10]}"

const IpcMessageID PlannerFulltrajID = { DGC_PLANNER_FULLTRAJ_NAME, 
					 DGC_PLANNER_FULLTRAJ_FMT };

#define      DGC_PLANNER_ALLOW_MOVEMENT     0

typedef struct {
  int command;
  double timestamp;
  char host[10];
} PlannerTrigger;

#define    DGC_PLANNER_TRIGGER_NAME    "dgc_planner_trigger"
#define    DGC_PLANNER_TRIGGER_FMT     "{int,double,[char:10]}"

const IpcMessageID PlannerTriggerID = { DGC_PLANNER_TRIGGER_NAME, 
					DGC_PLANNER_TRIGGER_FMT };

typedef struct {
  double timestamp;
  char host[10];
} PlannerFsmRequest;

#define    DGC_PLANNER_FSM_REQUEST_NAME    "dgc_planner_fsm_request"
#define    DGC_PLANNER_FSM_REQUEST_FMT     "{double,[char:10]}"

const IpcMessageID PlannerFsmRequestID = { DGC_PLANNER_FSM_REQUEST_NAME, 
					   DGC_PLANNER_FSM_REQUEST_FMT };

typedef struct {
  char *fsmdata;
  double timestamp;
  char host[10];
} PlannerFsmResponse;

#define    DGC_PLANNER_FSM_NAME       "dgc_planner_fsm"
#define    DGC_PLANNER_FSM_FMT        "{string,double,[char:10]}"

const IpcMessageID PlannerFsmResponseID = { DGC_PLANNER_FSM_NAME, 
					    DGC_PLANNER_FSM_FMT };

typedef struct {
  int state;
  double timestamp;
  char host[10];
} PlannerFsmState;

#define   DGC_PLANNER_FSM_STATE_NAME     "dgc_planner_fsm_state"
#define   DGC_PLANNER_FSM_STATE_FMT      "{int,double,[char:10]}"

const IpcMessageID PlannerFsmStateID = { DGC_PLANNER_FSM_STATE_NAME, 
					 DGC_PLANNER_FSM_STATE_FMT };

typedef struct {
  double goal_lat, goal_lon, goal_theta;
  double timestamp;
  char host[10];
} PlannerGoal;

#define   DGC_PLANNER_GOAL_NAME         "dgc_planner_goal"
#define   DGC_PLANNER_GOAL_FMT          "{double,double,double,double,[char:10]}"

const IpcMessageID PlannerGoalID = { DGC_PLANNER_GOAL_NAME, 
				     DGC_PLANNER_GOAL_FMT };

typedef struct {
  float local_score, global_score, obstacle_dist;
  char illegal, too_fast, lane_change;
} PlannerBtInfo;

typedef struct {
  int fsm_state;
  int mdf_goal_num;

  int num_bts;
  PlannerBtInfo *bt_info;
  int chosen_bt;

  float best_path_static_undercar_sum;
  float best_path_static_nearcar_sum;
  float best_path_undercar_sum;
  float best_path_nearcar_sum;
  float best_path_obstacle_dist;
  int best_path_obstacle_dist_i;

  float int_static_points;
  float int_moving_points;

  double timestamp;
  char host[10];
} PlannerStatus;

#define    DGC_PLANNER_STATUS_NAME     "dgc_planner_status"
#define    DGC_PLANNER_STATUS_FMT      "{int,int,int,<{float,float,float,char,char,char}:3>,int,float,float,float,float,float,int,float,float,double,[char:10]}"

const IpcMessageID PlannerStatusID = { DGC_PLANNER_STATUS_NAME, 
				       DGC_PLANNER_STATUS_FMT };

typedef struct {
  int goal_s, goal_l, goal_w;
  int current_goal_num, num_goals, checkpoint_num;
  double timestamp;
  char host[10];
} PlannerMdfGoal;

#define    DGC_PLANNER_MDF_GOAL_NAME    "dgc_planner_mdf_goal"
#define    DGC_PLANNER_MDF_GOAL_FMT     "{int,int,int,int,int,int,double,[char:10]}"

const IpcMessageID PlannerMdfGoalID = { DGC_PLANNER_MDF_GOAL_NAME, 
					DGC_PLANNER_MDF_GOAL_FMT };

}

#endif
