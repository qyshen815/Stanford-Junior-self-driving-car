#include <roadrunner.h>
#include <trajectory.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <planner_messages.h>
#include "fp.h"

using namespace dgc;

#define      PLANNER_HZ         10.0
#define      R                  50.0
#define      D                  0.2
#define      V                  dgc_mph2ms(10.0)

IpcInterface *ipc = NULL;

/* parameters */

char *trajectory_filename = NULL;
int num_waypoints;

/* global variables */

ApplanixPose applanix_pose;
int received_pose = 0;
double robot_x = 0, robot_y = 0, robot_yaw = 0;
double localize_x_offset = 0, localize_y_offset = 0;

int forwards = 1;
double distance_travelled = 0;
int waiting_for_stop = 0;

double can_velocity;
int received_can = 0;

void can_handler(CanStatus *can)
{
  can_velocity = dgc_kph2ms(0.5 * (can->wheel_speed_rl + can->wheel_speed_rr));
  received_can = 1;
}

void applanix_pose_handler(void)
{
  robot_x = applanix_pose.smooth_x + localize_x_offset;
  robot_y = applanix_pose.smooth_y + localize_y_offset;
  robot_yaw = applanix_pose.yaw;
  received_pose = 1;
}

void localize_pose_handler(LocalizePose *localize_pose)
{
  localize_x_offset = localize_pose->x_offset;
  localize_y_offset = localize_pose->y_offset;
}

dgc_trajectory_p generate_circular_traj(void)
{
  dgc_trajectory_p traj;
  double utm_x, utm_y;
  char utmzone[10];
  int i, n;

  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &utm_x, &utm_y, utmzone);
  
  n = (int)rint(2 * R * M_PI / 0.2);
  traj = dgc_trajectory_init(n, 1);
    
  traj->waypoint[0].x = utm_x;
  traj->waypoint[0].y = utm_y;
  traj->waypoint[0].theta = applanix_pose.yaw;
  traj->waypoint[0].velocity = V;
  traj->waypoint[0].yaw_rate = V / R;
  
  for(i = 1; i < n; i++) {
    traj->waypoint[i].x = traj->waypoint[i - 1].x + 
      D * cos(traj->waypoint[i - 1].theta);
    traj->waypoint[i].y = traj->waypoint[i - 1].y + 
      D * sin(traj->waypoint[i - 1].theta);
    traj->waypoint[i].theta = traj->waypoint[i - 1].theta +
      traj->waypoint[i - 1].yaw_rate * D / V;
    traj->waypoint[i].velocity = V;
    traj->waypoint[i].yaw_rate = V / R;
  }
  return traj;
}

void generate_plan(void)
{
  static dgc_trajectory_p traj = NULL;
  static PlannerTrajectory *plan = NULL;
  static int closest_waypoint;
  double fa_x, fa_y;
  int i, err;

  if(!received_pose)
    return;

  if(traj == NULL) 
    traj = generate_circular_traj();

  fa_x = robot_x + DGC_PASSAT_IMU_TO_FA_DIST * cos(robot_yaw);
  fa_y = robot_y + DGC_PASSAT_IMU_TO_FA_DIST * sin(robot_yaw);

  if(plan == NULL)
    plan = fp_plan_from_pose(traj, fa_x, fa_y, num_waypoints, 
                             &closest_waypoint, 6.0);
  else
    fp_plan_from_plan(plan, traj, fa_x, fa_y, &closest_waypoint, 6.0);

  for(i = 0; i < plan->num_waypoints; i++) {
    plan->waypoint[i].x -= localize_x_offset;
    plan->waypoint[i].y -= localize_y_offset;
  }

  /* decide if we should be going forwards or backwards */
  distance_travelled += fabs(can_velocity / PLANNER_HZ);
  if(distance_travelled > M_PI * R / 2) {
    distance_travelled = 0;
    waiting_for_stop = 1;
  }
  if(waiting_for_stop) {
    if(can_velocity == 0) {
      forwards = !forwards;
      waiting_for_stop = 0;
    }
    else 
      for(i = 0; i < plan->num_waypoints; i++) 
        plan->waypoint[i].v = 0;
  }
  if(!forwards) 
    for(i = 0; i < plan->num_waypoints; i++) {
      plan->waypoint[i].v *= -1;
      plan->waypoint[i].theta += M_PI;
    }

  err = ipc->Publish(PlannerTrajectoryID, plan);
  TestIpcExit(err, "Could not publish", PlannerTrajectoryID);
  fprintf(stderr, "P");
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param param[] = {
    {"fakeplanner", "num_waypoints", DGC_PARAM_INT, &num_waypoints, 0, NULL},
  };
  pint->InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
}

int main(int argc, char **argv)
{
  ParamInterface *pint = NULL;
  int err;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);

  if (ipc->ConnectLocked("planner") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);

  err = ipc->DefineMessage(PlannerTrajectoryID);
  TestIpcExit(err, "Could not define", PlannerTrajectoryID);

  ipc->Subscribe(CanStatusID, &can_handler);
  ipc->Subscribe(ApplanixPoseID, &applanix_pose, &applanix_pose_handler);
  ipc->Subscribe(LocalizePoseID, &localize_pose_handler);
  ipc->AddTimer(1.0 / (float)PLANNER_HZ, generate_plan);
  ipc->Dispatch();
  return 0;
}
