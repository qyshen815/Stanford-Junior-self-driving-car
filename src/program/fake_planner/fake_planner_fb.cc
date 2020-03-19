#include <roadrunner.h>
#include <trajectory.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <can_interface.h>
#include <planner_messages.h>
#include "fp.h"

using namespace dgc;

#define      PLANNER_HZ              10.0

  IpcInterface *ipc = NULL;

/* parameters */

char *trajectory_filename = NULL;
int num_waypoints = 200;

/* global variables */

int received_pose = 0;
double robot_x = 0, robot_y = 0, robot_yaw = 0;
double localize_x_offset = 0, localize_y_offset = 0;

dgc_trajectory_p traj = NULL;

int in_reverse_gear = 0;

void applanix_pose_handler(ApplanixPose *applanix_pose)
{
  robot_x = applanix_pose->smooth_x + localize_x_offset;
  robot_y = applanix_pose->smooth_y + localize_y_offset;
  robot_yaw = applanix_pose->yaw;
  received_pose = 1;
}

void localize_pose_handler(LocalizePose *localize_pose)
{
  localize_x_offset = localize_pose->x_offset;
  localize_y_offset = localize_pose->y_offset;
}

void can_status_handler(CanStatus *can)
{
  in_reverse_gear = (can->gear_position == CAN_GEAR_POSITION_R);
}

void generate_plan(void)
{
  static PlannerTrajectory *plan = NULL;
  static int closest_waypoint;
  double fa_x, fa_y;
  int i, err;

  if(!received_pose)
    return;

  fa_x = robot_x + DGC_PASSAT_IMU_TO_FA_DIST * cos(robot_yaw);
  fa_y = robot_y + DGC_PASSAT_IMU_TO_FA_DIST * sin(robot_yaw);

  if(plan == NULL)
    plan = fp_plan_from_pose(traj, fa_x, fa_y, num_waypoints, 
                             &closest_waypoint, 10.0);
  else
    fp_plan_from_plan(plan, traj, fa_x, fa_y, &closest_waypoint, 10.0);

  for(i = 0; i < plan->num_waypoints; i++) {
    plan->waypoint[i].x -= localize_x_offset;
    plan->waypoint[i].y -= localize_y_offset;
    plan->waypoint[i].v = dgc_mph2ms(5.0);
  }

  if(in_reverse_gear)
    for(i = 0; i < plan->num_waypoints; i++) {
      plan->waypoint[i].v *= -1;
      plan->waypoint[i].theta += M_PI;
    }

  err = ipc->Publish(PlannerTrajectoryID, plan);
  TestIpcExit(err, "Could not publish", PlannerTrajectoryID);
  if(in_reverse_gear)
    fprintf(stderr, "R");
  else
    fprintf(stderr, "P");
}

int main(int argc, char **argv)
{
  int err;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  if (ipc->ConnectLocked("planner") < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectory-file\n", argv[0]);
  trajectory_filename = argv[1];

  /* read the reference trajectory */
  traj = dgc_trajectory_read(trajectory_filename);

  err = ipc->DefineMessage(PlannerTrajectoryID);
  TestIpcExit(err, "Could not define", PlannerTrajectoryID);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose_handler);
  ipc->Subscribe(LocalizePoseID, &localize_pose_handler);
  ipc->Subscribe(CanStatusID, &can_status_handler);
  ipc->AddTimer(1.0 / (float)PLANNER_HZ, generate_plan);
  ipc->Dispatch();
  return 0;
}
