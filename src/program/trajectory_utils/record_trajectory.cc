#include <roadrunner.h>
#include <trajectory.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include <passat_constants.h>

using namespace dgc;

CanStatus can;
int received_can = 0;

dgc_trajectory_t trajectory;
int max_waypoints;

char trajectory_filename[200];

void can_status_handler(void)
{
  received_can = 1;
}

void pose_handler(ApplanixPose *pose)
{
  double x, y;
  char utmzone[5];

  if(!received_can)
    return;

  if(trajectory.num_waypoints == max_waypoints) {
    max_waypoints += 100000;
    trajectory.waypoint = 
      (dgc_trajectory_waypoint_p)realloc(trajectory.waypoint, max_waypoints *
                                         sizeof(dgc_trajectory_waypoint_t));
  }

  vlr::latLongToUtm(pose->latitude, pose->longitude, &x, &y, utmzone);
  trajectory.waypoint[trajectory.num_waypoints].x = x;
  trajectory.waypoint[trajectory.num_waypoints].y = y;
  trajectory.waypoint[trajectory.num_waypoints].theta = pose->yaw +
    dgc_d2r(can.steering_angle) / DGC_PASSAT_STEERING_RATIO;
  trajectory.waypoint[trajectory.num_waypoints].velocity = 
    0.5 * dgc_kph2ms(can.wheel_speed_rl + can.wheel_speed_rr);
  trajectory.waypoint[trajectory.num_waypoints].yaw_rate = pose->ar_yaw;
  trajectory.num_waypoints++;
}

void shutdown_recorder(int x)
{
  if(x == SIGINT) {
    dgc_trajectory_write(&trajectory, trajectory_filename);
    exit(0);
  }
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectory-filename\n", argv[0]);
  
  if(dgc_file_exists(argv[1]))
    dgc_die("Error: file %s already exists.\n", argv[1]);

  strcpy(trajectory_filename, argv[1]);

  trajectory.num_waypoints = 0;
  max_waypoints = 100000;
  trajectory.waypoint = 
    (dgc_trajectory_waypoint_p)calloc(max_waypoints, 
                                      sizeof(dgc_trajectory_waypoint_t));
  trajectory.looping = 0;
  dgc_test_alloc(trajectory.waypoint);

  signal(SIGINT, shutdown_recorder);
  
  ipc->Subscribe(ApplanixPoseID, &pose_handler, DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(CanStatusID, &can_status_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
