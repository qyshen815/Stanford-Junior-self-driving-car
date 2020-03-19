#include <roadrunner.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>

using namespace dgc;

IpcInterface *ipc = NULL;

ApplanixPose pose;

void dgc_applanix_publish_pose_message()
{
  int err;
  
  err = ipc->Publish(ApplanixPoseID, &pose);
  TestIpcExit(err, "Could not publish", ApplanixPoseID);
}

int main(int argc, char **argv)
{
  int err;
  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  switch(argc) {
    case 1:
      pose.latitude = 37.430114;
      pose.longitude = -122.18404;
      pose.altitude = 3.2;
      pose.yaw = 1.54;
      break;
    case 3:
      pose.latitude = atof(argv[1]);
      pose.longitude = atof(argv[2]);
      pose.altitude = 0;
      pose.yaw = 0;
      break;
    case 5:
      pose.latitude = atof(argv[1]);
      pose.longitude = atof(argv[2]);
      pose.altitude = atof(argv[3]);
      pose.yaw = atof(argv[4]);
      break;
    default:
      printf("Usage:\nfake_applanix [lat, lon [, alt, heading]]\n");
      exit(0);
  }
  
  strncpy(pose.host, dgc_hostname(), 10);
  pose.timestamp = dgc_get_time();
  pose.smooth_x = 0;
  pose.smooth_y = 0;
  pose.smooth_z = 0;
  pose.v_north = 0;
  pose.v_east = 0;
  pose.v_up = 0;
  pose.speed = 0;
  pose.track = 0;
  pose.roll = 0;  
  pose.pitch = 0;
  pose.ar_roll = 0;
  pose.ar_pitch = 0;
  pose.ar_yaw = 0;
  pose.a_x = 0;
  pose.a_y = 0;
  pose.a_z = 0;
  pose.wander = 0;
  pose.ID = 100;
  pose.postprocess_code = 0;
  pose.hardware_timestamp = 1.0;
  pose.hardware_time_mode = 2;

  err = ipc->DefineMessage(ApplanixPoseID);
  TestIpcExit(err, "Could not define", ApplanixPoseID);

  ipc->AddTimer(1.0 / 200.0, dgc_applanix_publish_pose_message);

  ipc->Dispatch();
  return 0;
}
