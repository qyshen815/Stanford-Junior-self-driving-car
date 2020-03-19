#include <roadrunner.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <localize_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>

using namespace dgc;

IpcInterface *ipc = NULL;

int received_applanix_pose = 0;
ApplanixPose applanix_pose;

double x_shift = 0, y_shift = 0;

void dgc_localize_publish_pose_message(double x_offset, double y_offset)
{
  static LocalizePose pose;
  static int first = 1;
  int err;
  
  if(first) {
    strncpy(pose.host, dgc_hostname(), 10);
    first = 0;
  }
  pose.x_offset = x_offset;
  pose.y_offset = y_offset;
  pose.timestamp = dgc_get_time();
  err = ipc->Publish(LocalizePoseID, &pose);
  TestIpcExit(err, "Could not publish", LocalizePoseID);
}

void applanix_pose_handler(void)
{
  received_applanix_pose = 1;
}

void localize_timer(void)
{
  double x, y;
  char utmzone[10];

  if(!received_applanix_pose)
    return;

  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &x, &y, utmzone);
  //  x -= ((int)floor(dgc_get_time()) % 2) * 1;

  dgc_localize_publish_pose_message(x - applanix_pose.smooth_x + x_shift,
				    y - applanix_pose.smooth_y + y_shift);
}

int main(int argc, char **argv)
{
  int err;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if(argc >= 3) {
    x_shift = atof(argv[1]);
    y_shift = atof(argv[2]);
    fprintf(stderr, "shift = %.2f %.2f\n", x_shift, y_shift);
  }

  err = ipc->DefineMessage(LocalizePoseID);
  TestIpcExit(err, "Could not define", LocalizePoseID);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose, &applanix_pose_handler);

  ipc->AddTimer(1.0 / 10.0, localize_timer);

  ipc->Dispatch();
  return 0;
}
