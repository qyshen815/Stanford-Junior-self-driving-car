#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>

using namespace dgc;

double distance_travelled = 0;
int count = 0;

void applanix_handler(ApplanixPose *applanix_pose)
{
  static int first = 1;
  static double last_smooth_x = 0, last_smooth_y = 0;
  double dx, dy;

  if(!first) {
    dx = applanix_pose->smooth_x - last_smooth_x;
    dy = applanix_pose->smooth_y - last_smooth_y;
    distance_travelled += hypot(dx, dy);
  }

  last_smooth_x = applanix_pose->smooth_x;
  last_smooth_y = applanix_pose->smooth_y;
  first = 0;

  count++;
  if(count == 100) {
    fprintf(stderr, "\rDistance travelled = %.2f miles (%.1f m)    ", 
	    dgc_meters2miles(distance_travelled), distance_travelled);
    count = 0;
  }
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc = NULL;
  
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
