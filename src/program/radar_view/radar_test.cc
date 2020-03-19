#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <radar_interface.h>

using namespace dgc;

RadarSensor radar1;

void radar1_handler(void)
{
  int i;
  fprintf(stderr, "RADAR: ");
  for(i = 0; i < radar1.num_targets; i++)
    fprintf(stderr, "%.2f ", radar1.target[i].distance);
  fprintf(stderr, "\n");
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(RadarSensor1ID, &radar1_handler);
  ipc->Dispatch();
  return 0;
}
