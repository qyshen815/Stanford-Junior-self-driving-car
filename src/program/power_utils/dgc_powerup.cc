#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <power_interface.h>

using namespace dgc;

IpcInterface *ipc = NULL;

void power_up_device(char *device)
{
  int err;

  fprintf(stderr, "POWER UP %s ... ", device);
  err = PowerSetNamedCommand(ipc, device, 1);
  if(err < 0)
    fprintf(stderr, "FAILED\n");
  else
    fprintf(stderr, "SUCCESS\n");
}

int main(int /*argc*/, char **argv)
{
  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  power_up_device("SICK1");
  power_up_device("SICK2");
  power_up_device("LDLRS1");
  power_up_device("LDLRS2");
  power_up_device("IBEO");
  power_up_device("VELODYNE");
  power_up_device("RADAR");
  return 0;
}
