#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <power_interface.h>

using namespace dgc;

IpcInterface *ipc;

void power_down_device(char *device)
{
  int err;

  fprintf(stderr, "POWER DOWN %s ... ", device);
  err = PowerSetNamedCommand(ipc, device, 0);
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

  power_down_device("SICK1");
  power_down_device("SICK2");
  power_down_device("LDLRS1");
  power_down_device("LDLRS2");
  power_down_device("IBEO");
  power_down_device("VELODYNE");
  power_down_device("RADAR");
  return 0;
}
