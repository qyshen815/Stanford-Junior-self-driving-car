#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <pidcontrol_interface.h>

using namespace dgc;

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  PidcontrolGroupSetCommand(ipc, "DRIVE", "*", 1);
  return 0;
}
