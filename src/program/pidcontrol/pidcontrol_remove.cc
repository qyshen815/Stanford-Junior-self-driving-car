#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc;

  if(argc != 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s modulename groupname\n", argv[0]);
  
  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  PidcontrolRemoveProcessCommand(ipc, argv[1], argv[2], "*");
  return 0;
}
