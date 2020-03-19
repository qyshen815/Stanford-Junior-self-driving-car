#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  int requested_state = 0;
  IpcInterface *ipc;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s groupname requested-state\n", argv[0]);
  
  if(strcmp(argv[2], "0") == 0 ||
     strcmp(argv[2], "OFF") == 0 ||
     strcmp(argv[2], "DOWN") == 0)
    requested_state = 0;
  else if(strcmp(argv[2], "1") == 0 ||
          strcmp(argv[2], "ON") == 0 ||
          strcmp(argv[2], "UP") == 0)
    requested_state = 1;
  else
    dgc_die("Error: requested state %s invalid.\n", argv[2]);

 /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  PidcontrolGroupSetCommand(ipc, argv[1], "*", requested_state);
  return 0;
}
