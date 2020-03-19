#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>

using namespace dgc;

#define MAX_STRING_LENGTH  400

int main(int argc, char **argv)
{
  IpcInterface *ipc = NULL;
  int requested_state = 0;
  int watch_heartbeats = 0;
  char command[MAX_STRING_LENGTH] = "";
  int i;

  if(argc < 5)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s modulename groupname requested-state command\n", 
	    argv[0]);
  
  if(strcmp(argv[3], "0") == 0 ||
     strcmp(argv[3], "OFF") == 0 ||
     strcmp(argv[3], "DOWN") == 0)
    requested_state = 0;
  else if(strcmp(argv[3], "1") == 0 ||
          strcmp(argv[3], "ON") == 0 ||
          strcmp(argv[3], "UP") == 0)
    requested_state = 1;
  else
    dgc_die("Error: requested state %s invalid.\n", argv[4]);

  /* connect to the IPC server */
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  strncat(command, argv[4], MAX_STRING_LENGTH - 1);

  for (i=5; i<argc; i++) {
    strncat(command, " ", MAX_STRING_LENGTH - 1);
    strncat(command, argv[i], MAX_STRING_LENGTH - 1);
  }

  PidcontrolAddProcessCommand(ipc, argv[1], argv[2], "*", requested_state, 
			      watch_heartbeats, command );
  return 0;
}
