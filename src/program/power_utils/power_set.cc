#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <power_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  int i, named = 0;
  int err, power_command = 0;
  IpcInterface *ipc = NULL;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "%s channel state\n", argv[0]);

  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  for(i = 0; i < (int)strlen(argv[1]); i++)
    if(!isdigit(argv[1][i]))
      named = 1;

  for(i = 0; i < (int)strlen(argv[2]); i++)
    argv[2][i] = toupper(argv[2][i]);

  if(strcmp(argv[2], "0") == 0 ||
     strcmp(argv[2], "OFF") == 0)
    power_command = 0;
  else if(strcmp(argv[2], "1") == 0 ||
	  strcmp(argv[2], "ON") == 0)
    power_command = 1;
  else
    dgc_die("Error: power command %s invalid.\n", argv[2]);

  if(named) {
    fprintf(stderr, "Sending power command %s - %s ... ", argv[1],
	    power_command ? "ON" : "OFF");
    err = PowerSetNamedCommand(ipc, argv[1], power_command);
    if(err < 0) {
      fprintf(stderr, "failed.\n");
      return 1;
    }
    else
      fprintf(stderr, "success!\n");
  }
  else {
    fprintf(stderr, "Sending power command %d - %s ... ", atoi(argv[1]),
	    power_command ? "ON" : "OFF");
    err = PowerSetCommand(ipc, atoi(argv[1]), power_command);
    if(err < 0) {
      fprintf(stderr, "failed.\n");
      return 1;
    }
    else
      fprintf(stderr, "success!\n");
  }
  return 0;
}
