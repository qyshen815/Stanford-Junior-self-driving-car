#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <error_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  char message[256];
  int i;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  message[0] = '\0';
  for(i = 1; i < argc; i++) {
    strcat(message, argv[i]);
    if(i != argc - 1)
      strcat(message, " ");
  }
  SendErrorString(ipc, message);
  return 0;
}
