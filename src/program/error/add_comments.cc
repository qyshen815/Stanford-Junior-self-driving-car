#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <error_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;
  char line[200];

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  do {
    fprintf(stderr, "COMMENT: ");
    if(fgets(line, 200, stdin) != line)
      dgc_fatal_error("Failed to read from stdin");
    line[strlen(line) - 1] = '\0';
    SendErrorComment(ipc, line);
  } while(1);
  return 0;
}
