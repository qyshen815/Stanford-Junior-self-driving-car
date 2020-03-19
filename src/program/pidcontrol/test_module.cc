#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <heartbeat_interface.h>

using namespace dgc;

int main(int argc, char ** argv)
{
  IpcInterface *ipc = new IpcStandardInterface;
  int stdout_count = 0, stderr_count = 0;

  /* connect to the IPC server, regsiter messages */
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s modulename\n", argv[0]);

  setlinebuf(stdout);
  do {
    if(stdout_count < 5)
      PublishHeartbeat(ipc, "BLAH");
    
    //    fprintf(stdout, "%s OUT: %d\n", module_name, stdout_count);
    //    fprintf(stderr, "%s ERR: %d\n", module_name, stderr_count);

    //   fprintf(stderr, "%d", stdout_count);

    fprintf(stderr, "test_module\n");
    fprintf(stdout, "12345678901234567890\n");

    stdout_count++;
    stderr_count++;
    ipc->Sleep(1.0);
  } while(1);
  return 0;
}
