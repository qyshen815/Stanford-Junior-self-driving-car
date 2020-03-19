#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>
#include <param_interface.h>

using namespace dgc;

void output_handler(PidcontrolOutput *output)
{
  fprintf(stderr, "%s", output->output);
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  ipc->Subscribe(PidcontrolOutputID, &output_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
