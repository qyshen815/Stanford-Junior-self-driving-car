#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include "applanixcore.h"

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);

  SignalHandler signal_handler;
  signal_handler.Start();

  ApplanixServer *applanix = new ApplanixServer(ipc);
  applanix->Startup(pint, argc, argv);

  while (!signal_handler.ReceivedSignal(SIGINT))
    applanix->ProcessInput();
  fprintf(stderr, "\nAPPLANIX: Caught SIGINT.  Shutting down network.\n");
  applanix->Shutdown();
  return 0;
}
