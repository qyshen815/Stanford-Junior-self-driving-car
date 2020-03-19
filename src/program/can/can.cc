#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include "cancore.h"

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);

  CanServer *can_server = new CanServer(ipc);
  can_server->Setup(pint, argc, argv);

  SignalHandler sig_handler;
  sig_handler.Start();
  while (!sig_handler.ReceivedSignal(SIGINT)) 
    can_server->ProcessInput();
  can_server->Shutdown();

  delete can_server;
  delete pint;
  delete ipc;
  return 0;
}
