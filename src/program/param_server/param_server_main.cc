#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <signal_handler.h>
#include "param_server.h"

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->ConnectLocked(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  SignalHandler signal_handler;
  signal_handler.Start();

  ParamServer *param_server = new ParamServer(ipc);
  param_server->Startup(argc, argv);

  while (!signal_handler.ReceivedSignal(SIGINT)) 
    ipc->Sleep(0.1);

  delete param_server;
  delete ipc;
  return 0;
}
