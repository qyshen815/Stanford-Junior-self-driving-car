#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <signal_handler.h>
#include "laser.h"

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  SignalHandler signal_handler;
  signal_handler.Start();

  ParamInterface *pint = new ParamInterface(ipc);
  LaserServer *laser_server = new LaserServer(ipc);
  laser_server->Setup(pint, argc, argv);

  do {
    laser_server->ProcessData();
    ipc->Sleep(0.005);
  } while(!signal_handler.ReceivedSignal(SIGINT));

  laser_server->Shutdown();
  delete laser_server;
  delete pint;
  delete ipc;
  return 0;
}

