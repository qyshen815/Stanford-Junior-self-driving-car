#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <passat_interface.h>
#include <param_interface.h>

using namespace dgc;

bool quit_signal = false;

void shutdown_rev_module(int x)
{
  if(x == SIGINT) {
    quit_signal = true;
  }
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.\n");
  signal(SIGINT, shutdown_rev_module);

  do {
    PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 0, 0, 0.2);
    ipc->Sleep(0.5);
    PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 0, 0, 0);
    ipc->Sleep(1.0);
  } while(!quit_signal);
  PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 0, 0, 0);
  return 0;
}
