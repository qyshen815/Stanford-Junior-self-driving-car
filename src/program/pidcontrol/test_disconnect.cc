#include <roadrunner.h>
#include <ipc_std_interface.h>

using namespace dgc;

IpcInterface *ipc;

char module_name[256];

void x_ipcRegisterExitProc(void (*proc)(void));

static void reconnect_central(void)
{
  int err;
  
  do {
    dgc_warning("IPC died. Reconnecting...\n");
    if(ipc->IsConnected())
      ipc->Disconnect();
    err = ipc->Connect(module_name);
    if(err == 0)
      dgc_warning("Reconnected to IPC.\n");
    ipc->RegisterExitCallback(reconnect_central);
  } while(err == -1);
}

int main(int /*argc*/, char **argv)
{
  strcpy(module_name, argv[0]);

  ipc = new IpcStandardInterface;
  if (ipc->Connect(module_name) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  ipc->RegisterExitCallback(reconnect_central);
  ipc->Dispatch();
  return 0;
}
