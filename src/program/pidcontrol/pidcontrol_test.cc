#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>

using namespace dgc;

void pidtable_handler(PidcontrolPidtable *pidtable)
{
  int i;

  for(i = 0; i < pidtable->num_processes; i++) {
    fprintf(stderr, "%d : %20s %20s %10d %10d\n", 
            i, pidtable->process[i].module_name,
            pidtable->process[i].group_name,
            pidtable->process[i].active,
            pidtable->process[i].active ?
            pidtable->process[i].pid : -1);
  }
  fprintf(stderr, "\n");
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(PidcontrolPidtableID, &pidtable_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
