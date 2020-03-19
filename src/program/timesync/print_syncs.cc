#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <timesync_messages.h>
#include <param_interface.h>

using namespace dgc;

void sync_handler(TimesyncSync *sync)
{
  fprintf(stderr, "%s : %f %f %f %f\n", sync->host, sync->base_time,
	  sync->offset, sync->drift, sync->rms_err);
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(TimesyncSyncID, &sync_handler);
  ipc->Dispatch();
  return 0;
}
