#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <timesync_interface.h>

#include "timeping_messages.h"

using namespace dgc;

TimeSync tsync;


void 
timeping_handler( TimePing *ping )
{
  fprintf( stderr, "[%s]: %f [%f]\n", 
	   ping->host, ping->timestamp,
	   tsync.getServerTime(ping->timestamp,ping->host) );
}

void 
timesync_handler( TimesyncSync *sync )
{
  tsync.updateSync(sync);
}

void register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    TimePingID, HeartbeatID 
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

int main(void)
{
  IpcInterface *ipc;
  ipc = new IpcStandardInterface;
  if (ipc->Connect("timeping") < 0)
    dgc_die("Could not connect to IPC network.");

  register_ipc_messages(ipc);

  ipc->Subscribe(TimePingID, &timeping_handler, DGC_SUBSCRIBE_ALL, NULL);
  ipc->Subscribe(TimesyncSyncID, &timesync_handler, DGC_SUBSCRIBE_ALL, NULL);

  ipc->Dispatch();

  return 0;
}
