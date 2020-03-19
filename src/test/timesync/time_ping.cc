#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>

#include "timeping_messages.h"

using namespace dgc;

void publish_ping(IpcInterface *ipc)
{
  static TimePing ping;
  static int first = 1;
  int err;

  if(first) {
    strcpy(ping.host, dgc_hostname());
    first = 0;
  }
  
  ping.timestamp = dgc_get_time();

  err = ipc->Publish(TimePingID, &ping);
  TestIpcExit(err, "Could not publish", TimePingID);
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
  double current_time = 0;
  double last_publish = 0;
  IpcInterface *ipc;

  ipc = new IpcStandardInterface;
  if (ipc->Connect("timeping") < 0)
    dgc_die("Could not connect to IPC network.");

  register_ipc_messages(ipc);

  do {
    current_time = dgc_get_time();
    if(current_time - last_publish > 1.0) {
      publish_ping(ipc);
      fprintf( stderr, "(p)" );
      last_publish = current_time;
    }
    usleep(10000);
  } while(1);

  return 0;
}
