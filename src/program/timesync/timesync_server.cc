#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <timesync_messages.h>
#include "udp.h"
#include "timesync.h"

using namespace dgc;

void publish_offset(IpcInterface *ipc, double base_time, double offset, 
		    double drift,  double rms_err)
{
  static TimesyncSync sync;
  static int first = 1;
  int err;

  if(first) {
    strcpy(sync.host, dgc_hostname());
    first = 0;
  }
  
  sync.base_time = base_time;
  sync.offset = offset;
  sync.drift = drift;
  sync.rms_err = rms_err;
  sync.timestamp = dgc_get_time();

  err = ipc->Publish(TimesyncSyncID, &sync);
  TestIpcExit(err, "Could not publish", TimesyncSyncID);
}

void register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    TimesyncSyncID, HeartbeatID 
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

int main(void)
{
  udp_sender_p udpsender = NULL;
  udp_listener_p udplistener = NULL;
  struct sockaddr_in senderaddr;
  time_query_t query;
  time_response_t response;
  int n;
  double last_publish = 0, current_time;
  IpcInterface *ipc;

  ipc = new IpcStandardInterface;
  if (ipc->ConnectLocked("timesync") < 0)
    dgc_die("Could not connect to IPC network.");

  register_ipc_messages(ipc);

  udpsender = udp_sender_init("127.0.0.1", CLIENT_PORT);
  if(udpsender == NULL)
    dgc_die("Error; could not open udp sender\n");

  udplistener = udp_listener_init(SERVER_PORT);
  if(udplistener == NULL)
    dgc_die("Error: could not open udp listener on port %d\n", SERVER_PORT);

  do {
    n = udp_listener_receive(udplistener, &query, sizeof(time_query_t),
			     &senderaddr, 0.25);
    if(n == sizeof(time_query_t)) {
      /* send packet back to where it came from, but different port */
      udpsender->remoteaddr.sin_addr = senderaddr.sin_addr;

      response.count = query.count;
      response.query_timestamp = query.timestamp;
      response.server_timestamp = dgc_get_time();

      if(udp_sender_send(udpsender, &response, sizeof(response), 0.1) < 0)
	fprintf(stderr, "Error: could not send response.\n");
    }

    current_time = dgc_get_time();
    if(current_time - last_publish > 1.0) {
      publish_offset(ipc, 0, 0, 0, 0);
      last_publish = current_time;
    }
  } while(1);

  udp_listener_close(udplistener);
  return 0;
}
