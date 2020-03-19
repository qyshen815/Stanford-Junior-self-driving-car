#include <roadrunner.h>
#include <power_messages.h>
#include <heartbeat_messages.h>

#include "power_ipc.h"

extern int num_channels;

namespace dgc {

void power_set_handler(PowerSetQuery *power);

void power_setnamed_handler(PowerSetNamedQuery *power);

void dgc_power_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    PowerSetQueryID, PowerSetNamedQueryID, PowerSetResponseID, PowerStatusID,
    HeartbeatID
  };
  int err;

  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));

  err = ipc->Subscribe(PowerSetQueryID, &power_set_handler, DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", PowerSetQueryID);

  err = ipc->Subscribe(PowerSetNamedQueryID, &power_setnamed_handler, 
		       DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", PowerSetNamedQueryID);
}

void dgc_power_publish_status_message(IpcInterface *ipc)
{
  static PowerStatus status;
  static int first = 1;
  int i, ctr, err;
  
  if(first) {
    strncpy(status.host, dgc_hostname(), 10);
    status.num_modules = num_channels;
    status.module = (PowerModule *)malloc(num_channels * sizeof(PowerModule));
    first = 0;
  }

  ctr = 0;
  for (i=0; i<MAX_CHANNELS; i++) {
    if (channel[i].name_active) {
      status.module[ctr].channel = i;
      strncpy( status.module[ctr].name, channel[i].channel_name, 20 );
      status.module[ctr].state = channel[i].state;
      ctr++;
    }
  }
  status.timestamp = dgc_get_time();
  
  err = ipc->Publish(PowerStatusID, &status);
  TestIpcExit(err, "Could not publish", PowerStatusID);
}

}
