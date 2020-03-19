#ifndef DGC_POWER_IPC_H
#define DGC_POWER_IPC_H

#define          MAX_CHANNELS          16

typedef struct {
  int    name_active;
  char * channel_name;
  int    state;
} channel_t, *channel_p;

extern   int        num_channels;
extern   channel_t  channel[MAX_CHANNELS];

namespace dgc {

void dgc_power_register_ipc_messages(IpcInterface *ipc);

void dgc_power_publish_status_message(IpcInterface *ipc);

}

#endif
