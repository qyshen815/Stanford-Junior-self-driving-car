#ifndef DGC_CAN_IPC_H
#define DGC_CAN_IPC_H

#include <can_messages.h>

namespace dgc { 

void dgc_can_register_ipc_messages(void);

void dgc_can_publish_status_message(const dgc::CanStatus *status);

void dgc_can_publish_heartbeat_message(void);

}

#endif
