#ifndef DGC_EVENT_NOTIFY_INTERFACE_H
#define DGC_EVENT_NOTIFY_INTERFACE_H

#include <ipc_interface.h>
#include <event_notify_messages.h>

namespace dgc {

void SendEventNotification(IpcInterface *ipc, char* module, char *message);

}

#endif
