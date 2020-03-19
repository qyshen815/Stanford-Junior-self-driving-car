#include <roadrunner.h>
#include <ipc_interface.h>
#include <event_notify_messages.h>

namespace dgc {

void SendEventNotification(IpcInterface *ipc, char *module, char *message) {
  EventNotification event;
  int err;
  static int first = 1;

  if(first) {
    strncpy(event.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(EventNotifyID);
    TestIpcExit(err, "Could not define message", EventNotifyID);
  }
  strncpy(event.modulename, module, DGC_EVENT_NOTIFY_MODULE_LEN);
  strncpy(event.message, message, DGC_EVENT_NOTIFY_MESSAGE_LEN);
  err = ipc->Publish(EventNotifyID, &event);
  TestIpc(err, "Could not publish", EventNotifyID);
}

}
