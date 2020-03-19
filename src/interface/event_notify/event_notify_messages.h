#ifndef DGC_EVENT_NOTIFY_MESSAGES_H
#define DGC_EVENT_NOTIFY_MESSAGES_H

#include <ipc_interface.h>

#define DGC_EVENT_NOTIFY_MODULE_LEN 20
#define DGC_EVENT_NOTIFY_MESSAGE_LEN 200

namespace dgc {

enum EventNotifySeverity {INFO, WARNING, ERROR, CRITICAL};
enum EventNotifyType     {DIALOG};

typedef struct {
  /**< Name of the module sending the notification*/
  char modulename[DGC_EVENT_NOTIFY_MODULE_LEN];      
  char message[DGC_EVENT_NOTIFY_MESSAGE_LEN];
  EventNotifySeverity severity; 
  EventNotifyType type;
  double timestamp;         /**< DGC timestamp */
  char host[10];            /**< hostname associated with timestamp */
} EventNotification;

#define     DGC_EVENT_NOTIFY_NAME      "dgc_event_notification"
#define     DGC_EVENT_NOTIFY_FMT       "{[char:20],[char:200],int,int,double,[char:10]}"

const IpcMessageID EventNotifyID = { DGC_EVENT_NOTIFY_NAME, 
				   DGC_EVENT_NOTIFY_FMT };
}

#endif
