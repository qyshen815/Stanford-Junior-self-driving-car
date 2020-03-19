#ifndef DGC_HEARTBEAT_MESSAGES_H
#define DGC_HEARTBEAT_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  char modulename[20];      /**< Name of the module emitting the heartbeat */
  double timestamp;         /**< DGC timestamp */
  char host[10];            /**< hostname associated with timestamp */
} Heartbeat;

#define     DGC_HEARTBEAT_NAME      "dgc_heartbeat"
#define     DGC_HEARTBEAT_FMT       "{[char:20],double,[char:10]}"

const IpcMessageID HeartbeatID = { DGC_HEARTBEAT_NAME, 
				   DGC_HEARTBEAT_FMT };

}

#endif
