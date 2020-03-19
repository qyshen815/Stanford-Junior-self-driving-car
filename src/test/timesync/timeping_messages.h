#ifndef DGC_TIME_PING_MESSAGES_H
#define DGC_TIME_PING_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  double timestamp;
  char host[10];
} TimePing;

#define    DGC_TIME_PING_NAME    "dgc_time_ping"
#define    DGC_TIME_PING_FMT     "{double,[char:10]}"

const IpcMessageID TimePingID = { DGC_TIME_PING_NAME, 
				  DGC_TIME_PING_FMT };

}

#endif
