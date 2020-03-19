#ifndef DGC_TIMESYNC_MESSAGES_H
#define DGC_TIMESYNC_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  double base_time;
  double offset;
  double drift;
  double rms_err;
  
  double timestamp;
  char host[10];
} TimesyncSync;

#define    DGC_TIMESYNC_SYNC_NAME    "dgc_timesync_sync"
#define    DGC_TIMESYNC_SYNC_FMT     "{double,double,double,double,double,[char:10]}"

const IpcMessageID TimesyncSyncID = { DGC_TIMESYNC_SYNC_NAME, 
				      DGC_TIMESYNC_SYNC_FMT };

}

#endif
