#ifndef DGC_ESTOP_MESSAGES_H
#define DGC_ESTOP_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define      DGC_ESTOP_DISABLE             0
#define      DGC_ESTOP_PAUSE               1
#define      DGC_ESTOP_RUN                 2

typedef struct {
  unsigned char estop_code;
  double timestamp;
  char host[10];
} EstopStatus;

#define      DGC_ESTOP_STATUS_NAME           "dgc_estop_status"
#define      DGC_ESTOP_STATUS_FMT            "{char,double,[char:10]}"

const IpcMessageID EstopStatusID = { DGC_ESTOP_STATUS_NAME, 
				     DGC_ESTOP_STATUS_FMT };

typedef struct {
  unsigned char estop_code;
  double timestamp;
  char host[10];
} EstopSoftstop;

#define      DGC_ESTOP_SOFTSTOP_NAME           "dgc_estop_softstop"
#define      DGC_ESTOP_SOFTSTOP_FMT            "{char,double,[char:10]}"

const IpcMessageID EstopSoftstopID = { DGC_ESTOP_SOFTSTOP_NAME, 
				       DGC_ESTOP_SOFTSTOP_FMT };

}

#endif

