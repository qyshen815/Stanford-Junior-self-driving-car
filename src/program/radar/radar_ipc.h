#ifndef DGC_RADAR_IPC_H
#define DGC_RADAR_IPC_H

#include <roadrunner.h>
#include <ipc_interface.h>
#include "radarcore.h"

namespace dgc {

void dgc_radar_register_ipc_messages(IpcInterface *ipc);

void dgc_radar_publish_targets(IpcInterface *ipc, dgc_bosch_lrr3_radar_p radar, 
			       int sensor_num);

}

#endif
