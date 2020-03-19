#ifndef DGC_LDLRS_IPC_H
#define DGC_LDLRS_IPC_H

#include <ipc_interface.h>
#include "ldlrscore.h"

namespace dgc {

void dgc_ldlrs_register_ipc_messages(IpcInterface *ipc);

void dgc_ldlrs_publish_scans(IpcInterface *ipc, dgc_ldlrs_p ldlrs, 
			     int use_intensity);
 
}

#endif
