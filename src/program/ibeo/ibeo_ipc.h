#ifndef DGC_IBEO_IPC_H
#define DGC_IBEO_IPC_H

#include <ipc_interface.h>
#include "ibeocore.h"

namespace dgc {

void dgc_ibeo_register_ipc_messages(IpcInterface *ipc);

void dgc_ibeo_publish_scans(IpcInterface *ipc, dgc_ibeo_p ibeo);

}

#endif
