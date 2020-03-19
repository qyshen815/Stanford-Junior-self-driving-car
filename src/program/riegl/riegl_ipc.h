#ifndef DGC_RIEGL_IPC_H
#define DGC_RIEGL_IPC_H

#include <ipc_interface.h>

namespace dgc {

void dgc_riegl_register_ipc_messages(IpcInterface *ipc);

void dgc_riegl_publish_laser_message(IpcInterface *ipc, dgc_riegl_p riegl);

}

#endif
