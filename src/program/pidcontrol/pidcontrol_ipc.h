#ifndef DGC_PIDCONTROL_IPC_H
#define DGC_PIDCONTROL_IPC_H

#include <roadrunner.h>
#include <ipc_interface.h>
#include "pidcontrol.h"

namespace dgc {

void pidcontrol_register_ipc_messages(IpcInterface *ipc);

void pidcontrol_publish_output(IpcInterface *ipc, int pid, char *output);

void pidcontrol_publish_pidtable(IpcInterface *ipc, int num_processes, 
				 process_info_p process);

}

#endif
