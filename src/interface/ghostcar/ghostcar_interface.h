#ifndef DGC_GHOSTCAR_INTERFACE_H
#define DGC_GHOSTCAR_INTERFACE_H

#include <ipc_interface.h>
#include <ghostcar_messages.h>

namespace dgc {

void GhostcarPoseWrite(GhostcarPose *msg, double logger_timestamp, 
		       dgc_FILE *outfile);

void GhostcarSyncCommand(IpcInterface *ipc, double lat, double lon);

}

#endif
