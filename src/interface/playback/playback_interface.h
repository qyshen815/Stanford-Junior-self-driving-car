#ifndef DGC_PLAYBACK_INTERFACE_H
#define DGC_PLAYBACK_INTERFACE_H

#include <ipc_interface.h>
#include <playback_messages.h>

namespace dgc {

void SendPlaybackCommand(IpcInterface *ipc, int pCommand, long int pArgument, 
			 float speed);

void SendPlaybackStatus(IpcInterface *ipc, PlaybackStatus *status);

}

#endif
