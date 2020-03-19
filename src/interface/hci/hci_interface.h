#ifndef DGC_HCI_INTERFACE_H
#define DGC_HCI_INTERFACE_H

#include <ipc_interface.h>
#include <hci_messages.h>

namespace dgc {

/** Send an hci audio msg. The message should be white
    space separated strings that will be used as keys
    to look up sound clips, which will be played in that
    order, as a very simple text to speech system */
  
void SendHciAudio(IpcInterface *ipc, char *message);
  
void QueueHciAudio(const char *message);

void SendHciAudioQueue(IpcInterface *ipc);

}

#endif
