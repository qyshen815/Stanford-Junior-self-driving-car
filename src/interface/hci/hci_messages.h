#ifndef DGC_HCI_MESSAGES_H
#define DGC_HCI_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

  /** Audio message to be played on car speakers. Msg will be tokenized 
      and used as keys to lookup pre-recorded sound clips. */

typedef struct {
  char *msg;             /**< text of message to be read */
  char host[10];
  double timestamp;
} HciAudio;

#define        DGC_HCI_AUDIO_NAME      "dgc_hci_audio"
#define        DGC_HCI_AUDIO_FMT       "{string,[char:10],double}"

const IpcMessageID HciAudioID = { DGC_HCI_AUDIO_NAME, 
				  DGC_HCI_AUDIO_FMT };

}

#endif
