#ifndef    DGC_PLAYBACK_MESSAGES_H
#define    DGC_PLAYBACK_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define    DGC_PLAYBACK_COMMAND_PLAY         0
#define    DGC_PLAYBACK_COMMAND_STOP         1
#define    DGC_PLAYBACK_COMMAND_RESET        2
#define    DGC_PLAYBACK_COMMAND_FORWARD      3
#define    DGC_PLAYBACK_COMMAND_REWIND       4
#define    DGC_PLAYBACK_COMMAND_FWD_SINGLE   5
#define    DGC_PLAYBACK_COMMAND_RWD_SINGLE   6
#define    DGC_PLAYBACK_COMMAND_SET_SPEED    7
#define    DGC_PLAYBACK_COMMAND_SEEK         8

typedef struct {
  int cmd;
  long int arg;
  float speed;
} PlaybackCommand;

#define DGC_PLAYBACK_COMMAND_NAME     "dgc_playback_command"
#define DGC_PLAYBACK_COMMAND_FMT      "{int,long,float}"

const IpcMessageID PlaybackCommandID = { DGC_PLAYBACK_COMMAND_NAME, 
					 DGC_PLAYBACK_COMMAND_FMT };

#define DGC_PLAYBACK_STATUS_PLAYING          0
#define DGC_PLAYBACK_STATUS_PAUSED           1

typedef struct {
  int mode;
  float percent_read;
  float speed;
} PlaybackStatus;


#define DGC_PLAYBACK_STATUS_NAME     "dgc_playback_status"
#define DGC_PLAYBACK_STATUS_FMT      "{int, float, float}"

const IpcMessageID PlaybackStatusID = {  DGC_PLAYBACK_STATUS_NAME, 
					 DGC_PLAYBACK_STATUS_FMT };

}

#endif
