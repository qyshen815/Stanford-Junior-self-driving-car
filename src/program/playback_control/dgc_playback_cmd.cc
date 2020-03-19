#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <playback_interface.h>

using namespace dgc;

int main(int argc, char **argv) 
{
  IpcInterface *ipc;
  float speed = -1; // don't change existing speed
 
  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if(argc >= 2) 
    speed = atof(argv[1]);
  
  if( strrchr(argv[0], '/') != NULL )
    argv[0] = strrchr(argv[0], '/')+1;
  
  if(strcmp(argv[0], "dgc_playback_play") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_PLAY, 0, speed);
  else if(strcmp(argv[0], "dgc_playback_stop") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_STOP, 0, speed);
  else if(strcmp(argv[0], "dgc_playback_reset") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_RESET, 0, speed);
  else fprintf(stderr, "Unrecognized command: %s\n"
               "Needs to be dgc_playback_{play|stop|reset}\n",
               argv[0]);
  ipc->Disconnect();
  return 0;
}
