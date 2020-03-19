#include <roadrunner.h>
#include <ipc_interface.h>
#include <playback_interface.h>

namespace dgc {

void SendPlaybackCommand(IpcInterface *ipc, int command, long int argument, 
			 float speed)
{
  static int first = 1;
  PlaybackCommand msg;
  int err;

  if (first) {
    err = ipc->DefineMessage(PlaybackCommandID);
    TestIpcExit(err, "Could not define message", PlaybackCommandID);
    first = 0;
  }
  msg.cmd = command;
  msg.arg = argument;
  msg.speed = speed;
  err = ipc->Publish(PlaybackCommandID, &msg);
  TestIpc(err, "Could not publish", PlaybackCommandID);
}

void SendPlaybackStatus(IpcInterface *ipc, PlaybackStatus *status)
{
  static int first = 1;
  int err;

  if (first) {
    err = ipc->DefineMessage(PlaybackStatusID);
    TestIpcExit(err, "Could not define message", PlaybackStatusID);
    first = 0;
  }
  err = ipc->Publish(PlaybackStatusID, status);
  TestIpc(err, "Could not publish", PlaybackStatusID);
}

}
