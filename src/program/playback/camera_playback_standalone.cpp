#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <camera_shm_interface.h>
#include <applanix_interface.h>
#include <playback_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include <dgc_curses.h>
#include <camplayer.h>
#include "camera_playback_server.h"

using namespace dgc;

static void PrintStats(CameraPlaybackServer *server)
{
  clear();
  server->PrintPlaybackStats(0, false);
  move(1, 0);
  refresh();
}

int main(int argc, char **argv)
{
  int err, camera_num = 0;
  char *cam_filename;

  if(argc < 2)
    dgc_fatal_error("Usage: %s <camlog-filename> [camera_num]\n", argv[0]);

  cam_filename = argv[1];
  if(argc >= 3)
    camera_num = atoi(argv[2]);

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  CameraInterface *camera_interface = new CameraShmInterface;
  if(camera_interface->CreateServer(camera_num) < 0) 
    dgc_die("Error: could not open camera interface.\n");

  CameraPlaybackServer *server = 
    new CameraPlaybackServer(ipc, camera_interface);
  server->Setup(cam_filename);

  err = ipc->Subscribe(PlaybackCommandID, server,
		       &CameraPlaybackServer::PlaybackCommandHandler);
  TestIpcExit(err, "Could not subscribe,", PlaybackCommandID);
  err = ipc->Subscribe(ApplanixPoseID, server,
		       &CameraPlaybackServer::ApplanixPoseHandler);
  TestIpcExit(err, "Could not subscribe,", ApplanixPoseID);

  /* start the stats timer */
  dgc_curses_initialize();
  ipc->AddTimer(0.25, PrintStats, server);

  SignalHandler signal_handler;
  signal_handler.Start();

  while(!signal_handler.ReceivedSignal(SIGINT)) {
    ipc->Sleep(0.1);
  }

  fprintf(stderr, "\n# INFO: quit program with CTRL-C\n");
  dgc_curses_close();
  delete server;
  delete camera_interface;
  delete ipc;
  return 0;
}
