#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <velodyne_shm_interface.h>
#include <playback_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include <dgc_curses.h>
#include "vlfplayer.h"
#include "velodyne_playback_server.h"

using namespace dgc;

static void PrintStats(VelodynePlaybackServer *server)
{
  clear();
  server->PrintPlaybackStats(0, 1.0);
  move(1, 0);
  refresh();
}

int main(int argc, char **argv)
{
  int err;

  if(argc != 2)
    dgc_die("Usage: %s vlf-filename\n", argv[0]);

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  VelodyneInterface *velo_int = new VelodyneShmInterface;
  if (velo_int->CreateServer() < 0) 
    dgc_die("Error: could not open velodyne interface.\n");

  ParamInterface *pint = new ParamInterface(ipc);

  VelodynePlaybackServer *server = 
    new VelodynePlaybackServer(ipc, pint, velo_int);
  server->Setup(argv[1], argc, argv);

  err = ipc->Subscribe(PlaybackCommandID, server,
		       &VelodynePlaybackServer::PlaybackCommandHandler);
  TestIpcExit(err, "Could not subscribe", PlaybackCommandID);
  err = ipc->Subscribe(ApplanixPoseID, server,
		       &VelodynePlaybackServer::ApplanixPoseHandler);
  TestIpcExit(err, "Could not subscribe", ApplanixPoseID);

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
  delete velo_int;
  delete pint;
  delete ipc;
  return 0;
}
