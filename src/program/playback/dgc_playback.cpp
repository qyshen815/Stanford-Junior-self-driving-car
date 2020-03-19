#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <velodyne_shm_interface.h>
#include <ladybug_shm_interface.h>
#include <signal_handler.h>
#include "playback_server.h"
#include "velodyne_playback_server.h"
#include "ladybug_playback_server.h"
#include <stdint.h>

using namespace dgc;

void usage(char *fmt, ...) 
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  fprintf(stderr, "Usage: playback filename <args>\n");
  fprintf(stderr, "\t-basic        - no interpreted messages.\n");
  fprintf(stderr, "\t-novelo       - don't playback velodyne.\n");
  fprintf(stderr, "\t-noladybug    - don't playback ladybug.\n");
  exit(-1);
}

int main(int argc, char **argv)
{
  char *completed_filename = NULL, *ipc_filename = NULL, *vlf_filename = NULL;
  char *llf_filename = NULL;
  int i, err;
  uint32_t velo_shm_key = 0;

  bool basic = false, use_velodyne = true, use_ladybug = true;
  VelodyneInterface *velo_interface = NULL;  
  VelodynePlaybackServer *velo_server = NULL;

  LadybugInterface *lbug_interface = NULL;
  LadybugPlaybackServer *lbug_server = NULL;

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);

  Param params[] = { 
    {"velodyne", "shm_key", DGC_PARAM_INT, &velo_shm_key, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
  
  if(argc < 2)
    usage("Error: Not enough arguments\n\n");

  /* try to find IPC, ladybug, and velodyne filenames */
  for(i = 1; i < argc; i++) 
    if(dgc_complete_filename(argv[i], ".log.gz", &completed_filename) ||
        dgc_complete_filename(argv[i], ".log", &completed_filename)) {
      if(ipc_filename != NULL) 
        dgc_die("Error: %s only accepts one .log or .log.gz file as input.\n",
            argv[0]);
      ipc_filename = completed_filename;
    }
    else if(dgc_complete_filename(argv[i], ".vlf", &completed_filename)) {
      if(vlf_filename != NULL) 
        dgc_die("Error: %s only accepts one .vlf file as input.\n", argv[0]);
      vlf_filename = completed_filename;
    }
    else if(dgc_complete_filename(argv[i], ".llf", &completed_filename)) {
      if(llf_filename != NULL) 
        dgc_die("Error: %s only accepts one .llf file as input.\n", argv[0]);
      llf_filename = completed_filename;
    }
    else if(strcmp(argv[i], "-basic") == 0)
      basic = true;
    else if(strcmp(argv[i], "-novelo") == 0)
      use_velodyne = false;
    else if(strcmp(argv[i], "-noladybug") == 0)
      use_ladybug = false;
    else if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
      usage(NULL);

  if(ipc_filename == NULL)
    dgc_die("Error: could not find unique .log.gz file in commandline"
        " parameters.\n");

  SignalHandler signal_handler;
  signal_handler.Start();

  if(use_velodyne) {
    /* if we don't have a vlf filename, see if we can find one */
    if(vlf_filename == NULL) {
      vlf_filename = find_matching_logfile(ipc_filename, ".vlf", 30);
      if(vlf_filename == NULL)
        fprintf(stderr, "Error: could not find vlf file matching %s\n", 
            ipc_filename);
      else
        fprintf(stderr, "Found matching vlf file %s\n", vlf_filename);
    }

    if(vlf_filename != NULL) {
      /* create velodyne interface and player */
      velo_interface = new VelodyneShmInterface;
      if(velo_shm_key != 0)
        velo_interface->SetKey(velo_shm_key);
      if(velo_interface->CreateServer() < 0) 
        dgc_die("Error: could not open velodyne interface.\n");

      velo_server = new VelodynePlaybackServer(ipc, pint, velo_interface);
      velo_server->Setup(vlf_filename, argc, argv);

      err = ipc->Subscribe(PlaybackCommandID, velo_server,
          &VelodynePlaybackServer::PlaybackCommandHandler);
      TestIpcExit(err, "Could not subscribe", PlaybackCommandID);
    }
  }

  if(use_ladybug) {
    /* if we don't have a llf filename, see if we can find one */
    if(llf_filename == NULL) {
      llf_filename = find_matching_logfile(ipc_filename, ".llf", 30);
      if(llf_filename == NULL)
        fprintf(stderr, "Error: could not find llf file matching %s\n", 
            ipc_filename);
      else
        fprintf(stderr, "Found matching llf file %s\n", llf_filename);
    }

    if(llf_filename != NULL) {
      /* create ladybug interface and player */
      lbug_interface = new LadybugShmInterface;
      if(lbug_interface->CreateServer() < 0) 
        dgc_die("Error: could not open ladybug interface.\n");

      lbug_server = new LadybugPlaybackServer(ipc, lbug_interface);
      lbug_server->Setup(llf_filename);

      err = ipc->Subscribe(PlaybackCommandID, lbug_server,
          &LadybugPlaybackServer::PlaybackCommandHandler);
      TestIpcExit(err, "Could not subscribe", PlaybackCommandID);
    }
  }

  CombinedPlaybackServer *playback_server = 
    new CombinedPlaybackServer(ipc, basic, use_velodyne,
        velo_server, use_ladybug, lbug_server);

  playback_server->Setup(ipc_filename);
  do {
    playback_server->ProcessData();
    if (playback_server->paused())
      ipc->Sleep(0.1);
    else
      ipc->SleepUntilClear(0.);
  } while(!signal_handler.ReceivedSignal(SIGINT));
  playback_server->Shutdown();

  if (lbug_server) {
    delete lbug_server;
    delete lbug_interface;
  }
  if (velo_server) {
    delete velo_server;
    delete velo_interface;
  }
  delete pint;
  delete ipc;
  return 0;
}
