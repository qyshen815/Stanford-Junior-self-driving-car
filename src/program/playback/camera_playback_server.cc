#include <roadrunner.h>
#include <ipc_interface.h>
#include <applanix_interface.h>
#include <camera_interface.h>
#include <playback_interface.h>
#include <param_interface.h>
#include <dgc_curses.h>
#include <camplayer.h>
#include "camera_playback_server.h"

namespace dgc {

CameraPlaybackServer::CameraPlaybackServer(IpcInterface *ipc,
					   CameraInterface *cam_int)
{
  ipc_ = ipc;
  camera_interface_ = cam_int;
  player = new cam_player(camera_interface_);
  playback_rate_ = 1.0;
}

CameraPlaybackServer::~CameraPlaybackServer()
{
  delete player;
}

void CameraPlaybackServer::ApplanixPoseHandler(ApplanixPose *pose)
{
  player->set_pose(pose->smooth_x, pose->smooth_y, pose->smooth_z,
		   pose->roll, pose->pitch, pose->yaw, pose->timestamp);
}

void CameraPlaybackServer::PlaybackCommandHandler(PlaybackCommand *command)
{
  if(command->cmd == DGC_PLAYBACK_COMMAND_RESET ||
     command->cmd == DGC_PLAYBACK_COMMAND_SEEK)
    player->reset();
  playback_rate_ = command->speed;
}

void CameraPlaybackServer::PrintPlaybackStats(int line_num, bool paused)
{
  double disk_rate, shm_rate, freq;

  dgc_curses_black();
  mvprintw(line_num, 0, "CAMERA");

  player->throughput_stats(&disk_rate, &shm_rate, &freq);
  
  dgc_curses_black();
  mvprintw(line_num, 11, "DISK:");
  dgc_curses_blue();
  mvprintw(line_num, 18, "%.1f MB/s", paused ? 0 : disk_rate * playback_rate_);
  
  dgc_curses_black();
  mvprintw(line_num, 28, "SHM:");
  dgc_curses_blue();
  mvprintw(line_num, 34, "%.1f MB/s", paused ? 0 : shm_rate * playback_rate_);
  
  dgc_curses_black();
  mvprintw(line_num, 45, "FREQ:");
  dgc_curses_blue();
  mvprintw(line_num, 51, "%.2f Hz", paused ? 0 : freq * playback_rate_);
}

void *camera_thread(void *ptr)
{
  cam_player *player = (cam_player *)ptr;
  player->play();
  return NULL;
}

void CameraPlaybackServer::Setup(char *filename)
{
  pthread_t thread;

  if(player->initialize(filename) < 0)
    dgc_die("Error: could not initialize camera playback.\n");

  /* start camera thread */
  pthread_create(&thread, NULL, camera_thread, player);
}

}
