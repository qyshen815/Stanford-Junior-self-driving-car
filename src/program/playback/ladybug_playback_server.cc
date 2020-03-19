#include <roadrunner.h>
#include <ipc_interface.h>
#include <ladybug_interface.h>
#include <applanix_interface.h>
#include <playback_interface.h>
#include <dgc_curses.h>
#include <llfplayer.h>
#include "ladybug_playback_server.h"

namespace dgc {

LadybugPlaybackServer::LadybugPlaybackServer(IpcInterface *ipc, 
					     LadybugInterface 
					     *lbug_int)
{
  ipc_ = ipc;
  lbug_interface_ = lbug_int;
  player_ = new llf_player(lbug_interface_);
  playback_speed_ = 1.0;
}

LadybugPlaybackServer::~LadybugPlaybackServer()
{
  player_->disable_playback();
  delete player_;
}

void LadybugPlaybackServer::ApplanixPoseHandler(ApplanixPose *pose)
{
  player_->set_pose(pose->smooth_x, pose->smooth_y, pose->smooth_z,
		    pose->roll, pose->pitch, pose->yaw, pose->timestamp);
}

void LadybugPlaybackServer::PlaybackCommandHandler(PlaybackCommand *command)
{
  if(command->cmd == DGC_PLAYBACK_COMMAND_RESET ||
     command->cmd == DGC_PLAYBACK_COMMAND_SEEK)
    player_->reset();
  playback_speed_ = command->speed;
}

void LadybugPlaybackServer::PrintPlaybackStats(int line_num, bool paused)
{
  double disk_rate, shm_rate, freq;

  dgc_curses_black();
  mvprintw(line_num, 0, "LADYBUG");

  player_->throughput_stats(&disk_rate, &shm_rate, &freq);
  
  dgc_curses_black();
  mvprintw(line_num, 11, "DISK:");
  dgc_curses_blue();
  mvprintw(line_num, 18, "%.1f MB/s", paused ? 0 : disk_rate * playback_speed_);
  
  dgc_curses_black();
  mvprintw(line_num, 28, "SHM:");
  dgc_curses_blue();
  mvprintw(line_num, 34, "%.1f MB/s", paused ? 0 : shm_rate * playback_speed_);
  
  dgc_curses_black();
  mvprintw(line_num, 45, "FREQ:");
  dgc_curses_blue();
  mvprintw(line_num, 51, "%.2f Hz", paused ? 0 : freq * playback_speed_);
}

void LadybugPlaybackServer::Setup(char *filename)
{
  if (player_->initialize(filename) < 0)
    dgc_die("Error: could not initialize ladybug playback.\n");

  player_->enable_playback();
}

}
