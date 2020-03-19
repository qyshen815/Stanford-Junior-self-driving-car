#include <roadrunner.h>
#include <ipc_interface.h>
#include <velodyne_interface.h>
#include <playback_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <dgc_curses.h>
#include <vlfplayer.h>
#include "velodyne_playback_server.h"

namespace dgc {

VelodynePlaybackServer::VelodynePlaybackServer(IpcInterface *ipc,
					       ParamInterface *pint,
					       VelodyneInterface *velo_int)
{
  int_filename_ = NULL;
  cal_filename_ = NULL;
  calibrate_intensities_ = 0;

  ipc_ = ipc;
  pint_ = pint;
  velo_interface_ = velo_int;
  player_ = new vlf_player(velo_interface_);
  playback_speed_ = 1.0;
}

VelodynePlaybackServer::~VelodynePlaybackServer()
{
  player_->disable_playback();
  delete player_;
}

void VelodynePlaybackServer::ApplanixPoseHandler(ApplanixPose *pose)
{
  player_->set_pose(pose->smooth_x, pose->smooth_y, pose->smooth_z,
		    pose->roll, pose->pitch, pose->yaw, pose->timestamp);
}

void VelodynePlaybackServer::PlaybackCommandHandler(PlaybackCommand *command)
{
  if(command->cmd == DGC_PLAYBACK_COMMAND_RESET ||
     command->cmd == DGC_PLAYBACK_COMMAND_SEEK)
    player_->reset();
  playback_speed_ = command->speed;
}

void VelodynePlaybackServer::PrintPlaybackStats(int line_num, bool paused)
{
  double disk_rate, shm_rate, freq;

  dgc_curses_black();
  mvprintw(line_num, 0, "VELODYNE");

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

void VelodynePlaybackServer::ReadParameters(int argc, char **argv)
{
  Param params[] = { 
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename_, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset_, 0, NULL},
    {"velodyne", "int_file", DGC_PARAM_FILENAME, &int_filename_, 0, NULL},
    {"velodyne", "calibrate_intensities", DGC_PARAM_INT, &calibrate_intensities_, 0, NULL},
  };
  pint_->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
  if(!calibrate_intensities_) {
	int_filename_ = NULL;
  }
}

void VelodynePlaybackServer::Setup(char *vlf_filename, int argc, char **argv)
{
  ReadParameters(argc, argv);

  if(int_filename_ == NULL) {
	printf("about to init with NULL filename\n");
  } else {
	printf("about to init with file: %s\n", int_filename_);
  }
  if(player_->initialize(vlf_filename, cal_filename_, int_filename_, velodyne_offset_) < 0)
    dgc_die("Error: could not initialize velodyne playback.\n");

  player_->enable_playback();
}

}
