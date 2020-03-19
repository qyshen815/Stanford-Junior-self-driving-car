#include <roadrunner.h>
#include <ipc_interface.h>
#include <heartbeat_interface.h>
//#include <laser_interface.h>
//#include <ibeo_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
//#include <riegl_interface.h>
#include <controller_interface.h>
#include <planner_interface.h>
#include <estop_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
//#include <ldlrs_interface.h>
#include <radar_interface.h>
#include <passat_interface.h>
#include <error_interface.h>
#include <playback_interface.h>
#include <simulator_interface.h>
#include <healthmon_interface.h>
#include <timesync_interface.h>
#include <velodyne_shm_interface.h>
#include <ladybug_shm_interface.h>
#include <trajectory_points_interface.h>
#include <signal_handler.h>
#include <dgc_curses.h>
#include <logio.h>
#include "velodyne_playback_server.h"
#include "ladybug_playback_server.h"
#include "playback_server.h"

namespace dgc {

CombinedPlaybackServer::CombinedPlaybackServer(IpcInterface *ipc, 
					       bool basic,
					       bool use_velodyne,
					       VelodynePlaybackServer 
					       *velo_server, bool use_ladybug,
					       LadybugPlaybackServer 
					       *lbug_server)
{
  ipc_ = ipc;
  basic_ = basic;
  use_velodyne_ = use_velodyne;
  velo_server_ = velo_server;
  use_ladybug_ = use_ladybug;
  lbug_server_ = lbug_server;

  paused_ = true;
  hit_eof_ = false;
  playback_starttime_ = 0;
  current_position_ = 0;
  playback_timestamp_ = 0;
  line_buffer_ = NULL;
  offset_ = 0;
  advance_frame_ = 0;
  playback_speed_ = 1.0;
  last_message_num_ = -1;
  last_print_stats_ = 0;

  infile_ = NULL;
  valid_index_ = false;
}

void CombinedPlaybackServer::PlaybackCommandHandler(PlaybackCommand *command)
{
  switch(command->cmd) {
  case DGC_PLAYBACK_COMMAND_PLAY:
    if(paused_ && !hit_eof_) {
      playback_starttime_ = 0.0;
      paused_ = false;
    }
    break;
  case DGC_PLAYBACK_COMMAND_STOP:
    if(!paused_) {
      paused_ = true;
    }
    break;
  case DGC_PLAYBACK_COMMAND_RESET:
    if(!paused_)
      paused_ = true;
    current_position_ = 0;
    playback_starttime_ = 0.0;
    playback_timestamp_ = 0;
    line_buffer_->Reset();
    hit_eof_ = false;
    break;
  case DGC_PLAYBACK_COMMAND_FORWARD:
    if(!hit_eof_) {
      line_buffer_->Reset();
      offset_ = command->arg;
      if(offset_ > 0 && paused_)
      advance_frame_ = 1;
    }
    break;
  case DGC_PLAYBACK_COMMAND_REWIND:
    if(!hit_eof_) {
      line_buffer_->Reset();
      offset_ = -1 * command->arg;
      if(offset_ < 0 && paused_)
	advance_frame_ = 1;
    }
    break;
  case DGC_PLAYBACK_COMMAND_SET_SPEED:
    break;
  case DGC_PLAYBACK_COMMAND_SEEK:
    line_buffer_->Reset();
    if(!paused_)
      paused_ = true;
    advance_frame_ = 1;
    current_position_ = command->arg;
    hit_eof_ = false;
    break;
  }
  if(command->speed > 0 &&
     fabs(command->speed - playback_speed_) > 0.001) {
    playback_starttime_ = 0.0;
    playback_speed_ = command->speed;
  }
}

void CombinedPlaybackServer::Setup(char *ipc_filename)
{
  int err;

  infile_ = dgc_fopen(ipc_filename, "r");
  if(infile_ == NULL)
    dgc_die("Error: could not open file %s for reading.\n", ipc_filename);

  err = logfile_index_.Load(infile_->filename);
  valid_index_ = (err == 0);
  fprintf(stderr, "valid index = %d\n", valid_index_);

  /* register the message callbacks */
  HeartbeatAddLogReaderCallbacks(&callbacks_);
  //IbeoAddLogReaderCallbacks(&callbacks_);
  //LaserAddLogReaderCallbacks(&callbacks_);
  //RieglAddLogReaderCallbacks(&callbacks_);
  CanAddLogReaderCallbacks(&callbacks_);
  ApplanixAddLogReaderCallbacks(&callbacks_);
  ControllerAddLogReaderCallbacks(&callbacks_);
  PlannerAddLogReaderCallbacks(&callbacks_);
  EstopAddLogReaderCallbacks(&callbacks_);
  PassatAddLogReaderCallbacks(&callbacks_);
  LocalizeAddLogReaderCallbacks(&callbacks_);
  //LdlrsAddLogReaderCallbacks(&callbacks_);
  RadarAddLogReaderCallbacks(&callbacks_);
  RadarLRR3AddLogReaderCallbacks(&callbacks_);
  ErrorAddLogReaderCallbacks(&callbacks_);
  SimulatorAddLogReaderCallbacks(&callbacks_);
  HealthmonAddLogReaderCallbacks(&callbacks_);
  TimesyncAddLogReaderCallbacks(&callbacks_);
  vlr::TrajectoryPointsAddLogReaderCallbacks(&callbacks_);

  callbacks_.DefineIpcMessages(ipc_);

  line_buffer_ = new LineBuffer;

  /* subscribe to playback commands */
  ipc_->Subscribe(PlaybackCommandID, this, 
		  &CombinedPlaybackServer::PlaybackCommandHandler,
		  DGC_SUBSCRIBE_ALL);
  dgc_curses_initialize();
}

void CombinedPlaybackServer::PrintPlaybackStats(void)
{
  char str[100];
  
  clear();
  if(hit_eof_)
    sprintf(str, "EOF");
  else if(paused_)
    sprintf(str, "PAUSED");
  else
    sprintf(str, "PLAYING");

  dgc_curses_black();
  mvprintw(0, 0, "DGC PLAYBACK");
  dgc_curses_blue();
  mvprintw(0, 18, "%s", str);

  dgc_curses_black();
  mvprintw(0, 28, "TIME:");
  dgc_curses_blue();
  mvprintw(0, 38, "%.2f s", playback_timestamp_);

  if(basic_) {
    dgc_curses_blue();
    mvprintw(0, 51, "[BASIC]", playback_timestamp_);
  }

  dgc_curses_black();
  mvprintw(1, 0, "----------------------------------------------------------");

  dgc_curses_black();
  mvprintw(2, 0, "VELODYNE");
  if (velo_server_ == NULL) {
    dgc_curses_blue();
    if (!use_velodyne_)
      mvprintw(2, 18, "DISABLED");
    else
      mvprintw(2, 18, "NO FILE FOUND");
  } else {
    velo_server_->PrintPlaybackStats(2, paused_);
  }

  dgc_curses_black();
  mvprintw(3, 0, "LADYBUG ");
  if (lbug_server_ == NULL) {
    dgc_curses_blue();
    if (!use_ladybug_)
      mvprintw(3, 18, "DISABLED");
    else
      mvprintw(3, 18, "NO FILE FOUND");
  } else {
    lbug_server_->PrintPlaybackStats(3, paused_);
  }

  move(4, 0);
  refresh();
}

void CombinedPlaybackServer::WaitForTimestamp(double ts) 
{
  double current_time, towait;
  struct timeval tv;

  if(playback_starttime_ == 0.0)
    playback_starttime_ = (dgc_get_time() - ts / playback_speed_);
  current_time = (dgc_get_time() - playback_starttime_) * playback_speed_;
  if(!paused_ && ts > current_time) {
    towait = (ts - current_time) / playback_speed_;
    tv.tv_sec = (int)floor(towait);
    tv.tv_usec = (int)((towait - tv.tv_sec) * 1e6);
    select(0, NULL, NULL, NULL, &tv);
  }
}

int CombinedPlaybackServer::ReadMessage(int message_num, int publish)
{
  char *line = NULL;
  ApplanixPose *pose;
  char *mark = NULL, command[100];
  double current_time;
  int j, err;
  bool found_pose = false;
  LogReaderCallback *cb;

  if(message_num <= last_message_num_) {
    dgc_fseek(infile_, 0, SEEK_SET);
    last_message_num_ = -1;
  }

  if(message_num != last_message_num_ + 1 && valid_index_) {
    dgc_fseek(infile_, logfile_index_.Offset(message_num), SEEK_SET);
    last_message_num_ = message_num - 1;
  }

  while(message_num > last_message_num_) {
    /* read the next line */
    line = line_buffer_->ReadLine(infile_);
    if(line == NULL) 
      return -1;
    
    mark = dgc_next_word(line);
    last_message_num_++;
  }
  
  /* copy the command over */
  j = 0;
  while(line[j] != ' ') {
    command[j] = line[j];
    j++;
  }
  command[j] = '\0';

  cb = callbacks_.FindCallback(command);
  if (cb == NULL)
    return false;

  if(!basic_ || !cb->interpreted) {
    mark = cb->conv_func(mark, cb->message_data);
    playback_timestamp_ = atof(mark);
    
    /* if we read an applanix message, use it to sync the velodyne
       playback */
    if(strcmp(cb->logger_message_name, "APPLANIX_POSE_V2") == 0) {
      pose = (ApplanixPose *)cb->message_data;
      if (velo_server_)
	velo_server_->ApplanixPoseHandler(pose);
      if (lbug_server_)
	lbug_server_->ApplanixPoseHandler(pose);
      found_pose = true;
    }
    
    if(publish) {
      current_time = dgc_get_time();
      WaitForTimestamp(playback_timestamp_);
      err = ipc_->Publish(cb->messageID, cb->message_data);
    }
    return found_pose;
  }
  return found_pose;
}

void CombinedPlaybackServer::ProcessData(void)
{
  double current_time;
  int err;

  current_time = dgc_get_time();
  if(current_time - last_print_stats_ > 0.25) {
    PrintPlaybackStats();
    last_print_stats_ = current_time;
  }
  
  if(offset_ != 0) {
    playback_starttime_ = 0.0;
    current_position_ += offset_;
    if(current_position_ < 0)
      current_position_ = 0;
    offset_ = 0;
  }
  
  if(!hit_eof_) {
    if(!paused_) {
      err = ReadMessage(current_position_, 1);
      if(err < 0) {
	paused_ = 1;
	hit_eof_ = 1;
      }
      else
	current_position_++;
    }
    else if(paused_ && advance_frame_) {
      do {
	err = ReadMessage(current_position_, 1);
	if(err < 0) {
	  paused_ = 1;
	  hit_eof_ = 1;
	}
	else
	  current_position_++;
      } while(err == 0);
      advance_frame_ = 0;
    }
  }
}

void CombinedPlaybackServer::Shutdown(void)
{
  dgc_curses_close();
}

}
