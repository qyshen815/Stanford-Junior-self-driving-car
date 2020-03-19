#ifndef DGC_VELODYNE_PLAYBACK_SERVER_H
#define DGC_VELODYNE_PLAYBACK_SERVER_H

#include <ipc_interface.h>
#include <velodyne_interface.h>
#include <playback_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <vlfplayer.h>

namespace dgc {

class VelodynePlaybackServer {
public:
  VelodynePlaybackServer(IpcInterface *ipc, ParamInterface *pint,
			 VelodyneInterface *velo_int);
  ~VelodynePlaybackServer();
  void Setup(char *filename, int argc, char **argv);
  void PrintPlaybackStats(int line_num, bool paused);
  void ApplanixPoseHandler(ApplanixPose *pose);
  void PlaybackCommandHandler(PlaybackCommand *command);

private:
  void ReadParameters(int argc, char **argv);

  char *cal_filename_;
  char *int_filename_;
  int calibrate_intensities_;
  dgc_transform_t velodyne_offset_;

  IpcInterface *ipc_;
  ParamInterface *pint_;
  VelodyneInterface *velo_interface_;
  vlf_player *player_;
  double playback_speed_;
};

}

#endif
