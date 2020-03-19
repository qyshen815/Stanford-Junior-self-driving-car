#ifndef DGC_CAMERA_PLAYBACK_SERVER_H
#define DGC_CAMERA_PLAYBACK_SERVER_H

#include <ipc_interface.h>
#include <camera_interface.h>
#include <applanix_interface.h>
#include <playback_interface.h>
#include <camplayer.h>

namespace dgc {

class CameraPlaybackServer {
public:
  CameraPlaybackServer(IpcInterface *ipc, CameraInterface *cam_int);
  ~CameraPlaybackServer();
  void Setup(char *filename);
  void PrintPlaybackStats(int line_num, bool paused);
  void ApplanixPoseHandler(ApplanixPose *pose);
  void PlaybackCommandHandler(PlaybackCommand *command);

private:
  IpcInterface *ipc_;
  CameraInterface *camera_interface_;
  cam_player *player;
  double playback_rate_;
};

}

#endif
