#ifndef DGC_CAMERA_PLAYBACK_SERVER_H
#define DGC_CAMERA_PLAYBACK_SERVER_H

#include <ipc_interface.h>
#include <ladybug_interface.h>
#include <applanix_interface.h>
#include <playback_interface.h>
#include <llfplayer.h>

namespace dgc {

class LadybugPlaybackServer {
public:
  LadybugPlaybackServer(IpcInterface *ipc, LadybugInterface *lbug_int);
  ~LadybugPlaybackServer();
  void Setup(char *filename);
  void PrintPlaybackStats(int line_num, bool paused);
  void ApplanixPoseHandler(ApplanixPose *pose);
  void PlaybackCommandHandler(PlaybackCommand *command);

private:
  IpcInterface *ipc_;
  LadybugInterface *lbug_interface_;
  llf_player *player_;
  double playback_speed_;
};

}

#endif
