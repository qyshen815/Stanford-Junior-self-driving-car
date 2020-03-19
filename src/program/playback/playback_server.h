#ifndef DGC_PLAYBACK_SERVER_H
#define DGC_PLAYBACK_SERVER_H

#include <ipc_interface.h>
#include <velodyne_interface.h>
#include <ladybug_interface.h>
#include <playback_interface.h>
#include <logio.h>
#include "velodyne_playback_server.h"
#include "ladybug_playback_server.h"

namespace dgc {

class CombinedPlaybackServer {
public:
  CombinedPlaybackServer(IpcInterface *ipc, bool basic, bool use_velodyne,
			 VelodynePlaybackServer *velo_server, bool use_ladybug,
			 LadybugPlaybackServer *lbug_server);
  ~CombinedPlaybackServer();
  void Setup(char *ipc_filename);
  void ProcessData(void);
  void Shutdown(void);
  bool paused(void) { return paused_; };

private:
  void PlaybackCommandHandler(PlaybackCommand *command);
  void PrintPlaybackStats(void);
  int ReadMessage(int message_num, int publish);
  void WaitForTimestamp(double ts);

  IpcInterface *ipc_;
  bool use_velodyne_;
  VelodynePlaybackServer *velo_server_;
  bool use_ladybug_;
  LadybugPlaybackServer *lbug_server_;

  dgc_FILE *infile_;
  bool valid_index_;
  LogfileIndex logfile_index_;
  
  LogReaderCallbackList callbacks_;

  double playback_starttime_, playback_timestamp_, playback_speed_;
  int current_position_, offset_, advance_frame_;
  bool basic_, paused_, hit_eof_;
  LineBuffer *line_buffer_;
  int last_message_num_;
  double last_print_stats_;
};

}

#endif
