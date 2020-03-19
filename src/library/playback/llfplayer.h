#ifndef DGC_LLFPLAYER_H
#define DGC_LLFPLAYER_H

#include <ladybug_interface.h>
#include <data_player.h>
#include <blf.h>

namespace dgc {

class llf_player : public data_player {
public:
  llf_player(LadybugInterface *lint);

  ~llf_player();

  int initialize(char *llf_filename);

private:
  void seek(double t);
  void read_packet(double t, dgc_pose_p pose, double max_age);

  LadybugInterface *lbug_interface;
  LadybugPacket *pkt;
  blf_t *blf;
  blf_index_t *blf_index;
};

}

#endif
