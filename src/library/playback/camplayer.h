#ifndef DGC_CAMPLAYER_H
#define DGC_CAMPLAYER_H

#include <camera_interface.h>
#include <data_player.h>
#include <blf.h>

namespace dgc {

class cam_player : public data_player {
 public:
  cam_player(CameraInterface *camint);
  ~cam_player();
  int initialize(char *cam_filename);
  void seek(double t);
  void read_packet(double t, dgc_pose_p pose, double max_age);

 private:
  CameraInterface *camera_interface;
  CameraImage *image;
  blf_index_t *blf_index;
  blf_t *blf;
};

}

#endif
