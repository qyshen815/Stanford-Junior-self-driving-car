#ifndef DGC_VLFPLAYER_H
#define DGC_VLFPLAYER_H

#include <roadrunner.h>
#include <data_player.h>
#include <velocore.h>
#include <velodyne_interface.h>
#include <transform.h>

namespace dgc {

#define       MAX_TIME_DELAY                   1.0
#define       MAX_NUM_SCANS                    20000
#define       MAX_TIME_DIFFERENCE              0.2
#define       VELODYNE_BYTES_PER_SECOND        3100000.0

class vlf_player : public data_player {
public:
  vlf_player(VelodyneInterface *vint);

  ~vlf_player();

  int initialize(char *vlf_filename, char *cal_filename_param, char *int_filename_param,
		 dgc_transform_t velodyne_offset_param);

private:
  void seek(double time);
  void read_packet(double t, dgc_pose_p pose, double max_age);

  dgc_velodyne_file_p       velodyne;
  dgc_velodyne_config_p     config;
  dgc_velodyne_scan_p       scans;
  int                       num_scans;

  VelodyneInterface        *velo_interface;

  double                    first_packet_ts;
  off64_t                   last_packet_fpos;
  int                       last_encoder;
};

}

#endif


