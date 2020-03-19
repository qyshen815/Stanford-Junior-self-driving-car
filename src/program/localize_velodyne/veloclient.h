#ifndef DGC_VELOCLIENT_H
#define DGC_VELOCLIENT_H

#include <roadrunner.h>
#include <transform.h>
#include <velocore.h>
#include <velo_support.h>
#include <velodyne_interface.h>

using namespace dgc;

class dgc_velodyne_client {
 public:
  dgc_velodyne_client(char *cal_filename, char *int_filename, dgc_transform_t velodyne_offset);
  ~dgc_velodyne_client();

  bool scans_available(void) { return velo_interface->ScanDataWaiting(); }
  void read_spin(dgc_velodyne_spin *spin);

  dgc_velodyne_config_p config;
  VelodyneInterface *velo_interface;
};

#endif
