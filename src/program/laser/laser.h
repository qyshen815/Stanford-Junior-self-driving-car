#ifndef LASER_H
#define LASER_H

#include "sick.h"

namespace dgc {

class LaserServer {
public:
  LaserServer(IpcInterface *ipc);
  ~LaserServer();
  void Setup(ParamInterface *pint, int argc, char **argv);
  void Shutdown(void);
  void ProcessData(void);

private:
  void ReadParameters(ParamInterface *pint, int argc, char **argv);
  void RegisterIpcMessages(void);
  void PublishLaser(void);

  IpcInterface *ipc_;
  sick_laser_param_t laser_settings_;
  sick_laser_p laser_;
  char modulename_[100];
  
  double start_time_;
  double last_heartbeat_;
  double last_stats_;
};

}

#endif

