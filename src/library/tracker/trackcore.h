#ifndef DGC_TRACKCORE_H
#define DGC_TRACKCORE_H

#include <roadrunner.h>
#include <ipc_interface.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <gls_interface.h>
#include <transform.h>

#include "angle_grid.h"
#include "tracker.h"
#include "sensor_manager.h"
#include "libtracker.h"

extern vlr::GlsOverlay *lasertrack_gls;

typedef struct {

  int     useVelodyne;
  
  int     doTracking;
  int     doGlobalSearch;	

} lasertrack_flags_t;

class LaserTracker {

public:

  LaserTracker(RndfLookup *rl, 
	       SensorManager *sensorManager);
  void update(dgc::PerceptionRobotPose *pose, int sensorId, 
	      void *sensorMsg, lasertrack_flags_t &flags);
  dgc::PerceptionObstacleList *get_msg() {return &msg;}

  void publish_output();
  void makeCombinedMessage();
  void gls_render();
  void drawRndfMask();
  void draw2DCell(const Vec2 &pos, double resolution);
  void make_msg(DynObsMsgList &lst);  

  bool sendMessages;

 private:
  Vec2 localizeOffset;
  RndfLookup *rl;
  int ptsProcessed;
  int useTracker;
  double groundLevel;
  SensorManager *sensorManager;
  Vec2 robotPos;
  dgc::PerceptionObstacleList msg;

};

#endif
