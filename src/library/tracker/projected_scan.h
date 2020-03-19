#ifndef DGC_PROJECT_IBEO_H
#define DGC_PROJECT_IBEO_H

#include <roadrunner.h>
#include <transform.h>
#include <applanix_interface.h>
#include <perception_interface.h>

#include "vectors.h"

//typedef dgc_perception_static_points_message combined_message_type;

typedef dgc::PerceptionObstacles combined_message_type;


typedef struct {

  bool     use;
  double   x, y, z, r;
  int      status, level;

} projected_point;



class projected_scan {

 public:
  Vec3               robotPos, robotOri, laserPos, laserOri;
  int                num_points;
  projected_point   *point;
  double             timestamp;

  projected_scan();
  ~projected_scan();
  void gls_render();
};


class projected_velodyne: public projected_scan {

 public:
  projected_velodyne(dgc::PerceptionRobotPose *pose,
		     dgc::PerceptionScan *velodyne, 
		     dgc_transform_t velodyne_offset);
  ~projected_velodyne();
};

#endif
