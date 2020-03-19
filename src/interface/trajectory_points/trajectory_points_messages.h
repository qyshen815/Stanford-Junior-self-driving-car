#ifndef TRAJECTORY_POINTS_MESSAGES_H
#define TRAJECTORY_POINTS_MESSAGES_H

#include <global.h>
#include <ipc_interface.h>

namespace vlr {

#define MAX_TRAJECTORY_POINTS 5000

struct TrajectoryPoint2D {
      double t;
      double x;
      double y;
      double theta;
      double kappa;
      double kappa_dot;
      double v;
      double a;
      double jerk;

      double delta_theta; // heading misalignment with center line
      double d;			      // offset to center line
      double a_lat;		    // lateral (to traj not to center line!) acceleration
};

typedef struct {
  int num_points;
  TrajectoryPoint2D* points;
  char host[10];
} TrajectoryPoints2D;

#define                TRAJECTORY_POINTS2D_NAME       "trajectory_points_2d"
#define                TRAJECTORY_POINTS2D_FMT        "{int,<{[double:12]}:1>,[char:10]}"


const dgc::IpcMessageID TrajectoryPoints2DID = {TRAJECTORY_POINTS2D_NAME, TRAJECTORY_POINTS2D_FMT};

} // namespace vlr

#endif
