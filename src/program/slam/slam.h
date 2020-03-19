#ifndef DGC_SLAM_H
#define DGC_SLAM_H

#include <roadrunner.h>
#include <vector>

typedef struct {
  double smooth_x, smooth_y, smooth_z;
  double utm_x, utm_y, utm_z;
  double fixed_x, fixed_y, fixed_z;
  double timestamp;
  bool use_pose;
} SlamPose;

typedef struct {
  double pose_ts;
  double x, y, z;
} SlamFixedConstraint;

typedef struct {
  double pose1_ts, pose2_ts;
  double dx, dy, dz;
} SlamMatchConstraint;

void OptimizeTrajectories(std::vector <std::vector <SlamPose> > &traj,
			  std::vector <bool> &optimize, 
			  std::vector <SlamMatchConstraint> &match,
			  std::vector <SlamFixedConstraint> &fixed,
			  double odom_std, double gps_std, double match_std);

#endif
