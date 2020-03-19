#ifndef DGC_FP_H
#define DGC_FP_H

#include <roadrunner.h>
#include <trajectory.h>
#include <planner_interface.h>

dgc::PlannerTrajectory *fp_plan_from_pose(dgc_trajectory_p trajectory,
					  double fa_x, double fa_y,
					  int num_waypoints,
					  int *closest_waypoint,
					  double back_distance);

void fp_plan_from_plan(dgc::PlannerTrajectory *plan, 
                       dgc_trajectory_p trajectory, double fa_x, double fa_y, 
                       int *closest_waypoint, double back_distance);

#endif
