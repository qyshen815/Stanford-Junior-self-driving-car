#include <roadrunner.h>
#include <trajectory.h>
#include <planner_messages.h>

using namespace dgc;

PlannerTrajectory *fp_plan_from_pose(dgc_trajectory_p trajectory,
				     double fa_x, double fa_y,
				     int num_waypoints,
				     int *closest_waypoint, 
				     double back_distance)
{
  PlannerTrajectory *plan;
  int i, min_i = 0, mark, mark2;
  double d, min_d = 0;
  
  plan = (PlannerTrajectory *)calloc(1, sizeof(PlannerTrajectory));
  dgc_test_alloc(plan);
  plan->waypoint = (PlannerWaypoint *)calloc(num_waypoints, 
					     sizeof(PlannerWaypoint));
  dgc_test_alloc(plan->waypoint);
  strncpy(plan->host, dgc_hostname(), 10);

  plan->stop_in_view = 0;
  plan->obstacle_in_view = 0;

  plan->num_waypoints = num_waypoints;

  for(i = 0; i < trajectory->num_waypoints; i++) {
    d = dgc_square(trajectory->waypoint[i].x - fa_x) +
      dgc_square(trajectory->waypoint[i].y - fa_y);
    if(i == 0 || d < min_d) {
      min_d = d;
      min_i = i;
    }
  }
  
  min_d = sqrt(min_d);
  mark = min_i;
  *closest_waypoint = mark;

  /* walk backwards for buffer distance */
  d = 0;
  mark2 = mark;
  do {
    mark2 = mark - 1;
    if(mark2 < 0)
      mark2 = trajectory->num_waypoints - 1;
    d += hypot(trajectory->waypoint[mark2].x - trajectory->waypoint[mark].x,
               trajectory->waypoint[mark2].y - trajectory->waypoint[mark].y);
    mark = mark2;
  } while(d < back_distance);

  for(i = 0; i < num_waypoints; i++) {
    plan->waypoint[i].x = trajectory->waypoint[mark].x;
    plan->waypoint[i].y = trajectory->waypoint[mark].y;
    plan->waypoint[i].theta = trajectory->waypoint[mark].theta;
    plan->waypoint[i].v = trajectory->waypoint[mark].velocity;
    plan->waypoint[i].yaw_rate = trajectory->waypoint[mark].yaw_rate;
    mark++;
    if(mark == trajectory->num_waypoints)
      mark = 0;
  }
  plan->timestamp = dgc_get_time();
  return plan;
}

void fp_plan_from_plan(PlannerTrajectory *plan, 
                       dgc_trajectory_p trajectory, double fa_x, double fa_y, 
                       int *closest_waypoint, double back_distance)
{
  double prev_dist, dist, last;
  int i, mark, mark2, forward_min, backward_min;

  prev_dist = hypot(fa_x - trajectory->waypoint[*closest_waypoint].x,
                    fa_y - trajectory->waypoint[*closest_waypoint].y);

  /* walk forward looking for closer waypoints */
  mark = *closest_waypoint;
  dist = prev_dist;
  do {
    mark++;
    if(mark == trajectory->num_waypoints)
      mark = 0;
    last = dist;
    dist = hypot(fa_x - trajectory->waypoint[mark].x, 
                 fa_y - trajectory->waypoint[mark].y);
  } while(dist <= last);
  mark--;
  if(mark < 0)
    mark = trajectory->num_waypoints - 1;
  forward_min = mark;

  /* walk backward looking for closer waypoints */
  mark = *closest_waypoint;
  dist = prev_dist;
  do {
    mark--;
    if(mark < 0)
      mark = trajectory->num_waypoints - 1;
    last = dist;
    dist = hypot(fa_x - trajectory->waypoint[mark].x, 
                 fa_y - trajectory->waypoint[mark].y);
  } while(dist <= last);
  mark++;
  if(mark == trajectory->num_waypoints)
    mark = 0;
  backward_min = mark;

  if(hypot(fa_x - trajectory->waypoint[forward_min].x, 
           fa_y - trajectory->waypoint[forward_min].y) < 
     hypot(fa_x - trajectory->waypoint[backward_min].x, 
           fa_y - trajectory->waypoint[backward_min].y))
    *closest_waypoint = forward_min;
  else
    *closest_waypoint = backward_min;

  /* walk backwards for buffer distance */
  dist = 0;
  mark2 = *closest_waypoint;
  do {
    mark2 = mark - 1;
    if(mark2 < 0)
      mark2 = trajectory->num_waypoints - 1;
    dist += hypot(trajectory->waypoint[mark2].x - 
                  trajectory->waypoint[mark].x,
                  trajectory->waypoint[mark2].y - 
                  trajectory->waypoint[mark].y);
    mark = mark2;
  } while(dist < back_distance);

  plan->stop_in_view = 0;
  plan->obstacle_in_view = 0;

  for(i = 0; i < plan->num_waypoints; i++) {
    plan->waypoint[i].x = trajectory->waypoint[mark].x;
    plan->waypoint[i].y = trajectory->waypoint[mark].y;
    plan->waypoint[i].theta = trajectory->waypoint[mark].theta;
    plan->waypoint[i].v = trajectory->waypoint[mark].velocity;
    plan->waypoint[i].yaw_rate = trajectory->waypoint[mark].yaw_rate;
    mark++;
    if(mark == trajectory->num_waypoints)
      mark = 0;
  }
  plan->timestamp = dgc_get_time();
}

