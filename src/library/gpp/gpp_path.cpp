#include <roadrunner.h>
#include <gls_interface.h>
#include "gpp_path.h"

using namespace dgc;

extern vlr::GlsOverlay *gls;

double perimeter_heading(rndf_waypoint *w2)
{
  rndf_waypoint *w1, *w3;
  double theta1, theta2;
  rndf_zone *zone;

  zone = w2->parentzone();
  w3 = w2->next();
  if(w3 == NULL)
    w3 = zone->perimeter(0);
  w1 = w2->prev();
  if(w1 == NULL)
    w1 = zone->perimeter(zone->num_perimeter_points() - 1);
  theta1 = atan2(w2->utm_y() - w1->utm_y(),
		 w2->utm_x() - w1->utm_x());
  theta2 = atan2(w3->utm_y() - w2->utm_y(),
		 w3->utm_x() - w2->utm_x());
  return dgc_average_angle(theta1, theta2);
}

rndf_perimeter::rndf_perimeter(rndf_waypoint *w1, rndf_waypoint *w2)
{
  double last_forward_x1 = 0, last_forward_y1 = 0;
  double last_backward_x1 = 0, last_backward_y1 = 0;
  double last_forward_x2 = 0, last_forward_y2 = 0;
  double last_backward_x2 = 0, last_backward_y2 = 0, last_theta = 0;
  double x1, y1, x2, y2;
  double d, width, length;
  rndf_waypoint *w;
  int i;

  rndf_point_x = NULL;
  rndf_point_y = NULL;
  rndf_point_type = NULL;
  status = NULL;
  num_points = 0;
  max_points = 0;

  w = w1;
  if(!w->in_lane() || w->parentlane()->width() == 0)
    width = dgc_feet2meters(15.0) / 2.0;
  else
    width = w->parentlane()->width() / 2.0;
  length = 20.0;
  width += DGC_PASSAT_WIDTH / 2.0 + 0;

  d = 0;
  while(d < length && w != NULL) {
    x1 = w->utm_x() + width * cos(w->heading() - M_PI / 2.0);
    y1 = w->utm_y() + width * sin(w->heading() - M_PI / 2.0);
    last_forward_x1 = x1;
    last_forward_y1 = y1;
    last_theta = w->heading();
    if(w->next() != NULL) {
      x2 = w->next()->utm_x() + width * cos(w->next()->heading() - M_PI / 2.0);
      y2 = w->next()->utm_y() + width * sin(w->next()->heading() - M_PI / 2.0);
      add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
			&rndf_point_type, x1, y1, x2, y2, 1, 3);
      d += hypot(x2 - x1, y2 - y1);
      last_forward_x1 = x2;
      last_forward_y1 = y2;
      last_theta = w->next()->heading();
    }
    w = w->next();
  }

  if(d < length) {
    add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		      &rndf_point_type, 
		      last_forward_x1, last_forward_y1,
		      last_forward_x1 + (length - d) * cos(last_theta),
		      last_forward_y1 + (length - d) * sin(last_theta),
		      1, 3);
    last_forward_x1 += (length - d) * cos(last_theta);
    last_forward_y1 += (length - d) * sin(last_theta);
  }

  d = 0;
  w = w1;
  while(d < length && w != NULL) {
    x1 = w->utm_x() + width * cos(w->heading() - M_PI / 2.0);
    y1 = w->utm_y() + width * sin(w->heading() - M_PI / 2.0);
    last_backward_x1 = x1;
    last_backward_y1 = y1;
    last_theta = w->heading();
    if(w->prev() != NULL) {
      x2 = w->prev()->utm_x() + width * cos(w->prev()->heading() - M_PI / 2.0);
      y2 = w->prev()->utm_y() + width * sin(w->prev()->heading() - M_PI / 2.0);
      add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
			&rndf_point_type, x1, y1, x2, y2, 1, 3);
      d += hypot(x2 - x1, y2 - y1);
      last_backward_x1 = x2;
      last_backward_y1 = y2;
      last_theta = w->prev()->heading();
    }
    w = w->prev();
  }

  if(d < length) {
    add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		      &rndf_point_type, 
		      last_backward_x1, last_backward_y1,
		      last_backward_x1 + (length - d) * cos(last_theta + M_PI),
		      last_backward_y1 + (length - d) * sin(last_theta + M_PI),
		      1, 3);
    last_backward_x1 += (length - d) * cos(last_theta + M_PI);
    last_backward_y1 += (length - d) * sin(last_theta + M_PI);
  }

  w = w2;
  if(!w->in_lane() || w->parentlane()->width() == 0)
    width = dgc_feet2meters(15.0) / 2.0;
  else
    width = w->parentlane()->width() / 2.0;
  length = 20.0;
  width += DGC_PASSAT_WIDTH / 2.0 + 0;

  d = 0;
  while(d < length && w != NULL) {
    x1 = w->utm_x() + width * cos(w->heading() - M_PI / 2.0);
    y1 = w->utm_y() + width * sin(w->heading() - M_PI / 2.0);
    last_forward_x2 = x1;
    last_forward_y2 = y1;
    last_theta = w->heading();
    if(w->next() != NULL) {
      x2 = w->next()->utm_x() + width * cos(w->next()->heading() - M_PI / 2.0);
      y2 = w->next()->utm_y() + width * sin(w->next()->heading() - M_PI / 2.0);
      add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
			&rndf_point_type, x1, y1, x2,y2, 1, 3);
      d += hypot(x2 - x1, y2 - y1);
      last_forward_x2 = x2;
      last_forward_y2 = y2;
      last_theta = w->next()->heading();
    }
    w = w->next();
  }

  if(d < length) {
    add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		      &rndf_point_type, 
		      last_forward_x2, last_forward_y2,
		      last_forward_x2 + (length - d) * cos(last_theta),
		      last_forward_y2 + (length - d) * sin(last_theta),
		      1, 3);
    last_forward_x2 += (length - d) * cos(last_theta);
    last_forward_y2 += (length - d) * sin(last_theta);
  }

  d = 0;
  w = w2;
  while(d < length && w != NULL) {
    x1 = w->utm_x() + width * cos(w->heading() - M_PI / 2.0);
    y1 = w->utm_y() + width * sin(w->heading() - M_PI / 2.0);
    last_backward_x2 = x1;
    last_backward_y2 = y1;
    last_theta = w->heading();
    if(w->prev() != NULL) {
      x2 = w->prev()->utm_x() + width * cos(w->prev()->heading() - M_PI / 2.0);
      y2 = w->prev()->utm_y() + width * sin(w->prev()->heading() - M_PI / 2.0);
      add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
			&rndf_point_type, x1, y1, x2, y2, 1, 3);
      d += hypot(x2 - x1, y2 - y1);
      last_backward_x2 = x2;
      last_backward_y2 = y2;
      last_theta = w->prev()->heading();
    }
    w = w->prev();
  }

  if(d < length) {
    add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		      &rndf_point_type, 
		      last_backward_x2, last_backward_y2,
		      last_backward_x2 + (length - d) * cos(last_theta + M_PI),
		      last_backward_y2 + (length - d) * sin(last_theta + M_PI),
		      1, 3);
    last_backward_x2 += (length - d) * cos(last_theta + M_PI);
    last_backward_y2 += (length - d) * sin(last_theta + M_PI);
  }

  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, 
		    last_forward_x1, last_forward_y1,
		    last_backward_x2, last_backward_y2, 1, 3);
  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, 
		    last_forward_x2, last_forward_y2,
		    last_backward_x1, last_backward_y1, 1, 3);
  status = (char *)calloc(num_points, 1);
  dgc_test_alloc(status);

  for(i = 0; i < num_points; i++)
    status[i] = POINT_STATUS_NORMAL;
}

rndf_perimeter& rndf_perimeter::operator=(const rndf_perimeter *src)
{
  int i;

  num_points = 0;
  for(i = 0; i < src->num_points; i++)
    add_obstacle_point(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		       &rndf_point_type, src->rndf_point_x[i],
		       src->rndf_point_y[i], src->rndf_point_type[i]);
  return *this;
}

rndf_perimeter::rndf_perimeter()
{
  rndf_point_x = NULL;
  rndf_point_y = NULL;
  rndf_point_type = NULL;
  status = NULL;
  num_points = 0;
  max_points = 0;
}

rndf_perimeter::rndf_perimeter(double x, double y, double theta,
			       double width, double length)
{
  double ctheta, stheta, x1, y1, x2, y2, x3, y3, x4, y4;
  int i;

  width /= 2;
  length /= 2;
  
  rndf_point_x = NULL;
  rndf_point_y = NULL;
  rndf_point_type = NULL;
  status = NULL;
  num_points = 0;
  max_points = 0;

  ctheta = cos(theta);
  stheta = sin(theta);

  x1 = x + length * ctheta - width * stheta;
  y1 = y + length * stheta + width * ctheta;
  x2 = x + length * ctheta - (-width) * stheta;
  y2 = y + length * stheta + (-width) * ctheta;
  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, x1, y1, x2, y2, 1, 3);

  x3 = x - length * ctheta - (-width) * stheta;
  y3 = y - length * stheta + (-width) * ctheta;
  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, x2, y2, x3, y3, 1, 3);
  x4 = x - length * ctheta - width * stheta;
  y4 = y - length * stheta + width * ctheta;
  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, x3, y3, x4, y4, 1, 3);
  add_obstacle_line(&num_points, &max_points, &rndf_point_x, &rndf_point_y,
		    &rndf_point_type, x4, y4, x1, y1, 1, 3);

  status = (char *)calloc(num_points, 1);
  dgc_test_alloc(status);

  for(i = 0; i < num_points; i++)
    status[i] = POINT_STATUS_NORMAL;
}

rndf_perimeter::rndf_perimeter(rndf_zone *zone)
{
  int i, j, n, k;
  rndf_waypoint *w1, *w2;
  double dx1, dy1, theta1, theta2, dx2, dy2;

  rndf_point_x = NULL;
  rndf_point_y = NULL;
  rndf_point_type = NULL;
  status = NULL;
  num_points = 0;
  max_points = 0;

  n = zone->num_perimeter_points();
  for(j = 0; j < n - 1; j++) {
    w1 = zone->perimeter(j);
    w2 = w1->next();
    theta1 = perimeter_heading(w1);
    theta2 = perimeter_heading(w2);
    dx1 = 2 * cos(theta1 + M_PI / 2.0);
    dy1 = 2 * sin(theta1 + M_PI / 2.0);
    dx2 = 2 * cos(theta2 + M_PI / 2.0);
    dy2 = 2 * sin(theta2 + M_PI / 2.0);
    add_obstacle_line(&num_points, &max_points,
		      &rndf_point_x, &rndf_point_y, &rndf_point_type,
		      w1->utm_x() + dx1, w1->utm_y() + dy1,
		      w2->utm_x() + dx2, w2->utm_y() + dy2, 1.0, 3);
  }
  if(n > 1) {
    w1 = zone->perimeter(n - 1);
    w2 = zone->perimeter(0);
    theta1 = perimeter_heading(w1);
    theta2 = perimeter_heading(w2);
    dx1 = 2 * cos(theta1 + M_PI / 2.0);
    dy1 = 2 * sin(theta1 + M_PI / 2.0);
    dx2 = 2 * cos(theta2 + M_PI / 2.0);
    dy2 = 2 * sin(theta2 + M_PI / 2.0);
    add_obstacle_line(&num_points, &max_points,
		      &rndf_point_x, &rndf_point_y, &rndf_point_type,
		      w1->utm_x() + dx1, w1->utm_y() + dy1,
		      w2->utm_x() + dx2, w2->utm_y() + dy2, 1.0, 3);
  }

  status = (char *)calloc(num_points, 1);
  dgc_test_alloc(status);

  for(i = 0; i < num_points; i++)
    status[i] = POINT_STATUS_NORMAL;

  for(j = 0; j < zone->num_perimeter_points(); j++) {
    w1 = zone->perimeter(j);
    if(w1->num_exits() > 0) {
      for(k = 0; k < num_points; k++)
	if(hypot(rndf_point_x[k] - w1->utm_x(), 
		 rndf_point_y[k] - w1->utm_y()) < 10.0)
	  status[k] = POINT_STATUS_EXIT;
    }
    if(w1->num_entries() > 0) {
      for(k = 0; k < num_points; k++)
	if(hypot(rndf_point_x[k] - w1->utm_x(), 
		 rndf_point_y[k] - w1->utm_y()) < 10.0)
	  status[k] = POINT_STATUS_ENTRANCE;
    }
  }
}

rndf_perimeter::~rndf_perimeter()
{
  free(rndf_point_x);
  free(rndf_point_y);
  free(status);
}

inline int
max_previous_velocity(double d, float *v1, float v2, double a)
{
  double new_v1;

  new_v1 = sqrt(dgc_square(v2) + a * d);
  if(*v1 > new_v1 + 0.001) {
    *v1 = new_v1;
    return 1;
  }
  return 0;
}

inline int
max_next_velocity(double d, float v1, float *v2, double a)
{
  double new_v2;

  new_v2 = sqrt(dgc_square(v1) + a * d);
  if(*v2 > new_v2 + 0.001) {
    *v2 = new_v2;
    return 1;
  }
  return 0;
}

void gpp_path::blank_velocities(double max_v_forward, double max_v_reverse)
{
  int i, j, k;

  if(num_waypoints() == 0)
    return;
  i = 1;
  while(i < num_waypoints() && waypoint[i].reverse == waypoint[0].reverse) 
    i++;
  i--;
  for(j = 0; j <= i; j++)
    if(waypoint[0].reverse) {
      waypoint[j].v = max_v_reverse;
      for(k = 0; k < (int)waypoint[j].inner_pose.size(); k++)
	waypoint[j].inner_pose[k].v = max_v_reverse;
    }
    else {
      waypoint[j].v = max_v_forward;
      for(k = 0; k < (int)waypoint[j].inner_pose.size(); k++)
	waypoint[j].inner_pose[k].v = max_v_forward;
    }
  for(j = i + 1; j < num_waypoints(); j++) {
    waypoint[j].v = 0;
    for(k = 0; k < (int)waypoint[j].inner_pose.size(); k++)
      waypoint[j].inner_pose[k].v = 0;
  }
}

void gpp_path::mark_obstacles(obstacle_info *obs, double min_width_buffer,
			      double min_length_buffer, 
			      double max_width_buffer, double max_length_buffer)
{
  double l = 0, obstacle_score;
  int i;

  /* mark states that intersect obstacles */
  found_obstacle = false;
  for(i = 0; i < num_waypoints(); i++) {
    waypoint[i].blocked = 
      !valid_state(obs, waypoint[i].smooth_x, waypoint[i].smooth_y,
		   waypoint[i].smooth_theta, min_width_buffer, 
		   min_length_buffer, max_width_buffer, max_length_buffer,
		   &obstacle_score);
    if(waypoint[i].blocked && !found_obstacle) {
      obstacle_dist = l;
      found_obstacle = true;
    }
    if(i < num_waypoints() - 1)
      l += hypot(waypoint[i + 1].x - waypoint[i].x,
		 waypoint[i + 1].y - waypoint[i].y);
    if(i == current_i)
      l = 0;
  }
}

bool gpp_path::safe(obstacle_info *obs, double min_width_buffer, 
		    double min_length_buffer, double max_width_buffer,
		    double max_length_buffer)
{
  double obstacle_score;
  int i;

  for(i = 0; i < num_waypoints(); i++) 
    if(!valid_state(obs, waypoint[i].smooth_x, waypoint[i].smooth_y, 
		    waypoint[i].smooth_theta, min_width_buffer, 
		    min_length_buffer, max_width_buffer, max_length_buffer, 
		    &obstacle_score)) 
      return false;
  return true;
}

void gpp_path::advance_path(double current_x, double current_y, 
			    double current_vel)
{
  int i, min_i = 0;
  double d, l = 0, min_d = 0;

  /* find closest point in first path segment */
  for(i = 0; i < num_waypoints() - 1; i++) {
    if(waypoint[i].reverse != waypoint[0].reverse)
      break;
    d = hypot(current_x - waypoint[i].smooth_x,
	      current_y - waypoint[i].smooth_y);
    if(i == 0 || d < min_d) {
      min_d = d;
      min_i = i;
    }
    l += hypot(waypoint[i + 1].smooth_x - waypoint[i].smooth_x,
	       waypoint[i + 1].smooth_y - waypoint[i].smooth_y);
    if(l > 10.0)
      break;
  }

  /* check to see if we are changing direction */
  if(waypoint[min_i + 1].reverse != waypoint[min_i].reverse &&
     fabs(current_vel) < dgc_mph2ms(0.1)) {
    waypoint.erase(waypoint.begin(), waypoint.begin() + min_i);
    if(num_waypoints() > 1)
      waypoint[0].reverse = !waypoint[0].reverse;
    return;
  }
  
  /* go backwards up to 2 meters */
  l = 0;
  i = min_i;
  while(i > 0 && l < 2.0) {
    l += hypot(waypoint[i].smooth_x - waypoint[i - 1].smooth_x,
               waypoint[i].smooth_y - waypoint[i - 1].smooth_y);
    i--;
  }
  
  /* cut off before that */
  if(i > 0)
    waypoint.erase(waypoint.begin(), waypoint.begin() + i);
}

int gpp_path::start_index(double current_x, double current_y, 
			  double holdpath_dist)
{
  double d, min_d = 0, l;
  int i, min_i = 0;
  int keep_going = 0;

  /* find closest point on new path */
  min_i = 0;
  min_d = hypot(current_x - waypoint[min_i].smooth_x, 
                current_y - waypoint[min_i].smooth_y);
  do {
    keep_going = 0;
    if(min_i < num_waypoints() - 1) {
      i = min_i + 1;
      d = hypot(current_x - waypoint[i].smooth_x, 
                current_y - waypoint[i].smooth_y);
      if(d < min_d && waypoint[i].reverse == waypoint[0].reverse) {
        min_d = d;
        min_i = i;
        keep_going = 1;
      }
    }
  } while(keep_going);

  /* hold fixed distance forward */
  i = min_i;
  if(holdpath_dist > 0) {
    l = 0;
    while(l < holdpath_dist && i + 1 < num_waypoints() && 
	  waypoint[i + 1].reverse == waypoint[0].reverse) {
      l += hypot(waypoint[i + 1].x - waypoint[i].x,
		 waypoint[i + 1].y - waypoint[i].y);
      i++;
    }
  }

  if(i == 0)
    i++;
  return i;
}

void gpp_path::print(char *heading)
{
  int i;

  fprintf(stderr, "%s\n", heading);
  for(i = 0; i < num_waypoints(); i++)
    fprintf(stderr, "%d : %d : %.2f %.2f %.2f %.2f %.2f %.2f %d %.2f v %.2f mph\n", i,
	    waypoint[i].reverse,
	    waypoint[i].x, waypoint[i].y, dgc_r2d(waypoint[i].theta),
	    waypoint[i].smooth_x, waypoint[i].smooth_y,
	    dgc_r2d(waypoint[i].smooth_theta), 
	    waypoint[i].action,
	    waypoint[i].cost,
	    dgc_ms2mph(waypoint[i].v));
  fprintf(stderr, "\n");
}

void gpp_path::extract_dgc_trajectory(PlannerTrajectory *plan,
				      double *end_vel,
				      double forward_decel, 
				      double lateral_accel)
{
  int i, j, n, mark, max;
  int first_i, last_i;
  bool backwards, changed;
  double l, v;

  if(num_waypoints() == 0)
    return;

  first_i = 0;
  last_i = 0;
  while(last_i + 1 < num_waypoints() && waypoint[last_i + 1].reverse ==
	waypoint[last_i].reverse)
    last_i++;

  backwards = (waypoint[first_i].reverse);
  plan->reverse = backwards;

  n = 0;
  for(i = first_i; i < last_i; i++)
    n += waypoint[i].inner_pose.size();
  n += 2;

  if(plan->num_waypoints != n) {
    plan->num_waypoints = n;
    plan->waypoint = 
      (PlannerWaypoint *)realloc(plan->waypoint, plan->num_waypoints * 
				 sizeof(PlannerWaypoint));
    dgc_test_alloc(plan->waypoint);
  }

  plan->obstacle_in_view = 0;
  plan->stop_in_view = 0;

  if(found_obstacle) {
    plan->stop_in_view = 1;
    plan->stop_dist = obstacle_dist - 2;
    if(plan->stop_dist < 0)
      plan->stop_dist = 0;
  }

  mark = 0;
  for(i = first_i; i < last_i; i++) {
    if(i < last_i)
      max = (int)waypoint[i].inner_pose.size();
    else
      max = 1;
    for(j = 0; j < max; j++) {
      plan->waypoint[mark].x = waypoint[i].inner_pose[j].x;
      plan->waypoint[mark].y = waypoint[i].inner_pose[j].y;
      plan->waypoint[mark].v = waypoint[i].inner_pose[j].v;

      /* curvature velocity */
      if(waypoint[i].inner_pose[j].curvature != 0 && 
	 fabs(1 / waypoint[i].inner_pose[j].curvature) < 100.0) {
	v = sqrt(lateral_accel / fabs(waypoint[i].inner_pose[j].curvature));
	if(v < plan->waypoint[mark].v) 
	  plan->waypoint[mark].v = v;
      }

#ifdef blah
      /* set blocked waypoints to zero vel */
      if(waypoint[i].blocked) {
	plan->waypoint[mark].v = 0;
	k = mark - 1;
	d = 0;
	while(k >= 0 && d < 2.0) {
	  d += hypot(plan->waypoint[k].x - plan->waypoint[k + 1].x,
		     plan->waypoint[k].y - plan->waypoint[k + 1].y);
	  plan->waypoint[k].v = 0;
	  k--;
	}
      }
#endif      

      plan->waypoint[mark].yaw_rate = 0;
      mark++;
    }
  }

  if(backwards) {
    plan->waypoint[mark].x = waypoint[last_i].smooth_x - 
      0 * cos(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].y = waypoint[last_i].smooth_y - 
      0 * sin(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].v = 0;
    plan->waypoint[mark].yaw_rate = 0;
    mark++;

    plan->waypoint[mark].x = waypoint[last_i].smooth_x - 
      5 * cos(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].y = waypoint[last_i].smooth_y - 
      5 * sin(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].v = 0;
    plan->waypoint[mark].yaw_rate = 0;
    mark++;
    *end_vel = 0;
  }
  else {
    plan->waypoint[mark].x = waypoint[last_i].smooth_x + 
      (0 + DGC_PASSAT_WHEEL_BASE) * cos(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].y = waypoint[last_i].smooth_y + 
      (0 + DGC_PASSAT_WHEEL_BASE) * sin(waypoint[last_i].smooth_theta);
    if(last_i == num_waypoints() - 1 && reached_goal) 
      plan->waypoint[mark].v = *end_vel;
    else {
      plan->waypoint[mark].v = 0;
      *end_vel = 0;
    }
    plan->waypoint[mark].yaw_rate = 0;
    mark++;

    plan->waypoint[mark].x = waypoint[last_i].smooth_x + 
      (5 + DGC_PASSAT_WHEEL_BASE) * cos(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].y = waypoint[last_i].smooth_y + 
      (5 + DGC_PASSAT_WHEEL_BASE) * sin(waypoint[last_i].smooth_theta);
    plan->waypoint[mark].v = 0;
    plan->waypoint[mark].yaw_rate = 0;
    mark++;
  }

  /* backwards/forwards optimization of vlocities */
  n = mark;
  do {
    changed = 0;
    
    for(i = n - 2; i >= 0; i--) {
      l = hypot(plan->waypoint[i + 1].x - plan->waypoint[i].x,
		plan->waypoint[i + 1].y - plan->waypoint[i].y);
      changed |= 
        max_previous_velocity(l, &plan->waypoint[i].v, 
			      plan->waypoint[i + 1].v, forward_decel);
    }
    for(i = 0; i < n - 1; i++) {
      l = hypot(plan->waypoint[i + 1].x - plan->waypoint[i].x,
		plan->waypoint[i + 1].y - plan->waypoint[i].y);
      changed |= 
        max_next_velocity(l, plan->waypoint[i].v, &plan->waypoint[i + 1].v, 
			  forward_decel);
    }
  } while(changed);

  if(backwards) 
    for(i = 0; i < mark; i++)
      plan->waypoint[i].v *= -1;

  for(i = 0; i < plan->num_waypoints - 1; i++) 
    plan->waypoint[i].theta = 
      atan2(plan->waypoint[i + 1].y - plan->waypoint[i].y,
	    plan->waypoint[i + 1].x - plan->waypoint[i].x);
  if(plan->num_waypoints >= 2) 
    plan->waypoint[plan->num_waypoints - 1].theta = 
      plan->waypoint[plan->num_waypoints - 2].theta;

  /*  for(i = 0; i < plan->num_waypoints; i++)
    fprintf(stderr, "%d : %f %f %f %f %f\n", i, plan->waypoint[i].x,
	    plan->waypoint[i].y, dgc_r2d(plan->waypoint[i].theta),
	    plan->waypoint[i].yaw_rate, dgc_ms2mph(plan->waypoint[i].v));*/
}

void gls_draw_robot(vlr::GlsOverlay *gls, double x, double y,
		    double theta, double r)
{
  glsCircle(gls, x - gls->origin_x, y - gls->origin_y, r, 20);
  glsBegin(gls, GLS_LINES);
  glsVertex2f(gls, x - gls->origin_x, y - gls->origin_y);
  glsVertex2f(gls, x + 1.5 * cos(theta) - gls->origin_x,
		  y + 1.5 * sin(theta) - gls->origin_y);
  glsEnd(gls);
}

void gls_draw_path(vlr::GlsOverlay *gls, gpp_path *p)
{
  int i;

  glsLineWidth(gls, 2.0);

  /* draw the A* back axle path */
  glsColor3f(gls, 1, 0, 0);
  glsBegin(gls, GLS_LINE_STRIP);
  for(i = 0; i < p->num_waypoints(); i++) 
    glsVertex2f(gls, p->waypoint[i].x - gls->origin_x,
		    p->waypoint[i].y - gls->origin_y);
  glsEnd(gls);

#ifdef blah
  for(i = 0; i < p->num_waypoints(); i++)
    glsCircle(gls, p->waypoint[i].x - gls->origin_x, 
		  p->waypoint[i].y - gls->origin_y, 0.15, 10);


  /* with orientations */
  glsColor3f(gls, 1, 1, 0);
  glsBegin(gls, GLS_LINES);
  for(i = 0; i < p->num_waypoints(); i++) {
    glsVertex2f(gls, p->waypoint[i].x - gls->origin_x, 
		    p->waypoint[i].y - gls->origin_y);
    glsVertex2f(gls, p->waypoint[i].x + 0.2 * cos(p->waypoint[i].theta) - 
		    gls->origin_x,
		    p->waypoint[i].y + 0.2 * sin(p->waypoint[i].theta) - 
		    gls->origin_y);
  }
  glsEnd(gls);
#endif

#ifdef DRAW_FRONT_AXLES
  /* draw the A* front axle path */
  glsColor3f(gls, 1, 0.4, 0.7);
  glsBegin(gls, GLS_LINE_STRIP);
  for(i = 0; i < p->num_waypoints(); i++) {
    glsVertex3f(gls, p->waypoint[i].x + DGC_PASSAT_WHEEL_BASE * 
		    cos(p->waypoint[i].theta) - gls->origin_x,
		    p->waypoint[i].y + DGC_PASSAT_WHEEL_BASE * 
		    sin(p->waypoint[i].theta) - gls->origin_y, 0.2);
  }
  glsEnd(gls);
#endif

  /* draw the CG smoothed path */
  glsColor3f(gls, 0, 0, 1);
  glsBegin(gls, GLS_LINE_STRIP);
  for(i = 0; i < p->num_waypoints(); i++) 
    glsVertex2f(gls, p->waypoint[i].smooth_x - gls->origin_x,
		    p->waypoint[i].smooth_y - gls->origin_y);
  glsEnd(gls);

  //#ifdef blah
  for(i = 0; i < p->num_waypoints(); i++)
    glsCircle(gls, p->waypoint[i].smooth_x - gls->origin_x,
		  p->waypoint[i].smooth_y - gls->origin_y, 0.1, 10);
  /* with orientations */
  glsColor3f(gls, 1, 1, 0);
  glsBegin(gls, GLS_LINES);
  for(i = 0; i < p->num_waypoints(); i++) {
    glsVertex2f(gls, 
		    p->waypoint[i].smooth_x - gls->origin_x, 
		    p->waypoint[i].smooth_y - gls->origin_y);
    glsVertex2f(gls, 
		    p->waypoint[i].smooth_x + 
		    0.2 * cos(p->waypoint[i].smooth_theta) - 
		    gls->origin_x,
		    p->waypoint[i].smooth_y + 
		    0.2 * sin(p->waypoint[i].smooth_theta) - 
		    gls->origin_y);
  }
  glsEnd(gls);
  //#endif

  //#ifdef DRAW_FRONT_AXLES
  /* draw the CG front axle path */
  glsColor3f(gls, 0, 1, 1);
  glsBegin(gls, GLS_LINE_STRIP);
  for(i = 0; i < p->num_waypoints(); i++) {
    glsVertex3f(gls, 
		    p->waypoint[i].smooth_x + 
		    DGC_PASSAT_WHEEL_BASE * cos(p->waypoint[i].smooth_theta) -
		    gls->origin_x,
		    p->waypoint[i].smooth_y + 
		    DGC_PASSAT_WHEEL_BASE * sin(p->waypoint[i].smooth_theta) - 
		    gls->origin_y, 0.2);
  }
  glsEnd(gls);
  //#endif
}

