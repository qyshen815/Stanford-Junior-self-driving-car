#ifndef GPP_PATH_H
#define GPP_PATH_H

#include <roadrunner.h>
#include <bucket.h>
#include <vector>
#include <rndf.h>
#include <passat_constants.h>
#include <gls_interface.h>
#include <planner_interface.h>

#define    POINT_STATUS_NORMAL       0
#define    POINT_STATUS_ENTRANCE     1
#define    POINT_STATUS_EXIT         2

#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#define MAX(x,y) (((x) > (y)) ? (x) : (y))

class rndf_perimeter {
 public:
  int num_points, max_points;
  double *rndf_point_x, *rndf_point_y;
  unsigned char *rndf_point_type;
  char *status;
  
  rndf_perimeter();
  rndf_perimeter& operator=(const rndf_perimeter *src);
  rndf_perimeter(dgc::rndf_zone *zone);
  rndf_perimeter(dgc::rndf_waypoint *w1, dgc::rndf_waypoint *w2);
  rndf_perimeter(double x, double y, double theta,
		 double width, double length);
  ~rndf_perimeter();
};

struct obstacle_info {
  int num_obstacles;
  double *obstacle_x, *obstacle_y;
  unsigned char *obstacle_type;
  bucket_grid *bgrid;
};

inline void add_obstacle_point(int *num_points, int *max_points,
			       double **x, double **y, 
			       unsigned char **point_type,
			       double p_x, double p_y, char p_t)
{
  if(*num_points == *max_points) {
    *max_points += 50000;
    *x = (double *)realloc(*x, *max_points * sizeof(double));
    dgc_test_alloc(*x);
    *y = (double *)realloc(*y, *max_points * sizeof(double));
    dgc_test_alloc(*y);
    *point_type = (unsigned char *)realloc(*point_type, *max_points * sizeof(unsigned char));
    dgc_test_alloc(*point_type);
  }
  (*x)[*num_points] = p_x;
  (*y)[*num_points] = p_y;
  (*point_type)[*num_points] = p_t;
  (*num_points)++;
}

inline void add_obstacle_line(int *num_points, int *max_points,
			      double **x, double **y, 
			      unsigned char **point_type,
			      double px1, double py1, 
			      double px2, double py2, double resolution,
			      unsigned char p_t)
{
  int i, n;
  double l, px, py;

  l = hypot(px2 - px1, py2 - py1);
  n = (int)ceil(l / resolution) + 1;
  for(i = 0; i <= n; i++) {
    px = px1 + i / (double)n * (px2 - px1);
    py = py1 + i / (double)n * (py2 - py1);
    add_obstacle_point(num_points, max_points, x, y, point_type, px, py, p_t);
  }
}

struct simple_pose {
  float x, y, theta, curvature, v;
};

struct gpp_pose {
  bool reverse;
  float x, y, theta, v;
  float smooth_x, smooth_y, smooth_theta;
  float cost, curvature;
  int action;
  bool blocked;
  std::vector <simple_pose> inner_pose;
};

class gpp_path {
 public:
  gpp_path() { }
  
  int num_waypoints(void) { return (int)waypoint.size(); }
  void advance_path(double current_x, double current_y, double current_vel);
  int start_index(double current_x, double current_y, 
		  double holdpath_dist);
  bool safe(obstacle_info *obs, double min_width_buffer, 
	    double min_length_buffer, double max_width_buffer, 
	    double max_length_buffer);
  void mark_obstacles(obstacle_info *obs, double min_width_buffer,
		      double min_length_buffer, double max_width_buffer,
		      double max_length_buffer);
  void blank_velocities(double max_v_forward, double max_v_reverse);
  void print(char *heading);

  void extract_dgc_trajectory(dgc::PlannerTrajectory *plan, 
			      double *end_vel, double forward_decel, 
			      double lateral_accel);

  int start_i;
  int current_i;
  //  double dist_to_goal;

  std::vector <gpp_pose> waypoint;
  double total_cost;

  bool found_obstacle;
  double obstacle_dist;

  dgc::rndf_waypoint *goal_rndf_wp;
  bool reached_goal;
  int goal_id;
};

void gls_draw_circle(vlr::GlsOverlay *gls, double x, double y, 
                     double r);

void gls_draw_robot(vlr::GlsOverlay *gls, double x, double y, 
		    double theta, double r);

void gls_draw_path(vlr::GlsOverlay *gls, gpp_path *p);

extern vlr::GlsOverlay *gls;

inline bool point_inside_rect(double x, double y, 
			      double x1, double y1,
			      double x2, double y2, 
			      double w)
{
  double v1x, v1y, v2x, v2y;
  double v2len, temp;
  double perp_dist, parallel_dist;

  v1x = x - x1;
  v1y = y - y1;
  v2x = x2 - x1;
  v2y = y2 - y1;
  v2len = hypot(v2x, v2y);
  parallel_dist = (v1x * v2x + v1y * v2y) / v2len;
  temp = v1x * v1x + v1y * v1y - parallel_dist * parallel_dist;
  if(temp < 0)
    perp_dist = 0;
  else
    perp_dist = sqrt(temp);
  if(parallel_dist >= 0 && parallel_dist <= v2len && perp_dist <= w / 2.0)
    return true;
  return false;
}

inline bool point_inside_poly(double *px, double *py, int N, 
			      double x, double y)
{
  double p1x, p1y, p2x, p2y;
  int counter = 0;
  int i;
  double xinters;

  p1x = px[0];
  p1y = py[0];
  for(i = 1; i <= N; i++) {
    p2x = px[i % N];
    p2y = py[i % N];
    if(y > MIN(p1y, p2y)) 
      if(y <= MAX(p1y, p2y)) 
        if(x <= MAX(p1x, p2x)) 
          if(p1y != p2y) {
            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
            if(p1x == p2x || x <= xinters)
              counter++;
          }
    p1x = p2x;
    p1y = p2y;
  }
  if(counter % 2 != 0)
    return true;
  return false;
}

inline bool valid_state_old(obstacle_info *obs, double x, double y, double theta,
			double width_buffer, double length_buffer,
			bool print = false)
{
  double xc, yc, x1, y1, x2, y2, ctheta, stheta;
  dgc_neighbor_list_p dlist, mark;
  double w, l, d, lf;

  w = DGC_PASSAT_WIDTH + 2 * width_buffer;
  l = DGC_PASSAT_LENGTH + 2 * length_buffer;
  d = 2 * hypot(w / 2, l / 2);

  ctheta = cos(theta);
  stheta = sin(theta);

  lf = DGC_PASSAT_WHEEL_BASE + DGC_PASSAT_FA_TO_BUMPER_DIST + length_buffer;
  x1 = x + lf * ctheta;
  y1 = y + lf * stheta;
  x2 = x + (lf - DGC_PASSAT_LENGTH - 2 * length_buffer) * ctheta;
  y2 = y + (lf - DGC_PASSAT_LENGTH - 2 * length_buffer) * stheta;
  xc = (x1 + x2) / 2.0;
  yc = (y1 + y2) / 2.0;

  dlist = obs->bgrid->range_search(xc, yc, d / 2);
  if(dlist == NULL)
    return true;

  mark = dlist;
  while(mark != NULL) {
    if(point_inside_rect(obs->obstacle_x[mark->id], obs->obstacle_y[mark->id],
			 x1, y1, x2, y2, w)) {
      if(print) {
	fprintf(stderr, "robot %f %f %f\n", x, y, dgc_r2d(theta));
	fprintf(stderr, "point %f %f\n", obs->obstacle_x[mark->id], obs->obstacle_y[mark->id]);
      }
      //      dgc_dlist_free(&dlist);
      dgc_neighbor_list_free(&dlist);
      return false;
    }
    mark = mark->next;
  }

  //  dgc_dlist_free(&dlist);
  dgc_neighbor_list_free(&dlist);
  return true;
}

inline bool valid_state(obstacle_info *obs, double x, double y, double theta,
			double min_width_buffer, double min_length_buffer,
			double max_width_buffer, double max_length_buffer,
			double *obstacle_score)
{
  static double xplarge[4], yplarge[4];
  static double xpsmall[4], ypsmall[4];
  double xc, yc, ctheta, stheta, w2_small, l2_small, w2_large, l2_large;
  dgc_neighbor_list_p dlist, mark;
  double d, lf;

  if(obs == NULL)
    return true;

  w2_small = DGC_PASSAT_WIDTH / 2.0 + min_width_buffer;
  l2_small = DGC_PASSAT_LENGTH / 2.0 + min_length_buffer;
  w2_large = DGC_PASSAT_WIDTH / 2.0 + max_width_buffer;
  l2_large = DGC_PASSAT_LENGTH / 2.0 + max_length_buffer;

  lf = DGC_PASSAT_WHEEL_BASE + DGC_PASSAT_FA_TO_BUMPER_DIST + 
    max_length_buffer - l2_large;

  ctheta = cos(theta);
  stheta = sin(theta);
  xc = x + lf * ctheta;
  yc = y + lf * stheta;

  d = hypot(w2_large, l2_large);
  dlist = obs->bgrid->range_search(xc, yc, d);
  if(dlist == NULL)
    return true;

  xplarge[0] = xc + l2_large * ctheta - w2_large * stheta;
  yplarge[0] = yc + l2_large * stheta + w2_large * ctheta;
  xplarge[1] = xc + l2_large * ctheta + w2_large * stheta;
  yplarge[1] = yc + l2_large * stheta - w2_large * ctheta;
  xplarge[2] = xc - l2_large * ctheta + w2_large * stheta;
  yplarge[2] = yc - l2_large * stheta - w2_large * ctheta;
  xplarge[3] = xc - l2_large * ctheta - w2_large * stheta;
  yplarge[3] = yc - l2_large * stheta + w2_large * ctheta;

  xpsmall[0] = xc + l2_small * ctheta - w2_small * stheta;
  ypsmall[0] = yc + l2_small * stheta + w2_small * ctheta;
  xpsmall[1] = xc + l2_small * ctheta + w2_small * stheta;
  ypsmall[1] = yc + l2_small * stheta - w2_small * ctheta;
  xpsmall[2] = xc - l2_small * ctheta + w2_small * stheta;
  ypsmall[2] = yc - l2_small * stheta - w2_small * ctheta;
  xpsmall[3] = xc - l2_small * ctheta - w2_small * stheta;
  ypsmall[3] = yc - l2_small * stheta + w2_small * ctheta;

  *obstacle_score = 0;
  mark = dlist;
  while(mark != NULL) {
    if(point_inside_poly(xpsmall, ypsmall, 4, 
			 obs->obstacle_x[mark->id], 
			 obs->obstacle_y[mark->id])) {
      dgc_neighbor_list_free(&dlist);
      return false;
    }

    if(point_inside_poly(xplarge, yplarge, 4, 
			 obs->obstacle_x[mark->id], 
			 obs->obstacle_y[mark->id])) {
      (*obstacle_score)++;
    }
    mark = mark->next;
  }
  dgc_neighbor_list_free(&dlist);
  return true;
}

#endif
