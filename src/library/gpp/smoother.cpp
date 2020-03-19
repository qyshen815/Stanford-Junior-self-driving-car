#include <roadrunner.h>
#include <cassert>
#include <gls_interface.h>
#include <passat_constants.h>
#include "smoother.h"

using namespace dgc;
using namespace sla;

extern vlr::GlsOverlay *gls;

PathCG::PathCG() 
{
  obstacle_x = NULL;
  obstacle_y = NULL;
  num_obstacles = 0;
}

// Update list of neighbors for given node
void PathCG::updateAllNodeNeighbors(void)
{
  if(num_obstacles == 0)
    return;
  for(int in = 0; in < nodes.length(); in++) 
    updateNodeNeighbors(in);
}

void PathCG::registerObstacles(double *x, double *y, int num_obstacles, 
			       bucket_grid *bgrid)
{
  obstacle_x = x;
  obstacle_y = y;
  this->num_obstacles = num_obstacles;
  this->bgrid = bgrid;
}

// Update list of neighbors for given node
void PathCG::updateNodeNeighbors(const int in) 
{
  double distance, closest_x, closest_y;
  int which;

  bgrid->nearest_neighbor(nodes(in).x.x[0], nodes(in).x.x[1],
			  max_obstacle_dist, &closest_x, &closest_y, 
			  &which, &distance);
  nodes(in).neighbor_index = which;
  nodes(in).neighbor_distance = distance;
}

// returns an array of pointers to the state variables - called once
// at setup of optimizer (REQUIRED)
double **PathCG::getStatePointers(void)
{
  double **x_ptr = new double *[size()];
  for(int i = 2, j = 0, n = nodes.length() - 2; i < n; i++) {
    x_ptr[j++] = &(nodes(i).x(0));
    x_ptr[j++] = &(nodes(i).x(1));
  }
  return x_ptr;
}

// returns an array of pointers to the gradient values - called once
// at setup of optimizer (OPTIONAL)
double **PathCG::getGradientPointers(void) 
{  
  double **g_ptr = new double *[size()];
  for(int i = 2, j = 0, n = nodes.length() - 2; i < n; i++) {
    g_ptr[j++] = &(nodes(i).df_dx(0));
    g_ptr[j++] = &(nodes(i).df_dx(1));
  }
  return g_ptr;
}

// returns the objective (potential) function (REQUIRED)
double PathCG::findPotential(void) 
{
  // compute potential function
  double f = 0.0;
  Vec2d dx;

  // add repulsive potential from barriers
  //  updateAllNodeNeighbors();
  if(num_obstacles > 0) {
    const double dm = max_obstacle_dist;
    for(int i = 1, n = nodes.length() - 1; i < n; i++) {
      int j = nodes(i).neighbor_index;
      if(j != -1) {
	dx.x[0] = nodes(i).x(0) - obstacle_x[j];
	dx.x[1] = nodes(i).x(1) - obstacle_y[j];
	f += obstacle_gain * sqr(std::max(dm - dx.norm(), 0.0));
      }
    }
  }
  
  // add curvature prior 
  for(int i = 1, n = nodes.length() - 1; i < n; i++) 
    f += smoothness_gain * 
      (nodes(i + 1).x - 2.0 * nodes(i).x + nodes(i - 1).x).normSqr();

  // add anchor prior
  for(int i = 2, n = nodes.length(); i < n - 2; i++) {
    dx = nodes(i).x - nodes(i).x0;
    f += midpoint_anchor_gain * dx.normSqr();
  }

  return f;
}

// computes the gradient of the potential (OPTIONAL)
// - if not implemented, omit from class or return NULL
// - if implemented, must also implement getGradientPointers()
void PathCG::findGradient(void) 
{
  Vec2d dx;

  // clear old gradient
  for(int i = 0, n = nodes.length(); i < n; i++) 
    nodes(i).df_dx.set(0.0);
  
  // gradient due to repulsive potential from obstacles
  if(num_obstacles > 0) {
    const double dm = max_obstacle_dist;
    for(int i = 1, n = nodes.length() - 1; i < n; i++) {
      int j = nodes(i).neighbor_index;
      if(j != -1) {
	dx.x[0] = nodes(i).x(0) - obstacle_x[j];
	dx.x[1] = nodes(i).x(1) - obstacle_y[j];
	if(dm - dx.norm() > 0.0) 
	  nodes(i).df_dx -= obstacle_gain * 2.0 * (dm - dx.norm()) * dx / dx.norm();
      }
    }    
  }

  // gradient due to curvature prior 
  for (int i = 1, n = nodes.length() - 1; i < n; i++) {
    Vec2d df_dx = smoothness_gain * 2.0 * 
      (nodes(i + 1).x - 2.0 * nodes(i).x + nodes(i - 1).x);
    if(i != 1)
      nodes(i - 1).df_dx += df_dx;
    nodes(i).df_dx += (-2.0) * df_dx;
    if(i != n - 1)
      nodes(i + 1).df_dx += df_dx;
  }

  for (int i = 2, n = nodes.length() - 2; i < n; i++) {
    dx = nodes(i).x - nodes(i).x0;
    nodes(i).df_dx += midpoint_anchor_gain * 2.0 * dx;
  }
}

PathSmoother::PathSmoother()
{
  // Set some non-standard configuration parameters
  cfg.cg_recall_step_back = 1;
  op.setPrintLevel(0);
  op.setConfiguration(cfg);
}

void PathSmoother::set_gains(double smoothness_gain, double obstacle_gain,
			     double max_obstacle_dist, 
			     double midpoint_anchor_gain)
{
  pl.smoothness_gain = smoothness_gain;
  pl.obstacle_gain = obstacle_gain;
  pl.max_obstacle_dist = max_obstacle_dist;
  pl.midpoint_anchor_gain = midpoint_anchor_gain;
}

void PathSmoother::reverse_path(void)
{
  int i, n;
  Vec2d temp;

  n = pl.nodes.size();
  for(i = 0; i < n / 2; i++) {
    temp = pl.nodes(i).x;
    pl.nodes(i).x = pl.nodes(n - i - 1).x;
    pl.nodes(n - i - 1).x = temp;

    temp = pl.nodes(i).x0;
    pl.nodes(i).x0 = pl.nodes(n - i - 1).x0;
    pl.nodes(n - i - 1).x0 = temp;
  }
}

bool PathSmoother::optimize(gpp_path *path, obstacle_info *obs, 
			    int first_i, bool force_first, double first_x, 
			    double first_y, double first_theta, int last_i, 
			    bool force_last, double last_x, double last_y, 
			    double last_theta, bool reversing, 
			    bool restart)
{
  int i, n;
  double theta_f, theta_b, d;
  Vec2d dx;
  
  //  path->print("smoother input:");

  /* if there aren't enough points to optimize, then skip it */
  if(last_i - first_i + 1 < 5)
    return false;

  /* fill in internal trajectory data structure */
  pl.nodes.setSize(last_i - first_i + 1);


  /* set anchors to A* path, current state to smoothed path */
  for(i = first_i + 2; i <= last_i; i++) {
    pl.nodes(i - first_i).x(0) = path->waypoint[i].smooth_x;
    pl.nodes(i - first_i).x(1) = path->waypoint[i].smooth_y;
    pl.nodes(i - first_i).x0(0) = path->waypoint[i].x;
    pl.nodes(i - first_i).x0(1) = path->waypoint[i].y;
  }

  if(force_first) {
    pl.nodes(0).x0(0) = first_x;
    pl.nodes(0).x0(1) = first_y;
    pl.nodes(0).x = pl.nodes(0).x0;
    d = 1;
    if(reversing) {
      pl.nodes(1).x0(0) = first_x - d * cos(first_theta);
      pl.nodes(1).x0(1) = first_y - d * sin(first_theta);
    }
    else {
      pl.nodes(1).x0(0) = first_x + d * cos(first_theta);
      pl.nodes(1).x0(1) = first_y + d * sin(first_theta);
    }
    pl.nodes(1).x = pl.nodes(1).x0;

    path->waypoint[first_i].smooth_x = pl.nodes(0).x0(0);
    path->waypoint[first_i].smooth_y = pl.nodes(0).x0(1);
    path->waypoint[first_i].smooth_theta = first_theta;
    path->waypoint[first_i + 1].smooth_x = pl.nodes(1).x0(0);
    path->waypoint[first_i + 1].smooth_y = pl.nodes(1).x0(1);
  }
  else {
    /* set the first two anchor points to the smooth path positions */
    for(i = first_i; i <= first_i + 1; i++) {
      pl.nodes(i - first_i).x0(0) = path->waypoint[i].smooth_x;
      pl.nodes(i - first_i).x0(1) = path->waypoint[i].smooth_y;
      pl.nodes(i - first_i).x = pl.nodes(i - first_i).x0;
    }
  }

  n = pl.nodes.size();
  if(force_last) {
    pl.nodes(n - 1).x0(0) = last_x;
    pl.nodes(n - 1).x0(1) = last_y;
    pl.nodes(n - 1).x = pl.nodes(n - 1).x0;

    d = hypot(path->waypoint[last_i].smooth_x - 
	      path->waypoint[last_i - 1].smooth_x,
	      path->waypoint[last_i].smooth_y - 
	      path->waypoint[last_i - 1].smooth_y);

    if(reversing) {
      pl.nodes(n - 2).x0(0) = last_x + d * cos(last_theta);
      pl.nodes(n - 2).x0(1) = last_y + d * sin(last_theta);
    }
    else {
      pl.nodes(n - 2).x0(0) = last_x - d * cos(last_theta);
      pl.nodes(n - 2).x0(1) = last_y - d * sin(last_theta);
    }
    pl.nodes(n - 2).x = pl.nodes(n - 2).x0;
    
    path->waypoint[last_i].smooth_x = pl.nodes(n - 1).x(0);
    path->waypoint[last_i].smooth_y = pl.nodes(n - 1).x(1);
    path->waypoint[last_i].smooth_theta = last_theta;
    path->waypoint[last_i - 1].smooth_x = pl.nodes(n - 2).x(0);
    path->waypoint[last_i - 1].smooth_y = pl.nodes(n - 2).x(1);
    path->waypoint[last_i - 1].smooth_theta = last_theta;
  }

  /* do the conjugate gradient optimization */
  pl.registerObstacles(obs->obstacle_x, obs->obstacle_y, 
		       obs->num_obstacles, obs->bgrid);
  pl.updateAllNodeNeighbors();
  op.setup(pl);
  op.doCG();
  //  op.compareGradients();

  /* extract out the result */
  for(i = first_i + 2; i <= last_i - 2; i++) {
    path->waypoint[i].smooth_x = pl.nodes(i - first_i).x(0);
    path->waypoint[i].smooth_y = pl.nodes(i - first_i).x(1);
  }

  if(restart) {
      theta_f = atan2(path->waypoint[first_i + 2].smooth_y - 
		      path->waypoint[first_i + 1].smooth_y,
		      path->waypoint[first_i + 2].smooth_x - 
		      path->waypoint[first_i + 1].smooth_x);
      if(reversing)
	theta_f += M_PI;
      theta_b = atan2(path->waypoint[first_i + 1].smooth_y - 
		      path->waypoint[first_i].smooth_y,
		      path->waypoint[first_i + 1].smooth_x - 
		      path->waypoint[first_i].smooth_x);
      if(reversing)
	theta_b += M_PI;
      path->waypoint[first_i + 1].smooth_theta = 
	dgc_average_angle(theta_f, theta_b);
  }

  /* compute orientations */
    for(i = first_i + 2; i <= last_i - 1; i++) {
      theta_f = atan2(path->waypoint[i + 1].smooth_y - 
		      path->waypoint[i].smooth_y,
		      path->waypoint[i + 1].smooth_x - 
		      path->waypoint[i].smooth_x);
      //      if(path->waypoint[i + 1].reverse)
      if(reversing)
	theta_f += M_PI;
      theta_b = atan2(path->waypoint[i].smooth_y - 
		      path->waypoint[i - 1].smooth_y,
		      path->waypoint[i].smooth_x - 
		      path->waypoint[i - 1].smooth_x);
      //      if(path->waypoint[i].reverse)
      if(reversing)
	theta_b += M_PI;
      path->waypoint[i].smooth_theta = dgc_average_angle(theta_f, theta_b);
    }

    //  path->print("smoother output:");

  return true;
}


