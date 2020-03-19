#include <roadrunner.h>
#include <cassert>
#include <vector>
#include "interp.h"
#include "gpp_path.h"

using namespace sla;

// returns an array of pointers to the state variables - called once
// at setup of optimizer (REQUIRED)
double **InterpPathCG::getStatePointers(void)
{
  double **x_ptr = new double *[size()];

  for(int i = 0, j = 0, n = nodes.length(); i < n; i++) 
    if(!nodes(i).anchor) {
      x_ptr[j++] = &(nodes(i).x(0));
      x_ptr[j++] = &(nodes(i).x(1));
    }
  return x_ptr;
}

// returns an array of pointers to the gradient values - called once
// at setup of optimizer (OPTIONAL)
double **InterpPathCG::getGradientPointers(void) 
{  
  double **g_ptr = new double *[size()];
 
  for(int i = 0, j = 0, n = nodes.length(); i < n; i++)
    if(!nodes(i).anchor) {
      g_ptr[j++] = &(nodes(i).df_dx(0));
      g_ptr[j++] = &(nodes(i).df_dx(1));
    }
  return g_ptr;
}

// returns the objective (potential) function (REQUIRED)
double InterpPathCG::findPotential(void) 
{
  // compute potential function
  double f = 0.0;

  // add curvature prior 
  for(int i = 1, n = nodes.length() - 1; i < n; i++) 
    f += smoothness_gain * 
      (nodes(i + 1).x - 2.0 * nodes(i).x + nodes(i - 1).x).normSqr();

  return f;
}

// computes the gradient of the potential (OPTIONAL)
// - if not implemented, omit from class or return NULL
// - if implemented, must also implement getGradientPointers()
void InterpPathCG::findGradient(void) 
{
  Vec2d dx;

  // clear old gradient
  for(int i = 0, n = nodes.length(); i < n; i++) 
    nodes(i).df_dx.set(0.0);
  
  // gradient due to curvature prior 
  for(int i = 1, n = nodes.length() - 1; i < n; i++) {
    Vec2d df_dx = smoothness_gain * 2.0 * 
      (nodes(i + 1).x - 2.0 * nodes(i).x + nodes(i - 1).x);
    if(i != 1)
      nodes(i - 1).df_dx += df_dx;
    nodes(i).df_dx += (-2.0) * df_dx;
    if(i != n - 1)
      nodes(i + 1).df_dx += df_dx;
  }
}

PathInterpolator::PathInterpolator()
{
  // Set some non-standard configuration parameters
  cfg.cg_recall_step_back = 1;
  op.setPrintLevel(0);
  op.setConfiguration(cfg);
}

inline int
max_previous_velocity(double d, double *v1, double v2, double a)
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
max_next_velocity(double d, double v1, double *v2, double a)
{
  double new_v2;

  new_v2 = sqrt(dgc_square(v1) + a * d);
  if(*v2 > new_v2 + 0.001) {
    *v2 = new_v2;
    return 1;
  }
  return 0;
}

void PathInterpolator::optimize(interp_path *path, double smoothness_gain)
{
  double theta_b, theta_f;
  int i, n;

  if(path->num_waypoints() < 3)
    return;
  
  if(!path->waypoint[0].fixed || 
     !path->waypoint[path->num_waypoints() - 1].fixed)
    dgc_die("Error: beginning and end waypoints must be fixed.\n");

  pl.smoothness_gain = smoothness_gain;

  pl.nodes.setSize(path->num_waypoints());

  pl.num_variables = 0;
  for(i = 0; i < path->num_waypoints(); i++) {
    pl.nodes(i).x(0) = path->waypoint[i].x;
    pl.nodes(i).x(1) = path->waypoint[i].y;
    if(!path->waypoint[i].fixed)
      pl.num_variables += 2;
    pl.nodes(i).anchor = path->waypoint[i].fixed;
  }

  op.setup(pl);
  op.doCG();

  /* extract positions */
  for(i = 0; i < path->num_waypoints(); i++) {
    path->waypoint[i].x = pl.nodes(i).x(0);
    path->waypoint[i].y = pl.nodes(i).x(1);
  }

  /* compute orientations */
  n = path->num_waypoints();
  for(i = 1; i < n - 1; i++) {
    theta_b = atan2(pl.nodes(i).x(1) - pl.nodes(i - 1).x(1), 
		    pl.nodes(i).x(0) - pl.nodes(i - 1).x(0));
    theta_f = atan2(pl.nodes(i + 1).x(1) - pl.nodes(i).x(1), 
		    pl.nodes(i + 1).x(0) - pl.nodes(i).x(0));
    path->waypoint[i].theta = dgc_average_angle(theta_b, theta_f);
  }
  path->waypoint[0].theta = atan2(pl.nodes(1).x(1) - pl.nodes(0).x(1), 
				  pl.nodes(1).x(0) - pl.nodes(0).x(0));
  path->waypoint[n - 1].theta = 
    atan2(pl.nodes(n - 1).x(1) - pl.nodes(n - 2).x(1), 
	  pl.nodes(n - 1).x(0) - pl.nodes(n - 2).x(0));
  
  /* compute curvature */
  for(i = 1; i < path->num_waypoints() - 1; i++) {
    path->waypoint[i].curvature = 
      dgc_normalize_theta(path->waypoint[i].theta -
			  path->waypoint[i - 1].theta) / 
      hypot(path->waypoint[i].x - path->waypoint[i - 1].x,
	    path->waypoint[i].y - path->waypoint[i - 1].y);
  }
  path->waypoint[0].curvature = path->waypoint[1].curvature;
  path->waypoint[n - 1].curvature = path->waypoint[n - 2].curvature;

#ifdef blah
  /* set blocked waypoints to zero vel */
  for(i = 0; i < path->num_waypoints(); i++)
    if(path->waypoint[i].blocked) {
      j = i - 1;
      d = 0;
      while(j >= 0 && d < 2.0) {
	d += hypot(path->waypoint[j].x - path->waypoint[j + 1].x,
		   path->waypoint[j].y - path->waypoint[j + 1].y);
	path->waypoint[j].v = 0;
	j--;
      }
    }
#endif
}

