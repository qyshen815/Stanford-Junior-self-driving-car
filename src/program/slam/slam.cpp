#include <roadrunner.h>
#include <arrays.h>
#include <matmath.h>
#include <optimizer.h>
#include <debug_macros.h>
#include <vector>
#include "slam.h"

using namespace std;
using namespace sla;

class NodeCG {
public:
  sla::Vec3d x, df_dx, applanix, delta;
  char do_odom;
  double odom_k;
  double timestamp;
};

class cg_fixed {
 public:
  int pose;
  sla::Vec3d pos;
};

class cg_match {
public:
  int pose1, pose2;
  sla::Vec3d delta;
};

class PathCG : public StateContainer<double> {  
public:
  //////////
  // DATA //
  //////////

  sla::Array1 <NodeCG> nodes;
  std::vector <cg_match> match;
  std::vector <cg_fixed> fixed;
  double match_k;
  double gps_k;

  int *path_size;

  ///////////////////////////////
  // OPTIMIZER-RELATED METHODS //
  ///////////////////////////////

  PathCG() { }

  // returns the number of state variables (REQUIRED)
  int size(void) { return 3 * (nodes.length()); }

  double **getStatePointers(void);
  double **getGradientPointers(void); 
  double findPotential(void);
  void findGradient(void);
  
  // called by optimizer before each full iteration (OPTIONAL)
  void startOfIteration(void) { }
  
  // called by optimizer after each full iteration (OPTIONAL)
  void endOfIteration(void) { }
  
  // called by optimizer after each change to state variables (OPTIONAL)
  void afterMove(void) { }  
};

// returns an array of pointers to the state variables - called once
// at setup of optimizer (REQUIRED)
double **PathCG::getStatePointers(void)
{
  double **x_ptr = new double *[size()];
  for(int i = 0, j = 0, n = nodes.length(); i < n; i++) {
    x_ptr[j++] = &(nodes(i).x(0));
    x_ptr[j++] = &(nodes(i).x(1));
    x_ptr[j++] = &(nodes(i).x(2));
  }
  return x_ptr;
}

// returns an array of pointers to the gradient values - called once
// at setup of optimizer (OPTIONAL)
double **PathCG::getGradientPointers(void) 
{  
  double **g_ptr = new double *[size()];
  for(int i = 0, j = 0, n = nodes.length(); i < n; i++) {
    g_ptr[j++] = &(nodes(i).df_dx(0));
    g_ptr[j++] = &(nodes(i).df_dx(1));
    g_ptr[j++] = &(nodes(i).df_dx(2));
  }
  return g_ptr;
}

// returns the objective (potential) function (REQUIRED)
double PathCG::findPotential(void) 
{
  double f = 0.0;

  // odometry contraints
  for(int i = 0, n = nodes.length(); i < n; i++) 
    if(nodes(i).do_odom)
      f += nodes(i).odom_k * 
	(nodes(i).x - nodes(i - 1).x - nodes(i).delta).normSqr();

  // GPS constraints
  for(int i = 0, n = nodes.length(); i < n; i++) 
    f += gps_k * (nodes(i).x - nodes(i).applanix).normSqr();

  // match constraints
  for(int i = 0, n = (int)match.size(); i < n; i++) 
    f += match_k * (nodes(match[i].pose2).x - 
		    nodes(match[i].pose1).x - match[i].delta).normSqr();

  // fixed constraints (one sided match constraints)
  for(int i = 0, n = (int)fixed.size(); i < n; i++) 
    f += match_k * (nodes(fixed[i].pose).x - fixed[i].pos).normSqr();
  
  return f;
}

// computes the gradient of the potential (OPTIONAL)
// - if not implemented, omit from class or return NULL
// - if implemented, must also implement getGradientPointers()
void PathCG::findGradient(void) 
{
  //  double dx;
  Vec3d dx;

  // clear old gradient
  for(int i = 0, n = nodes.length(); i < n; i++) 
    nodes(i).df_dx.set(0.0);

  // odometry constraint gradient
  for(int i = 0, n = nodes.length(); i < n; i++)
    if(nodes(i).do_odom) {
      dx = nodes(i).odom_k * 2.0 * 
	(nodes(i).x - nodes(i - 1).x - nodes(i).delta);
      nodes(i).df_dx += dx;
      nodes(i - 1).df_dx -= dx;
  }

  // GPS constraint gradient
  for(int i = 0, n = nodes.length(); i < n; i++) 
    nodes(i).df_dx += gps_k * 2 * (nodes(i).x - nodes(i).applanix);

  // match constraints
  for(int i = 0, n = (int)match.size(); i < n; i++) {
    dx = match_k * 2.0 * (nodes(match[i].pose2).x - 
			  nodes(match[i].pose1).x - match[i].delta);
    nodes(match[i].pose2).df_dx += dx;
    nodes(match[i].pose1).df_dx -= dx;
  }

  // fixed constraints (one sided match constraints)
  for(int i = 0, n = (int)fixed.size(); i < n; i++) 
    nodes(fixed[i].pose).df_dx += 
      match_k * 2 * (nodes(fixed[i].pose).x - fixed[i].pos);
}

void OptimizeTrajectories(std::vector <std::vector <SlamPose> > &traj,
			  std::vector <bool> &optimize, 
			  std::vector <SlamMatchConstraint> &match,
			  std::vector <SlamFixedConstraint> &fixed,
			  double odom_std, double gps_std, double match_std)
{
  double offset_x, offset_y, offset_z, last_offset_x = 0, last_offset_y = 0;
  int t_num, i, j, pose, pose1, pose2, mark, num_used, last_i = 0;
  double last_offset_z = 0, d, u;
  vector <SlamPose> *t;
  cg_match m;
  cg_fixed f;
  int count = 0;

  PathCG pl;
  OptimizerSettings <double> cfg;
  Optimizer <double> op;

  // Set some non-standard configuration parameters
  cfg.cg_recall_step_back = 1;
  op.setPrintLevel(0);
  op.setConfiguration(cfg);

  int num_trajectories = (int)traj.size();

  pl.gps_k = 1 / dgc_square(gps_std);
  pl.match_k = 1 / dgc_square(match_std);

  pl.path_size = new int[num_trajectories];

  /* count number of poses to be used in optimization */
  num_used = 0;
  for(t_num = 0; t_num < num_trajectories; t_num++) {
    pl.path_size[t_num] = 0;
    for(i = 0; i < (int)traj[t_num].size(); i++) 
      if(traj[t_num][i].use_pose) {
	num_used++;
	pl.path_size[t_num]++;
      }
  }
  
  pl.nodes.setSize(num_used);
  mark = 0;
  for(t_num = 0; t_num < num_trajectories; t_num++) 
    if(optimize[t_num]) {
      t = &(traj[t_num]);
      count = 0;
      for(i = 0; i < (int)t->size(); i++) 
	if((*t)[i].use_pose) {
	  // initialize poses to smooth coordinates
	  pl.nodes(mark).x(0) = (*t)[i].smooth_x;
	  pl.nodes(mark).x(1) = (*t)[i].smooth_y;
	  pl.nodes(mark).x(2) = (*t)[i].smooth_z;
	  
	  // GPS constraints
	  pl.nodes(mark).applanix(0) = (*t)[i].utm_x;
	  pl.nodes(mark).applanix(1) = (*t)[i].utm_y;
	  pl.nodes(mark).applanix(2) = (*t)[i].utm_z;
	  
	  // odometry constraints 
	  if(count > 0) {
	    pl.nodes(mark).do_odom = 1;
	    pl.nodes(mark).delta = pl.nodes(mark).x - pl.nodes(mark - 1).x;
	    pl.nodes(mark).odom_k = 1 / dgc_square(0.01);
	    
	    d = pl.nodes(mark).delta.norm() * odom_std;
	    if(d < 0.01)
	      d = 0.01;
	    odom_std = 1 / dgc_square(d);
	  }
	  else
	    pl.nodes(mark).do_odom = 0;
	  
	  // pose timestamp
	  pl.nodes(mark).timestamp = (*t)[i].timestamp;
	  
	  mark++;
	  count++;
	}
    }
  
  for(i = 0; i < (int)match.size(); i++) {
    pose1 = -1;
    pose2 = -1;

    for(j = 0; j < num_used; j++)
      if(fabs(pl.nodes(j).timestamp - match[i].pose1_ts) < 0.0001) {
	pose1 = j;
	break;
      }

    for(j = 0; j < num_used; j++)
      if(fabs(pl.nodes(j).timestamp - match[i].pose2_ts) < 0.0001) {
	pose2 = j;
	break;
      }
    
    if(pose1 != -1 && pose2 != -1) {
      m.pose1 = pose1;
      m.pose2 = pose2;
      m.delta(0) = match[i].dx;
      m.delta(1) = match[i].dy;
      m.delta(2) = match[i].dz;
      pl.match.push_back(m);
    }
    else
      dgc_die("Error: match between two unoptimized trajectories. This shouldn't happen.\n");
  }

  for(i = 0; i < (int)fixed.size(); i++) {
    pose = -1;
    for(j = 0; j < num_used; j++) 
      if(fabs(pl.nodes(j).timestamp - fixed[i].pose_ts) < 0.0001) {
	pose = j;
	break;
      }
    
    if(pose != -1) {
      f.pose = pose;
      f.pos(0) = fixed[i].x;
      f.pos(1) = fixed[i].y;
      f.pos(2) = fixed[i].z;
      pl.fixed.push_back(f);
    }
  }

  op.setup(pl);
  op.doCG();

  mark = 0;
  for(t_num = 0; t_num < num_trajectories; t_num++) 
    if(optimize[t_num]) {
      t = &(traj[t_num]);
      count = 0;
      for(i = 0; i < (int)(*t).size(); i++) 
	if((*t)[i].use_pose) {
	  (*t)[i].fixed_x = pl.nodes(mark).x(0);
	  (*t)[i].fixed_y = pl.nodes(mark).x(1);
	  (*t)[i].fixed_z = pl.nodes(mark).x(2);
	  offset_x = (*t)[i].fixed_x - (*t)[i].smooth_x;
	  offset_y = (*t)[i].fixed_y - (*t)[i].smooth_y;
	  offset_z = (*t)[i].fixed_z - (*t)[i].smooth_z;
	  if(count > 0) {
	    for(j = last_i + 1; j < i; j++) {
	      u = (j - last_i) / (double)(i - last_i);
	      (*t)[j].fixed_x = (*t)[j].smooth_x + 
		last_offset_x + u * (offset_x - last_offset_x);
	      (*t)[j].fixed_y = (*t)[j].smooth_y + 
		last_offset_y + u * (offset_y - last_offset_y);
	      (*t)[j].fixed_z = (*t)[j].smooth_z + 
		last_offset_z + u * (offset_z - last_offset_z);
	    }
	  }
	  last_offset_x = offset_x;
	  last_offset_y = offset_y;
	  last_offset_z = offset_z;
	  last_i = i;
	  mark++;
	  count++;
	}
    }
}

