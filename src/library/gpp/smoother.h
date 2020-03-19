#ifndef DGC_SMOOTHER_H
#define DGC_SMOOTHER_H

#include <roadrunner.h>
#include <bucket.h>

#include <arrays.h>
#include <matmath.h>
#include <optimizer.h>
#include <debug_macros.h>

#include <vector>

#include "gpp_path.h"

#define       KD_EPS                      1e-12

class NodeCG {
public:
  sla::Vec2d x, x0, df_dx;

  int neighbor_index;
  double neighbor_distance;
};

class PathCG : public StateContainer<double> {  
public:
  //////////
  // DATA //
  //////////

  sla::Array1 <NodeCG> nodes;

  int num_obstacles;
  bucket_grid *bgrid;
  double *obstacle_x, *obstacle_y;

  double smoothness_gain;
  double obstacle_gain;
  double max_obstacle_dist;
  double midpoint_anchor_gain;

  PathCG();

  ///////////////////////
  // UTILITY FUNCTIONS //
  ///////////////////////

  void registerObstacles(double *x, double *y, int num_obstacles, 
			 bucket_grid *bgrid);
  void updateAllNodeNeighbors(void);
  void updateNodeNeighbors(const int in);

  ///////////////////////////////
  // OPTIMIZER-RELATED METHODS //
  ///////////////////////////////

  // returns the number of state variables (REQUIRED)
  int size(void) { return 2 * (nodes.length() - 4); }

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

class PathSmoother {
public:
  PathSmoother();
  void reverse_path(void);
  bool optimize(gpp_path *path, obstacle_info *obs, int first_i,
		bool force_first, double first_x, double first_y,
		double first_theta, int last_i, bool force_last, 
		double last_x, double last_y, double last_theta,
		bool reversing, bool restart);
  void set_gains(double smoothness_gain, double obstacle_gain,
		 double max_obstacle_dist, double midpoint_anchor_gain);

private:
  PathCG pl;
  OptimizerSettings <double> cfg;
  Optimizer <double> op;
};

#endif
