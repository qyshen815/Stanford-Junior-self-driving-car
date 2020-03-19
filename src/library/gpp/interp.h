#ifndef DGC_NEWINTERP_H
#define DGC_NEWINTERP_H

#include <roadrunner.h>

#include <arrays.h>
#include <matmath.h>
#include <optimizer.h>
#include <debug_macros.h>

#include <vector>

#define          KD_EPS              1e-12

struct interp_waypoint {
  double x, y, theta, curvature, v;
  int blocked;
  bool fixed;
};

class interp_path {
public:
  int num_waypoints() { return (int)waypoint.size(); }
  std::vector <interp_waypoint> waypoint;
};

class InterpNodeCG {
public:
  sla::Vec2d x, df_dx;
  bool anchor;
};

class InterpPathCG : public StateContainer<double> {  
public:
  //////////
  // DATA //
  //////////

  int num_variables;
  double smoothness_gain;
  sla::Array1 <InterpNodeCG> nodes;

  ///////////////////////////////
  // OPTIMIZER-RELATED METHODS //
  ///////////////////////////////

  // returns the number of state variables (REQUIRED)
  int size(void) { return num_variables; }

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

class PathInterpolator {
public:
  PathInterpolator();

  void optimize(interp_path *path, double smoothness_gain);

private:
  InterpPathCG pl;
  OptimizerSettings <double> cfg;
  Optimizer <double> op;
};

#endif
