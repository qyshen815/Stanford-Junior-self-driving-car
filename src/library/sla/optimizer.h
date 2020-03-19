//-*-c++-*-
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

/*
  James R. Diebel
  Stanford University
  
  Started: 26 January 2006

  Optimizer class, accepts datapointers to state variables and
  function pointers to compute the objective function and its
  gradient.
*/

#include <iostream>
#include <list>
#include <cstdlib>
#include <cmath>
#include <fstream>

#define CG_QUADRATIC_LINE_SEARCH 0
#define CG_BACKTRACK_LINE_SEARCH 1
#define CG_HYBRID_LINE_SEARCH 2
#define CG_FILENAME_STRING_LENGTH 200
#define CG_VERY_BIG (T(1e20))

// Forward declarations
template <class T> class Optimizer;

namespace OptimizerUtil { 
  // Barrier penalty function for h(x) > sign(dh)*h0, with a width of influence of |dh|
  // returns a 1/(h-h0)-type barrier penalty that decays to zero at h0+dh.
  // h0 is arbitrary, dh can be negative, h is assumed to be on the correct side 
  // of the barrier. 
  template <class T> inline T barrierPotential(T h, const T h0, T dh) {
    h -= h0;
    if (dh < T(0)) { dh = -dh; h = -h; }
    if (h < T(0)) return CG_VERY_BIG; 
    return (h < dh) ? T(1)/h + h/sqr(dh) - T(2)/dh : T(0);     
  }

  // returns the gradient of the barrier penalty
  // h0 is arbitrary, dh can be negative, h is assumed to be on the correct side 
  // of the barrier. 
  template <class T> inline T barrierGradient(T h, const T h0, T dh) {
    h -= h0; 
    if (dh < T(0)) { dh = -dh; h = -h; }
    if (h < T(0)) return -CG_VERY_BIG; 
    return (h < dh) ? T(-1)/sqr(h) + T(1)/sqr(dh) : T(0);
  }

  // Removes the deadzone from psi, i.e. [0-dpsi] -> 0, [dpsi - inf] -> [0 - inf-dpsi]
  // Takes the abs of psi, so it is guaranteed to be [0-inf] after calling this function
  template <class T> inline void deadZone(T& psi, const T dpsi) { 
    if (psi < T(0)) { 
      psi = (psi > -dpsi) ? T(0) : psi+dpsi;
    } else {
      psi = (psi <  dpsi) ? T(0) : psi-dpsi;
    }
  }
}

// Configuration parameters struct
template <class T>
class OptimizerSettings {
public:
  // CG configuration parameters
  int cg_iter_for_restart; // number of iterations between CG restarts
  int cg_min_iter; // minimum number of iterations
  int cg_max_iter; // maximum number of iterations
  int cg_knock_out_count_down; // number of iteration to perform after tolerances are met
  int cg_recall_step_back; // restore state with memory

  int cg_use_eps_from_step; // use fraction of previous step to set eps
  T cg_min_eps; // minimum allowed epsilon
  T cg_max_eps; // maximum allowed epsilon
  T cg_iter_fraction; // reduction in size per line search iteration
  T cg_step_fraction; // eps set to this fraction of step
  T cg_fd_eps; // eps used for finite difference

  int cg_use_polak_ribiere; // use Polak Ribiere formulation or not

  T cg_rel_tol; // main CG relative tolerance
  T cg_abs_tol; // main CG absolute tolCG_QUADRATIC_LINE_SEARCHerance
  int cg_line_search_type; //  is all for now
  T cg_quadratic_line_search_rel_tol; // quadratic line search tolerance
  int cg_quadratic_line_search_max_iter; // maximum number of line search iter
  T cg_max_step_size; // maximum step size

  int cg_output_state_history; // whether to write state history to disk
  std::string cg_state_history_filename;
  int cg_output_potential_history; // whether to write potential values to disk
  std::string cg_potential_history_filename;

  OptimizerSettings() {
    setDefaultParameters();
  }

  void setDefaultParameters() {
    // CG parameters default values
    cg_min_iter = 0;
    cg_max_iter = 1000;
    cg_knock_out_count_down = 0;
    cg_iter_for_restart = 1000;
    cg_recall_step_back = 0;

    cg_use_eps_from_step = 1;
    cg_min_eps = 1e-4f;
    cg_max_eps = 1e-2f;
    cg_step_fraction = 0.1f;
    cg_iter_fraction = 0.5f;
    cg_fd_eps = 1e-4f;

    cg_use_polak_ribiere = 0;

    cg_rel_tol = T(1e-5);
    cg_abs_tol = T(1e-5);
    cg_line_search_type = CG_QUADRATIC_LINE_SEARCH;
    cg_quadratic_line_search_rel_tol = 1e-3f;
    cg_quadratic_line_search_max_iter = 3;
    cg_max_step_size = T(0);

    cg_output_state_history = 0;
    cg_state_history_filename = "state_history.txt";
    cg_output_potential_history = 0;
    cg_potential_history_filename = "potential_history.txt";
  }
};

// Container class.  Classes that you want to optimize the variables of
// should inherit from this class.
template <class T>
class StateContainer {
public:
  virtual int size() = 0;
  virtual T findPotential() = 0;

  virtual T** getStatePointers() = 0;
  virtual T** getGradientPointers() { return NULL; }

  virtual void findGradient() {}
  virtual void startOfIteration() {}
  virtual void endOfIteration() {}
  virtual void afterMove() {}

  virtual ~StateContainer() {}

  friend class Optimizer<T>;
};

template <class T>
class Optimizer {
public:
  // Constructors
  Optimizer();
  ~Optimizer();

  // Setup and clean-up
  void setup(StateContainer<T>& obj_);
  void deleteAll();

  // Optimization routines
  void doCG();

  // Compare provided gradient to FD gradient
  void compareGradients();

  // I/O routines
  void printStateVector();
  T getLatestPotential() { return f_latest; }
  
  // Parameter control
  void setPrintLevel(const int op_pl_) { op_pl = op_pl_; }
  void setConfiguration(const OptimizerSettings<T>& new_cfg) { cfg = new_cfg; }
  int getCurrentIteration() { return current_iteration; }

  // CG Variables
  int current_iteration; // current iteration counter
  int op_pl; // optimizer print level

  // CG configuration parameters
  OptimizerSettings<T> cfg;

  //protected:
  // Internal configuration routines
  void setDefaultParameters();
  void registerParameters();

  // Internal optimization routines
  T findPotential();
  void findGradient();
  void doFiniteDifference(T fd_eps);
  void doBasicCG();
  T doLineSearch();
  T doBacktrackLineSearch();
  T doQuadraticLineSearch();
  void move(T ds);
  void storeState();
  void recallState();

  // State vector and gradients pointers
  StateContainer<T>* obj; // pointer to container class
  int n; // number of state variables
  T** x; // array of pointers to state variables
  T* x_old; // memory of previous state after move
  T** df_dx; // array of pointers to gradient
  T* df_dx_fd; // array of gradient values if none provided
  int use_finite_difference_grad; // whether finite diff. gradient is used

  // General optimization variables
  T* d; // search direction vector
  T f_latest; // latest value of potential function

  // Some internal variables
  int num_line_search_steps;
  int knock_out_count;
  T max_abs_dir;
  T current_epsilon; // current finite difference step size
};

template <class T>
inline Optimizer<T>::Optimizer() {
  // State and gradient
  n = 0;
  obj = NULL;
  x = NULL;
  x_old = NULL;
  df_dx = NULL;
  df_dx_fd = NULL;
  use_finite_difference_grad = 0;
  
  // Optimization
  d = NULL;
  current_iteration = 0;
  num_line_search_steps = 0;
  knock_out_count = 0;

  // Set default parameter values and register parameters with filer
  setDefaultParameters();
}

template <class T>
inline Optimizer<T>::~Optimizer() {
  deleteAll();
}

template <class T>
inline void Optimizer<T>::deleteAll() {
  if (x) delete [] x;
  if (x_old) delete [] x_old;
  if (df_dx) delete [] df_dx;
  if (df_dx_fd) delete [] df_dx_fd;
  if (d) delete [] d;
  n = 0;
  obj = NULL;
  x = NULL; 
  x_old = NULL; 
  df_dx = NULL;
  df_dx_fd = NULL;
  d = NULL;
  current_iteration = 0;
  num_line_search_steps = 0;
  knock_out_count = 0;
  use_finite_difference_grad = 0;
  if (op_pl>1) std::cout << std::endl << "Optimizer: Freed all memory and reset pointers."
			 << std::flush;
}

template <class T>
inline void Optimizer<T>::setDefaultParameters() {
  op_pl = 0;
}

template <class T>
inline void Optimizer<T>::setup(StateContainer<T>& obj_) {
  // Clear memory if already allocated
  if (n>0) deleteAll();

  // Assign local pointer to passed StateContainer object
  obj = &obj_;

  // Set local pointers to passed values
  n = obj->size();

  // Make sure parameters are valid
  if (n < 2) {
    std::cerr << "At least 2 state variables required!" << std::endl << std::flush;
    n = 0;
    exit(1);
  }

  // Allocate memory for optimization variables
  d = new T[n];
  x_old = new T[n];

  if (op_pl>1) std::cout << std::endl << "Setup Optimizer with " << n 
			 << " state variables." << std::flush;  

  // Get state vector from object
  x = obj->getStatePointers();
  if (!x) {
    std::cerr << "Error, state vector pointers required." << std::endl << std::flush;
    exit(1);
  }
  if (op_pl>3) std::cout << std::endl << "Set state vector." << std::flush;

  // Get gradient vector from object
  df_dx = obj->getGradientPointers();
  if (!df_dx) {
    std::cerr << std::endl << "Warning, no gradient pointer provided, using finite difference." 
	      << std::flush;
    use_finite_difference_grad = 1;
    df_dx_fd = new T[n];
    df_dx = new T*[n];
    for (int i=0;i<n;i++) df_dx[i] = &(df_dx_fd[i]);
  } else if (op_pl>3) std::cout << std::endl << "Set gradient vector." << std::flush;
}

template <class T>
inline void Optimizer<T>::printStateVector() {
  T f0 = findPotential();
  std::cout << std::endl << "x = [";
  for (int i=0;i<n;i++) {
    std::cout << *x[i];
    if (i==n-1) std::cout << "]";
    else std::cout << ", ";
  }
}

template <class T>
inline T Optimizer<T>::findPotential() {
  f_latest = obj->findPotential(); 
  return f_latest;
}

template <class T>
inline void Optimizer<T>::findGradient() {
  if (!use_finite_difference_grad) obj->findGradient();
  else doFiniteDifference(cfg.cg_fd_eps);
}

template <class T>
inline void Optimizer<T>::doFiniteDifference(T fd_eps) {
  if (!df_dx_fd) df_dx_fd = new T[n];
  //T f0 = findPotential();
  for (int i=0;i<n;i++) {
    *x[i] -= fd_eps;
    T f_l = findPotential();
    *x[i] += T(2)*fd_eps;
    T f_r = findPotential();
    df_dx_fd[i] = (f_r - f_l)/(T(2)*fd_eps);
    *x[i] -= fd_eps;
  }
}

template <class T>
inline void Optimizer<T>::compareGradients() {
  double best_std = 1e10; 
  if (use_finite_difference_grad) {
    std::cerr << "Problem, no independent gradient function provided."
	      << std::endl << std::flush;
  }
  if (op_pl > 1) {
    std::cout << std::endl << "Comparing provided gradient to finite difference gradient... "
	      << std::flush;
  }
  printf("\nepsilon, RMS error, max abs diff, min abs diff, med abs diff\n");
  printf("------------------------------------------------------------\n");
  std::list<double> abs_diffs;
  std::list<double> abs_diffs_sort;
  bool output_to_file = false;
  for (T fd_eps=T(1e-1);fd_eps>T(1e-10);fd_eps*=T(0.1)) {
    findPotential(); //Workaround: have to call findPotential before findGradient for the GenerativeModel
    findGradient();
    doFiniteDifference(fd_eps);
    double sum_diffs = 0.0;
    double sum_sqr_diffs = 0.0;
    double max_abs_diff = 0.0;
    double min_abs_diff = 1e12;
    abs_diffs.clear();
    abs_diffs_sort.clear();
    for (int i=0;i<n;i++) {
      double diff = (*df_dx[i] - df_dx_fd[i]);
      if (fabs(diff) > max_abs_diff) max_abs_diff = fabs(diff);
      if (fabs(diff) < min_abs_diff) min_abs_diff = fabs(diff);
      abs_diffs.push_back(fabs(diff));
      abs_diffs_sort.push_back(fabs(diff));
      sum_diffs += diff;
      sum_sqr_diffs += diff*diff;
    }
    abs_diffs_sort.sort();
    std::list<double>::iterator i_diffs = abs_diffs_sort.begin();
    for (int i=0;i<n/2;i++,i_diffs++) {}
    double median = *i_diffs;
    double var = sum_sqr_diffs/double(n) - 
      (sum_diffs/double(n))*(sum_diffs/double(n));
    double std = sqrt(double(var));
    printf("%7.2e, %9.4e, %9.4e, %9.4e, %9.4e\n", 
	   fd_eps, std, max_abs_diff, min_abs_diff, median);
    if (std < best_std) {
      best_std = std; 
      std::ofstream fout("compare_gradients.m");
      fout.precision(16);
      fout << "df_dx = [";
      for (int i=0;i<n;i++) fout << *(df_dx[i]) << " ";
      fout << "];" << std::endl;
      fout << "df_dx_fd = [";
      for (int i=0;i<n;i++) fout << df_dx_fd[i] << " ";
      fout << "];" << std::endl;
      fout.close();
      output_to_file = true;
    }
  }
}

template <class T>
inline void Optimizer<T>::move(T ds) {
  for (int i=0;i<n;i++) *x[i] += ds*d[i];
  obj->afterMove();
}

template <class T>
inline void Optimizer<T>::storeState() {
  for (int i=0;i<n;i++) x_old[i] = *x[i];
}


template <class T>
inline void Optimizer<T>::recallState() {
  for (int i=0;i<n;i++) *x[i] = x_old[i];
}

template <class T>
inline T Optimizer<T>::doLineSearch() {
  T retval;
  
  // find max absolute search direction magnitude
  if (cfg.cg_max_step_size > T(0)) {
    max_abs_dir = fabs(d[0]);
    T abs_dir;
    for (int i=1;i<n;i++) {
      abs_dir = fabs(d[i]); 
      if (abs_dir > max_abs_dir) max_abs_dir = abs_dir;
    }
  }

  switch (cfg.cg_line_search_type) {
  default: case CG_QUADRATIC_LINE_SEARCH: 
    retval = doQuadraticLineSearch(); 
    break;
  case CG_BACKTRACK_LINE_SEARCH: 
    retval = doBacktrackLineSearch();
    break;
  case CG_HYBRID_LINE_SEARCH: 
    T retval = doQuadraticLineSearch();
    if (retval == T(0)) retval = doBacktrackLineSearch();
  }
  return retval;
}

template <class T>
inline T Optimizer<T>::doBacktrackLineSearch() {
  // move along search direction until potential goes up, then backtrack 
  T eps = current_epsilon;
  T total_step = 0.0;
  T f0, f_really_old, f_old, f_new; 
  int line_search_iter = 0;
  f0 = f_really_old = f_old = f_new = f_latest;
  int i=0;
  if (1) do {
      // step forward until we start going up
      do {
	if (cfg.cg_recall_step_back) storeState();
	move(eps);
	total_step += eps;
	f_really_old = f_old;
	f_old = f_new;
	f_new = findPotential();
	i++;
      } while (f_new < f_old);
      // now go back to beginning of interval where minimum might be
      if (f_old > f_new || f_old > f_really_old) {
	std::cerr << "What the fuck!" << std::endl;
	exit(1);
      }
      //printf("f0 = %f, f1 = %f, f2 = %f\n",f_really_old,f_old,f_new);
      f_new = f_old = f_really_old;
      T back = (T(2)*eps >= total_step)? total_step:T(2)*eps;
      if (cfg.cg_recall_step_back) recallState();
      else move(-back);
      total_step -= back;
      // reduce my the step size by half, and try again
      eps *= T(0.5);
      line_search_iter++;
    } while (eps > cfg.cg_min_eps);
  findPotential();
  //std::cout << std::endl << i << ", " << total_step;
  num_line_search_steps += line_search_iter;
  return (f_latest - f0);
}


template <class T>
inline T Optimizer<T>::doQuadraticLineSearch() {
  // bound current_epsilon to not effect a move of greater than cfg.cg_max_step_size
  if (cfg.cg_max_step_size > T(0)) {
    // note that current_epsilon is always positive, so we don't need to use fabs
    if (current_epsilon*max_abs_dir > cfg.cg_max_step_size) current_epsilon = cfg.cg_max_step_size/max_abs_dir;
  }

  // move along search direction, sampling three points and fitting a quadratic
  T eps_0 = current_epsilon;
  T total_step = T(0);
  int line_search_iter = 0;
  T f0, f1, f2, f3;
  do {
    if (cfg.cg_recall_step_back) storeState();
    f0 = f_latest;
    move(current_epsilon); f1 = findPotential();
    move(current_epsilon); f2 = findPotential();
    T a = (f2 - (T)(2.0)*f1 + f0)/((T)(2.0)*current_epsilon*current_epsilon);
    T b = (f2 - f0)/((T)(2.0)*current_epsilon) + (T)(2.0)*a*current_epsilon;
    
    // check curvature and compute optimal step
    T step;
    if (a > 0) { // if curvature is upwards
      step = -b/((T)(2.0)*a); 
    } else { // else don't go anywhere
      step = 0; 
    }
    
    // move optimal step and recompute
    if (cfg.cg_max_step_size > T(0)) {
      if (step*max_abs_dir > cfg.cg_max_step_size) step = cfg.cg_max_step_size/max_abs_dir;
      if (-step*max_abs_dir > cfg.cg_max_step_size) step = -cfg.cg_max_step_size/max_abs_dir;
    }
    move(step);
    f3 = findPotential();
    
    // check to make sure potential is lower, if not, pick best so far
    if (f0 < f3 && f0 < f1 && f0 < f2) {
      if (cfg.cg_recall_step_back) recallState();
      else move(-(step+2*current_epsilon));
      findPotential();
    } else if (f1 < f3 && f1 < f0 && f1 < f2) {
      if (cfg.cg_recall_step_back) {
        recallState();
        move(current_epsilon);
      } else move(-(step+current_epsilon));
      total_step += current_epsilon;
      findPotential();
    } else if (f2 < f3 && f2 < f0 && f2 < f1) {
      if (cfg.cg_recall_step_back) {
        recallState();
        move(current_epsilon);
        move(current_epsilon);
      } else move(-step);
      total_step += T(2)*current_epsilon;
      findPotential();
    } else {
      total_step += step + T(2)*current_epsilon;
    }

    // prepare for next iteration
    current_epsilon *= cfg.cg_iter_fraction;
    if (current_epsilon < cfg.cg_min_eps) current_epsilon = cfg.cg_min_eps;
    if (current_epsilon > cfg.cg_max_eps) current_epsilon = cfg.cg_max_eps;
    if (f0 == T(0)) break;
    line_search_iter++;
  } while ((f0 - f_latest)/f0 > cfg.cg_quadratic_line_search_rel_tol && 
	   line_search_iter < cfg.cg_quadratic_line_search_max_iter);
  
  // compute epsilon for next iteration
  if (cfg.cg_use_eps_from_step) {
    current_epsilon = cfg.cg_step_fraction*total_step;
    if (current_epsilon < cfg.cg_min_eps) current_epsilon = cfg.cg_min_eps;
    if (current_epsilon > cfg.cg_max_eps) current_epsilon = cfg.cg_max_eps;
  } else {
    current_epsilon = eps_0;
  }
  num_line_search_steps += line_search_iter;
  return (f_latest - f0);
}

template <class T>
inline void Optimizer<T>::doCG() {
  if (n==0) {
    std::cerr << "Can't run optimization, no data." << std::endl << std::flush;
    exit(0);
  }
  if (op_pl>1) std::cout << std::endl << "Running Conjugate Gradient Optimization... " << std::flush;
  current_iteration = 0; 
  current_epsilon = cfg.cg_max_eps;
  doBasicCG();
  findPotential();
  if (op_pl>1) std::cout << std::endl << "Done." << std::flush;
}

template <class T>
inline void Optimizer<T>::doBasicCG() {
  // Open file for exporting the state/potential history
  std::ofstream fout[2];
  if (cfg.cg_output_state_history) {
    fout[0].open(cfg.cg_state_history_filename.c_str());
    if (!fout[0].is_open()) {
      std::cerr << "Can't open " << cfg.cg_state_history_filename << " for writing."
		<< " Exiting." << std::endl << std::flush;
      exit(1);
    }
  }
  if (cfg.cg_output_potential_history) {
    fout[1].open(cfg.cg_potential_history_filename.c_str());
    if (!fout[1].is_open()) {
      std::cerr << "Can't open " << cfg.cg_potential_history_filename 
		<< " for writing. Exiting." << std::endl << std::flush;
      exit(1);
    }
  }

  // at the start of the first iteration do callback
  obj->startOfIteration();

  // Set up interally-used CG variables, etc.
  num_line_search_steps = 0;
  int restart_count = 0;
  T delta_0, delta_new, delta_old, delta_d;
  T beta;   
  findPotential();
  findGradient();
  delta_new = 0;
  for (int i=0;i<n;i++) {
    d[i] = -(*df_dx[i]);
    delta_new += d[i]*d[i];
  }
  delta_0 = delta_new;
  T f0 = f_latest;
  T fp, rel_change = T(0);

  // The main iteration
  for (current_iteration = 0; current_iteration < cfg.cg_max_iter; current_iteration++) {
    // do line search
    fp = f_latest;
    doLineSearch();

    // at the end of each iteration do callback
    obj->endOfIteration();

    // check tolerances
    bool knock_out = false;
    if (fp > 0) rel_change = (fp - f_latest)/fp; else rel_change = 0;
    if (rel_change < cfg.cg_rel_tol && current_iteration > cfg.cg_min_iter) knock_out = true;
    if ((fp - f_latest) < cfg.cg_abs_tol && current_iteration > cfg.cg_min_iter) knock_out = true;
    if (knock_out) {
      knock_out_count++;
      if (knock_out_count > cfg.cg_knock_out_count_down) break;
    } else knock_out_count = 0;

    // at the start of each iteration do callback
    obj->startOfIteration();

    // compute new CG parameters for next iteration
    findGradient();
    delta_old = delta_new;
    delta_new = delta_d = 0;
    for (int i=0;i<n;i++) delta_new += (*df_dx[i])*(*df_dx[i]);

    // DD (bail out of iteration if reached local minimum)
    // else can get NaN's in beta and subsequently in nodes
    if(fabs(delta_new) < 1e-30) {
      std::cerr << "DD: found local minimum, exiting iteration" <<
	std::endl << std::flush;
      break;
    }

    beta = delta_new/delta_old;
    for (int i=0;i<n;i++) {
      d[i] = -(*df_dx[i]) + beta*d[i];
      delta_d += -(*df_dx[i])*d[i];
    }
    if (delta_d <= 0 || current_iteration % cfg.cg_iter_for_restart == 0) {
      for (int i=0;i<n;i++) d[i] = -(*df_dx[i]);
      beta = 0;
      restart_count++;
      if (op_pl>3) std::cout << "*";
    }
    
    // output
    if (op_pl>1) {
      printf("\ri=%8d, f=%f, f0=%.4e, df=%.2e, eps=%.3e    ",
	     current_iteration,f_latest,f0,rel_change,current_epsilon);
      std::cout << std::flush;
    }
    if (cfg.cg_output_state_history) {
      fout[0].precision(10);
      for (int i=0;i<n;i++) fout[0] << *x[i] << " ";
      fout[0] << std::endl;
    }
    if (cfg.cg_output_potential_history) {
      fout[1].precision(10);
      fout[1] << f_latest << std::endl;
    }
  }

  // Close export files
  if (cfg.cg_output_state_history) fout[0].close();
  if (cfg.cg_output_potential_history) fout[1].close();

  // Print final statistics, etc.
  if (op_pl>1) {
    T mean_line_search_steps = T(num_line_search_steps)/T(current_iteration+1);
    std::cout.precision(6);
    std::cout << std::endl 
	      << "Basic Non-Linear CG Results: " << std::endl
	      << " - Number of iterations .... " << current_iteration << std::endl
	      << " - Number of restarts ...... " << restart_count << std::endl
	      << " - Mean line search steps .. " << mean_line_search_steps << std::endl
	      << " - Initial potential, fi ... " << f0 << std::endl
	      << " - Final potential, ff ..... " << f_latest << std::endl
	      << " - log10(fi/ff) ............ " << log(f0/f_latest)/log(10.0) << std::endl
	      << " - Final relative change ... " << rel_change*100 << "%";
    if (cfg.cg_output_state_history)
      std::cout << std::endl << " - History exported to ..... " << cfg.cg_state_history_filename
		<< std::flush;
  }
}

  
#endif
