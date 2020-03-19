#ifndef DGC_HEURISTIC_H
#define DGC_HEURISTIC_H

#include <roadrunner.h>
#include <vector>

struct hcell {
  bool computed, rev_computed;
  float cost, rev_cost;
  float min_cost, min_rev_cost;
};

class heuristic_table {
 public:
  double min_x, min_y, xy_resolution;
  int x_size, y_size, theta_size;
  float min_h, max_h;
  hcell **data;

  double goal_x, goal_y, goal_theta;
  double ctheta, stheta;

  double theta_resolution;

  heuristic_table(double min_x, double min_y, double xy_resolution, 
		  int x_size, int y_size, int theta_size);
  heuristic_table(char *filename);

  void fill_uncomputed(void);
  void precompute_mins(void);
  void recompute_extrema(void);

  void set_goal(double goal_x, double goal_y, double goal_theta);
  double get_value(double x, double y, double theta, bool reverse);
  ~heuristic_table();
  void save(char *filename);
};

#endif
