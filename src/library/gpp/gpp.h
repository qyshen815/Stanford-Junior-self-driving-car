#ifndef DGC_GPP_H
#define DGC_GPP_H

#include <roadrunner.h>
#include <vector>
#include <gpp_path.h>
#include <smoother.h>
#include <interp.h>
#include <heuristic.h>
#include <rndf.h>

struct gpp_params {
  double astar_xy_resolution;
  double astar_grid_size;
  double astar_goal_window_deg;

  double astar_width_buffer;
  double astar_length_buffer;

  double pathcheck_width_buffer;
  double pathcheck_length_buffer;
  
  double max_steer_deg;
  bool allow_reverse;
  double reverse_direction_penalty;
  double reverse_travel_penalty;
  
  double max_vel_mph;
  double forward_accel;
  double lateral_accel;
  
  bool use_rndf_perimeter;
  bool use_goal_fence;

  double cg_smoothness_gain;
  double cg_obstacle_gain;
  double cg_max_obstacle_dist;
  double cg_midpoint_anchor_gain;

  bool use_gridmap;

  char *heuristic_filename;
};

class grid_cell {
public:
  float xc, yc, thetac;
  short int xi, yi, thetai;
  char action;
  bool reverse;

  float g, h;
  grid_cell *parent;

  int index;

  bool closed, goal, subgoal;
  char clean;
};

class pri_queue {
public:
  pri_queue();
  int size(void) { return num_heap; }

  void push(grid_cell *cell);
  grid_cell *pop(void);

  void update_priority(int which);
  void update_priority(grid_cell *cell);

  void print(void);

  inline bool empty(void) { return num_heap == 0; }

  bool valid(void);

  void clear(void);

private:
  int capacity(void) { return max_heap; }
  void bubble_up(int which);
  void bubble_down(int which);

  int num_heap, max_heap;
  grid_cell **heap;
};

#define    GOAL_POINT      1
#define    GOAL_LINE       2

struct gpp_single_goal {
  double x, y, theta;
  double xy_slop, theta_slop;
  bool favorite, use_fence, picky;
  dgc::rndf_waypoint *rndf_wp;
};

class gpp_goal {
 public:
  int num_goals(void) { return (int)goal.size(); }
  void add_goal(double x, double y, double theta, 
		double xy_slop, double theta_slop, bool favorite,
		bool use_fence, dgc::rndf_waypoint *w, bool picky);
  void clear_goals(void) { goal.clear(); }
  
  std::vector <gpp_single_goal> goal;
  int id;
};
  
struct theta_action {
  int dx, dy;
  float l;
};

struct action {
  bool reverse;
  int num_poses, max_poses;
  float step_size;
  gpp_pose *pose;
};

#define     GPP_REACHED_GOAL          1
#define     GPP_REACHED_SUBGOAL       2
#define     GPP_REACHED_OTHER         3

class general_planner {
public:
  int x_size(void);
  int y_size(void);
  int theta_size(void);
  
  general_planner(gpp_params *p);
  ~general_planner();
  
  void check(void);

  inline void mark_changed(grid_cell *cell);
  void clear_grid(void);

  double score_path(gpp_path *path);

  void center_grid(double x, double y);
  inline grid_cell *lookup_cell_simple(int xi, int yi, int thetai,
				       bool reverse);
  inline void cell_to_pose(grid_cell *cell, float *x, 
			   float *y, float *theta);
  inline grid_cell *lookup_cell_cont(double x, double y, double theta,
				     bool reverse);

  inline void compute_heuristic(grid_cell *cell);
  inline void apply_action(grid_cell *current, grid_cell **next, 
			   float *l, int i);
  inline double apply_action_cont(float *x, float *y, 
				  float *theta, 
				  int *steps, action *a, 
				  float grid_resolution);

  inline void add_start(double x, double y, double theta,
			bool reverse, int start_action);
  inline void add_goal(double x, double y, double theta, bool reverse);
  inline void add_subgoal(double x, double y, double theta, bool reverse);

  void add_action(float alpha, float d, float action_l, bool reverse = false);

  void plan(double current_x, double current_y, double current_theta, 
	    bool current_reverse, double current_vel, gpp_goal *goal,
	    int use_fine, bool use_smoother, bool breadth_first, 
	    obstacle_info *obs, gpp_path *path, bool *improved, 
	    int pop_limit);

  void extract_path(gpp_path *path, grid_cell *final_goal, 
		    int start_i, double prev_cost, bool *improved);

  void interp_segment(gpp_path *path, int first_i, int last_i, bool reverse);
  void smooth_bidirectional_path(gpp_path *path, int start_i, 
				 obstacle_info *obs, double start_x,
				 double start_y, double start_theta, 
				 double goal_x, double goal_y,
				 double goal_theta, bool use_smoother, 
				 bool restart, bool reached_goal);

  void smooth(double current_x, double current_y, double current_theta, 
	      double current_vel, double goal_x, double goal_y, 
	      double goal_theta, bool use_smoother, 
	      obstacle_info *obs, gpp_path *path);

  void recompute_velocities(gpp_path *path, obstacle_info *obs);

  void create_neighbor_table(void);

  double last_planned_from_x;
  double last_planned_from_y;
  double last_planned_from_theta;

  gpp_params param;

  double theta_resolution_;
  int theta_size_;

  bool reset_planning;

  double width_buffer, length_buffer, check_width_buffer, check_length_buffer;

  int pop_count;
  int plan_status;

private:
  int min_x_, min_y_;

  int xy_size_;
  double xy_resolution_;
  grid_cell **grid, **reverse_grid;
  int standard_xy_size;
  double standard_xy_resolution;
  grid_cell **standard_grid, **standard_reverse_grid;
  double standard_theta_resolution;
  int standard_theta_size;

  int fine_xy_size;
  double fine_xy_resolution;
  grid_cell **fine_grid, **fine_reverse_grid;
  double fine_theta_resolution;
  int fine_theta_size;

  int num_changes, max_changes;
  grid_cell **change_list;

  theta_action *theta_offset, *rev_theta_offset;

  heuristic_table *htable;

  int num_actions, max_actions;
  action **actions;

  bool breadth_first;

  pri_queue open;
  PathSmoother smoother;
  PathInterpolator interpolator;
 };

#endif
