#include <roadrunner.h>
#include <passat_constants.h>
#include <perception_interface.h>
#include <gls_interface.h>
#include <pswrap.h>
#include <rndf.h>
#include "gpp.h"
#include "smoother.h"
#include "interp.h"

#define CONTINUOUS
//#define DRAW

using std::vector;
using namespace dgc;

extern vlr::GlsOverlay *gpp_gls;

void gpp_goal::add_goal(double x, double y, double theta, 
			double xy_slop, double theta_slop, bool favorite,
			bool use_fence, rndf_waypoint *w, bool picky)
{
  gpp_single_goal g;

  if(num_goals() == 0) {
    x = x;
    y = y;
    theta = theta;
  }
  g.x = x;
  g.y = y;
  g.theta = theta;
  g.xy_slop = xy_slop;
  g.theta_slop = theta_slop;
  g.favorite = favorite;
  g.use_fence = use_fence;
  g.rndf_wp = w;
  g.picky = picky;
  goal.push_back(g);
}

/* priority queue implementation */

pri_queue::pri_queue()
{
  num_heap = 0;
  max_heap = 50000;
  heap = (grid_cell **)calloc(max_heap, sizeof(grid_cell *));
  dgc_test_alloc(heap);
}

#define LOWER_PRIORITY(a,b) ((a)->g + (a)->h < (b)->g + (b)->h)

inline bool greaterorequal_priority(grid_cell *a, grid_cell *b)
{
  if(a->g + a->h >= b->g + b->h)
    return true;
  return false;
}

void pri_queue::bubble_up(int which)
{
  int mark, mark2;
  grid_cell *temp, *h2;

  /* make a copy of the starting node */
  temp = heap[which];
  mark = which;
  while(mark > 0) {
    mark2 = (mark - 1) / 2;
    h2 = heap[mark2];
    if(LOWER_PRIORITY(h2, temp))
      break;
    /* copy parent down */
    heap[mark] = h2;
    heap[mark]->index = mark;
    mark = mark2;
  }
  /* place node where bubbling stops */
  heap[mark] = temp;
  heap[mark]->index = mark;
}

void pri_queue::bubble_down(int which)
{
  int mark, mark2;
  grid_cell *temp;

  /* make a copy of the starting node */
  temp = heap[which];

  /* bubble it down until the heap is valid again */
  mark = which;
  while(mark * 2 + 1 < num_heap) {
    /* figure out which is the smallest child */
    mark2 = mark * 2 + 1;
    if(mark2 + 1 < num_heap && 
       LOWER_PRIORITY(heap[mark2 + 1], heap[mark2]))
      mark2++;
    
    if(LOWER_PRIORITY(temp, heap[mark2]))
      break;
    
    /* copy the smaller child up */
    heap[mark] = heap[mark2];
    heap[mark]->index = mark;

    mark = mark2;
  }
  heap[mark] = temp;
  heap[mark]->index = mark;
}

void pri_queue::update_priority(int which)
{
  if(which > 0 && LOWER_PRIORITY(heap[which], heap[(which - 1) / 2]))
    bubble_up(which);
  else
    bubble_down(which);
}

void pri_queue::update_priority(grid_cell *cell)
{
  if(cell->index != -1)
    update_priority(cell->index);
}

void pri_queue::push(grid_cell *cell)
{
  if(num_heap == max_heap) {
    fprintf(stderr, "resizing queue to %d cells\n", capacity() * 2);
    max_heap *= 2;
    heap = (grid_cell **)realloc(heap, max_heap * sizeof(grid_cell *));
    dgc_test_alloc(heap);
  }

  /* add the new element to the end of the heap */
  heap[num_heap] = cell;
  cell->index = num_heap;
  num_heap++;

  /* bubble it up as necessary */
  bubble_up(cell->index);
}

void pri_queue::print(void)
{
  int i;

  fprintf(stderr, "HEAP: ");
  for(i = 0; i < size(); i++) 
    fprintf(stderr, "%.2f ", heap[i]->g + heap[i]->h);
  fprintf(stderr, "\n");
}

void pri_queue::clear(void)
{
  int i;

  for(i = 0; i < num_heap; i++)
    heap[i]->index = -1;
  num_heap = 0;
}

grid_cell *pri_queue::pop(void)
{
  grid_cell *cell;
						
  if(size() == 0) 
    return NULL;
  else {
    /* copy out the top node of the heap */
    cell = heap[0];
    heap[0]->index = -1;

    /* copy the end of the heap to the top of the heap */
    if(size() == 1) 
      num_heap = 0;
    else {
      heap[0] = heap[size() - 1];
      heap[0]->index = 0;
      num_heap--;
      
      /* bubble it down as necessary */
      bubble_down(0);
    }
    return cell;
  }
}

bool pri_queue::valid(void)
{
  int i;

  for(i = 1; i < size(); i++)
    if(greaterorequal_priority(heap[(i - 1) / 2], heap[i])) {
      fprintf(stderr, "node %d greater than node %d\n", (i - 1) / 2, i);
      return false;
    }
  for(i = 0; i < size(); i++)
    if(heap[i]->index != i)
      fprintf(stderr, "node %d has index %d\n", i, heap[i]->index);
  return true;
}

/* A* Implementation */

action *create_action(float alpha, float d, float action_l, 
		      bool reverse = false)
{
  float l;
  gpp_pose p;
  action *a;

  a = new action;
  a->num_poses = 0;
  a->max_poses = 100;
  a->pose = (gpp_pose *)calloc(a->max_poses, sizeof(gpp_pose));
  dgc_test_alloc(a->pose);

  p.x = 0;
  p.y = 0;
  p.theta = 0;
  l = 0;
  if(reverse)
    d *= -1;
  a->step_size = fabs(d);
  while(l <= action_l) {
    l += fabs(d);
    p.x += d * cos(p.theta);
    p.y += d * sin(p.theta);
    p.theta += d * tan(alpha) / DGC_PASSAT_WHEEL_BASE;

    if(a->num_poses == a->max_poses) {
      a->max_poses += 100;
      a->pose = (gpp_pose *)realloc(a->pose, a->max_poses * sizeof(gpp_pose));
      dgc_test_alloc(a->pose);
    }
    a->pose[a->num_poses] = p;
    a->num_poses++;
  }
  a->reverse = reverse;
  return a;
}

void general_planner::add_action(float alpha, float d, float action_l, 
				 bool reverse)
{
  action *a = create_action(alpha, d, action_l, reverse);
  if(num_actions == max_actions) {
    max_actions += 10;
    actions = (action **)realloc(actions, max_actions * sizeof(action *));
    dgc_test_alloc(actions);
  }
  actions[num_actions] = a;
  num_actions++;
}

struct temp_action {
  int dx, dy;
  double theta;
};

void add_temp_action(temp_action *action, int *num_temp_actions,
		     int dx, int dy)
{
  action[*num_temp_actions].dx = dx;
  action[*num_temp_actions].dy = dy;
  action[*num_temp_actions].theta = atan2(dy, dx);
  (*num_temp_actions)++;
}

void general_planner::create_neighbor_table(void)
{
  temp_action taction[20];
  int num_taction = 0;
  int i, j, min_j = 0;
  double theta, dtheta, min_dtheta = 0;

  add_temp_action(taction, &num_taction, 1, 0);
  add_temp_action(taction, &num_taction, 1, 1);
  add_temp_action(taction, &num_taction, 0, 1);
  add_temp_action(taction, &num_taction, -1, 1);
  add_temp_action(taction, &num_taction, -1, 0);
  add_temp_action(taction, &num_taction, -1, -1);
  add_temp_action(taction, &num_taction, 0, -1);
  add_temp_action(taction, &num_taction, 1, -1);

  add_temp_action(taction, &num_taction, 2, 1);
  add_temp_action(taction, &num_taction, 1, 2);
  add_temp_action(taction, &num_taction, -1, 2);
  add_temp_action(taction, &num_taction, -2, 1);
  add_temp_action(taction, &num_taction, -2, -1);
  add_temp_action(taction, &num_taction, -1, -2);
  add_temp_action(taction, &num_taction, 1, -2);
  add_temp_action(taction, &num_taction, 2, -1);
  
  theta_offset = (theta_action *)calloc(theta_size_, sizeof(theta_action));
  dgc_test_alloc(theta_offset);

  rev_theta_offset = (theta_action *)calloc(theta_size_, sizeof(theta_action));
  dgc_test_alloc(rev_theta_offset);

  for(i = 0; i < theta_size_; i++) {
    theta = (i * theta_resolution_ - M_PI);
    for(j = 0; j < num_taction; j++) {
      dtheta = fabs(dgc_normalize_theta(theta - taction[j].theta));
      if(j == 0 || dtheta < min_dtheta) {
	min_dtheta = dtheta;
	min_j = j;
      }
    }
    theta_offset[i].dx = taction[min_j].dx;
    theta_offset[i].dy = taction[min_j].dy;
    theta_offset[i].l = hypot(theta_offset[i].dx, theta_offset[i].dy);
  }

  for(i = 0; i < theta_size_; i++) {
    theta = (i * theta_resolution_ - M_PI) + M_PI;
    for(j = 0; j < num_taction; j++) {
      dtheta = fabs(dgc_normalize_theta(theta - taction[j].theta));
      if(j == 0 || dtheta < min_dtheta) {
	min_dtheta = dtheta;
	min_j = j;
      }
    }
    rev_theta_offset[i].dx = taction[min_j].dx;
    rev_theta_offset[i].dy = taction[min_j].dy;
    rev_theta_offset[i].l = hypot(rev_theta_offset[i].dx, 
				  rev_theta_offset[i].dy);
  }

  num_actions = 0;
  max_actions = 10;
  actions = (action **)calloc(max_actions, sizeof(action *));
  dgc_test_alloc(actions);
}

void allocate_grid(int xy_size, int theta_size, double theta_resolution,
		   grid_cell ***grid, grid_cell ***reverse_grid)
{
  double grid_mem, queue_mem;
  int i, x, y, z;

  fprintf(stderr, "Theta resolution = %.2f deg\n", dgc_r2d(theta_resolution));
  fprintf(stderr, "Sizeof grid cell %d bytes\n", (int)sizeof(grid_cell));
  grid_mem = (xy_size * xy_size * sizeof(grid_cell *) +
	      xy_size * xy_size * theta_size * 
	      sizeof(grid_cell)) * 2.0 / 1024.0 / 1024.0;
  queue_mem = (xy_size * xy_size * theta_size * 
	       2 * sizeof(grid_cell *)) / 1024.0 / 1024.0;
  fprintf(stderr, "Grid memory required : %.2f MB\n", grid_mem);
  fprintf(stderr, "Queue memory required : %.2f MB\n", queue_mem);
  fprintf(stderr, "Total required : %.2f MB\n", grid_mem + queue_mem);

  (*grid) = (grid_cell **)calloc(xy_size * xy_size, sizeof(grid_cell *));
  dgc_test_alloc(*grid);
  
  for(i = 0; i < xy_size * xy_size; i++) {
    (*grid)[i] = (grid_cell *)calloc(theta_size, sizeof(grid_cell));
    dgc_test_alloc((*grid)[i]);
  }

  (*reverse_grid) = (grid_cell **)calloc(xy_size * xy_size, sizeof(grid_cell *));
  dgc_test_alloc(*reverse_grid);
  
  for(i = 0; i < xy_size * xy_size; i++) {
    (*reverse_grid)[i] = (grid_cell *)calloc(theta_size, sizeof(grid_cell));
    dgc_test_alloc((*reverse_grid)[i]);
  }

  for(x = 0; x < xy_size; x++)
    for(y = 0; y < xy_size; y++)
      for(z = 0; z < theta_size; z++) {
	(*grid)[y * xy_size + x][z].index = -1;
	(*reverse_grid)[y * xy_size + x][z].index = -1;

	(*grid)[y * xy_size + x][z].xi = x;
	(*reverse_grid)[y * xy_size + x][z].xi = x;

	(*grid)[y * xy_size + x][z].yi = y;
	(*reverse_grid)[y * xy_size + x][z].yi = y;

	(*grid)[y * xy_size + x][z].thetai = z;
	(*reverse_grid)[y * xy_size + x][z].thetai = z;

	(*grid)[y * xy_size + x][z].reverse = false;
	(*reverse_grid)[y * xy_size + x][z].reverse = true;
      }
}

general_planner::general_planner(gpp_params *p)
{
  double theta_per_cell;

  param = *p;

  standard_xy_resolution = p->astar_xy_resolution;
  standard_xy_size = (int)ceil(p->astar_grid_size / standard_xy_resolution);
  theta_per_cell = standard_xy_resolution * 
    tan(dgc_d2r(param.max_steer_deg) / DGC_PASSAT_STEERING_RATIO) / 
    DGC_PASSAT_WHEEL_BASE;
  standard_theta_size = (int)ceil(M_PI * 2 / theta_per_cell);
  standard_theta_resolution = 2 * M_PI / standard_theta_size;

  allocate_grid(standard_xy_size, standard_theta_size, 
		standard_theta_resolution, &standard_grid, 
		&standard_reverse_grid);

  xy_size_ = standard_xy_size;
  xy_resolution_ = standard_xy_resolution;
  theta_size_ = standard_theta_size;
  theta_resolution_ = standard_theta_resolution;
  grid = standard_grid;
  reverse_grid = standard_reverse_grid;

  fine_xy_resolution = 0.25;
  fine_xy_size = (int)ceil(30.0 / fine_xy_resolution);
  theta_per_cell = fine_xy_resolution * 
    tan(dgc_d2r(param.max_steer_deg) / DGC_PASSAT_STEERING_RATIO) / 
    DGC_PASSAT_WHEEL_BASE;
  fine_theta_size = (int)ceil(M_PI * 2 / theta_per_cell);
  fine_theta_resolution = 2 * M_PI / fine_theta_size;

  allocate_grid(fine_xy_size, fine_theta_size, 
		fine_theta_resolution, &fine_grid, 
		&fine_reverse_grid);

  breadth_first = false;

  smoother.set_gains(p->cg_smoothness_gain, p->cg_obstacle_gain, 
		     p->cg_max_obstacle_dist, p->cg_midpoint_anchor_gain);

  center_grid(0, 0);

  num_changes = 0;
  max_changes = 10000;
  change_list = (grid_cell **)calloc(max_changes, sizeof(grid_cell *));
  dgc_test_alloc(change_list);

  create_neighbor_table();

  htable = new heuristic_table(param.heuristic_filename);

  reset_planning = false;
}

general_planner::~general_planner()
{
  int i;

  for(i = 0; i < xy_size_ * xy_size_; i++) 
    free(grid[i]);
  free(grid);
}

inline void general_planner::mark_changed(grid_cell *cell)
{
  if(num_changes == max_changes) {
    max_changes += 10000;
    change_list = 
      (grid_cell **)realloc(change_list, max_changes * sizeof(grid_cell *));
    dgc_test_alloc(change_list);
  }
  change_list[num_changes] = cell;
  num_changes++;
}

void general_planner::clear_grid(void)
{
  int i;
  
  for(i = 0; i < num_changes; i++) {
    change_list[i]->closed = false;
    change_list[i]->goal = false;
    change_list[i]->subgoal = false;
  }
  num_changes = 0;
}

void general_planner::center_grid(double x, double y)
{
  min_x_ = (int)floor(x / xy_resolution_) - xy_size_ / 2;
  min_y_ = (int)floor(y / xy_resolution_) - xy_size_ / 2;
}

inline grid_cell *general_planner::lookup_cell_simple(int xi, int yi,
						      int thetai,
						      bool reverse)
{
  int i;

  if(xi < 0 || yi < 0 || xi >= xy_size_ || yi >= xy_size_ || 
     thetai < 0 || thetai >= theta_size_)
    return NULL;
  i = yi * xy_size_ + xi;
  if(reverse) 
    return reverse_grid[i] + thetai;
  else 
    return grid[i] + thetai;
}

inline grid_cell *general_planner::lookup_cell_cont(double x, double y, 
						    double theta, bool reverse)
{
  int xi, yi, thetai, i;

  xi = (int)floor(x / xy_resolution_) - min_x_;
  yi = (int)floor(y / xy_resolution_) - min_y_;
  
  if(xi < 0 || yi < 0 || xi >= xy_size_ || yi >= xy_size_)
    return NULL;
  i = yi * xy_size_ + xi;

  thetai = (int)rint((dgc_normalize_theta(theta) + M_PI) / 
		     (2 * M_PI) * theta_size_);
  if(thetai == theta_size_)
    thetai = 0;

  if(reverse) 
    return reverse_grid[i] + thetai;
  else 
    return grid[i] + thetai;
}

inline void general_planner::apply_action(grid_cell *current, 
					  grid_cell **next, float *l, int i)
{
  bool rev = false;
  int xi, yi, thetai, dx, dy;
  float cost;

  if(i >= 3)
    rev = true;

  thetai = current->thetai;
  if(i == 1 || i == 4) 
    thetai--;
  else if(i == 2 || i == 5) 
    thetai++;
  if(thetai < 0)
    thetai += theta_size_;
  if(thetai >= theta_size_)
    thetai -= theta_size_;

  if(rev) {
    dx = rev_theta_offset[thetai].dx;
    dy = rev_theta_offset[thetai].dy;
    cost = rev_theta_offset[thetai].l * param.reverse_travel_penalty;
  }
  else {
    dx = theta_offset[thetai].dx;
    dy = theta_offset[thetai].dy;
    cost = theta_offset[thetai].l;
  }
  xi = current->xi + dx;
  yi = current->yi + dy;

  *next = lookup_cell_simple(xi, yi, thetai, rev);
  *l = cost;
}

inline void general_planner::cell_to_pose(grid_cell *cell, float *x, 
					  float *y, float *theta)
{
  *x = (cell->xi + min_x_ + 0.5) * xy_resolution_;
  *y = (cell->yi + min_y_ + 0.5) * xy_resolution_;
  *theta = (cell->thetai * theta_resolution_ - M_PI);
}

inline void general_planner::compute_heuristic(grid_cell *cell)
{
  float x, y, theta;

  if(breadth_first) {
    cell->h = 0;
    return;
  }
  cell_to_pose(cell, &x, &y, &theta);
  cell->h = htable->get_value(x, y, theta, cell->reverse);
}

void general_planner::extract_path(gpp_path *path, grid_cell *final_goal, 
				   int start_i, double prev_cost, 
				   bool *improved)
{
  vector <grid_cell *> temp_path;
  double new_cost = 0;
  gpp_pose pose;
  grid_cell *p;
  int i, n;

  p = final_goal;
  if(p != NULL)
    path->total_cost = p->g;
  while(p != NULL) {
    temp_path.push_back(p);
    p = p->parent;
  }
    
  n = (int)temp_path.size();
  for(i = 0; i < n; i++) {
#ifdef CONTINUOUS
    pose.x = temp_path[n - i - 1]->xc;
    pose.y = temp_path[n - i - 1]->yc;
    pose.theta = temp_path[n - i - 1]->thetac;
    //    fprintf(stderr, "pose %f %f %f\n", pose.x, pose.y, dgc_r2d(pose.theta));
#else
    cell_to_pose(temp_path[n - i - 1], &pose.x, &pose.y, &pose.theta);
#endif
    pose.smooth_x = pose.x;
    pose.smooth_y = pose.y;
    pose.smooth_theta = pose.theta;

    pose.reverse = temp_path[n - i - 1]->reverse;
    pose.v = 0;
    pose.action = temp_path[n - i -1]->action;
    pose.cost = temp_path[n - i - 1]->g;
    if(path->num_waypoints() == 0 || i != 0)
      path->waypoint.push_back(pose);

  }

  if(start_i == -1) 
    *improved = false;
  else {
    new_cost = path->total_cost;
    *improved = (new_cost < prev_cost);
  }
}

inline double interp(double min, double max, double frac)
{
  return min + frac * (max - min);
}

void general_planner::interp_segment(gpp_path *path, int first_i,
				     int last_i, bool reverse)
{
  interp_path t;
  interp_waypoint p;
  double x1, y1, x2, y2;
  int i, j, mark, n, N = 5;
  double frac;
  simple_pose p2;
  bool seg_begin;

  if(last_i == first_i)
    return;

  seg_begin = (path->waypoint[first_i].reverse != reverse || first_i == 0);

  if(!seg_begin) {
    n = path->waypoint[first_i - 1].inner_pose.size();
    p.x = path->waypoint[first_i - 1].inner_pose[n - 2].x;
    p.y = path->waypoint[first_i - 1].inner_pose[n - 2].y;
    p.v = path->waypoint[first_i - 1].v;
    p.fixed = true;
    p.blocked = false;
    t.waypoint.push_back(p);
    p.x = path->waypoint[first_i - 1].inner_pose[n - 1].x;
    p.y = path->waypoint[first_i - 1].inner_pose[n - 1].y;
    p.v = path->waypoint[first_i - 1].v;
    p.fixed = true;
    p.blocked = false;
    t.waypoint.push_back(p);
  }

  for(i = first_i; i <= last_i; i++) {
    x1 = path->waypoint[i].smooth_x;
    y1 = path->waypoint[i].smooth_y;
    if(!reverse) {
      x1 += DGC_PASSAT_WHEEL_BASE * cos(path->waypoint[i].smooth_theta);
      y1 += DGC_PASSAT_WHEEL_BASE * sin(path->waypoint[i].smooth_theta);
    }

    p.x = x1;
    p.y = y1;
    p.v = path->waypoint[i].v;
    p.fixed = true;
    p.blocked = path->waypoint[i].blocked;
    t.waypoint.push_back(p);

    if(i < last_i) {
      x2 = path->waypoint[i + 1].smooth_x;
      y2 = path->waypoint[i + 1].smooth_y;
      if(!reverse) {
        x2 += DGC_PASSAT_WHEEL_BASE * cos(path->waypoint[i + 1].smooth_theta);
        y2 += DGC_PASSAT_WHEEL_BASE * sin(path->waypoint[i + 1].smooth_theta);
      }

      for(j = 1; j <= N; j++) {
        frac = j / (double)(N + 1);
        p.x = interp(x1, x2, frac);
        p.y = interp(y1, y2, frac);
	p.v = path->waypoint[i].v;
        p.fixed = false;
	p.blocked = false;
	t.waypoint.push_back(p);
      }
    }
  }

  interpolator.optimize(&t, 10000);

  mark = 0;
  if(!seg_begin)
    mark += 2;
  for(i = first_i; i <= last_i; i++) {
    path->waypoint[i].inner_pose.clear();

    p2.x = t.waypoint[mark].x;
    p2.y = t.waypoint[mark].y;
    p2.theta = t.waypoint[mark].theta;
    p2.curvature = t.waypoint[mark].curvature;
    p2.v = t.waypoint[mark].v;
    path->waypoint[i].inner_pose.push_back(p2);
    mark++;

    if(i < last_i) 
      for(j = 1; j <= N; j++) {
	p2.x = t.waypoint[mark].x;
	p2.y = t.waypoint[mark].y;
	p2.theta = t.waypoint[mark].theta;
	p2.curvature = t.waypoint[mark].curvature;
	p2.v = t.waypoint[mark].v;
	path->waypoint[i].inner_pose.push_back(p2);
	mark++;
      }

  }

}

void general_planner::smooth_bidirectional_path(gpp_path *path, int start_i,
						obstacle_info *obs,
						double start_x, double start_y,
						double start_theta,
						double goal_x, double goal_y,
						double goal_theta, 
						bool use_smoother,
						bool restart, 
						bool reached_goal)
{
  bool going_backwards;
  int first_i, last_i;
  bool replan = false;
  bool first_smooth = true;
  int temp_first_i;

  if(path->num_waypoints() < 1)
    return;

  if(start_i == -1) {
    replan = true;
    first_i = 0;
  }
  else {
    first_i = start_i;
  }

  going_backwards = path->waypoint[first_i].reverse;

  do {
    last_i = first_i;
    while(last_i + 1 < path->num_waypoints() && 
	  path->waypoint[last_i + 1].reverse == going_backwards)
      last_i++;
    if(first_i > 0 && first_smooth && path->waypoint[first_i].reverse ==
       path->waypoint[first_i - 1].reverse)
      temp_first_i = first_i - 1;
    else
      temp_first_i = first_i;
    
    if(use_smoother)
      smoother.optimize(path, obs,
			temp_first_i, replan, start_x, start_y, start_theta,
			last_i, reached_goal && 
			last_i == path->num_waypoints() - 1,
			goal_x, goal_y, goal_theta, going_backwards, restart);

    //    path->print("after smoother");
    replan = false;
    first_smooth = false;
    interp_segment(path, temp_first_i, last_i, going_backwards);
    going_backwards = !going_backwards;
    first_i = last_i;
  } while(first_i < path->num_waypoints() - 1);
}

inline void general_planner::add_goal(double x, double y, double theta,
				      bool reverse)
{
  grid_cell *goal_cell = lookup_cell_cont(x, y, theta, reverse);
  float x2, y2, theta2;

  if(goal_cell == NULL) {
    //    dgc_warn("Warning: goal off map!\n");
    return;
  }
  if(!goal_cell->goal) {
    goal_cell->goal = true;
    goal_cell->subgoal = false;
    mark_changed(goal_cell);

    cell_to_pose(goal_cell, &x2, &y2, &theta2);

    //#define DRAW_GOALS
#ifdef DRAW_GOALS
    glsColor3f(gpp_gls, 0, 1, 0);
    glsCircle(gpp_gls, x2 - gls->origin_x, y2 - gls->origin_y, 0.1, 10);
    glsBegin(gpp_gls, GLS_LINES);
    glsVertex2f(gpp_gls, x2 - gls->origin_x, y2 - gls->origin_y);
    glsVertex2f(gpp_gls, x2 + 0.15 * cos(theta2) - gls->origin_x,
		    y2 + 0.15 * sin(theta) - gls->origin_y);
    glsEnd(gpp_gls);
#endif
  }
}

inline void general_planner::add_subgoal(double x, double y, double theta,
					 bool reverse)
{
  grid_cell *goal_cell = lookup_cell_cont(x, y, theta, reverse);

  if(goal_cell == NULL) {
    //    dgc_warn("Warning: subgoal off map!\n");
    return;
  }
  if(!goal_cell->goal && !goal_cell->subgoal) {
    goal_cell->subgoal = true;
    mark_changed(goal_cell);

#ifdef DRAW_GOALS
    glsColor3f(gpp_gls, 1, 1, 0);
    glsCircle(gpp_gls, x - gls->origin_x, y - gls->origin_y, 0.1, 10);
    glsBegin(gpp_gls, GLS_LINES);
    glsVertex2f(gpp_gls, x - gls->origin_x, y - gls->origin_y);
    glsVertex2f(gpp_gls, x + 0.15 * cos(theta) - gls->origin_x,
		    y + 0.15 * sin(theta) - gls->origin_y);
    glsEnd(gpp_gls);
#endif
  }
}

inline void general_planner::add_start(double x, double y, double theta,
				       bool reverse, int start_action)
{
  grid_cell *start_cell = lookup_cell_cont(x, y, theta, reverse);

  start_cell->action = start_action;
  start_cell->parent = NULL;
  start_cell->g = 0;

#ifdef CONTINUOUS
  start_cell->xc = x;
  start_cell->yc = y;
  start_cell->thetac = theta;
#endif

  start_cell->clean = 0;

  compute_heuristic(start_cell);
  open.push(start_cell);
}

inline double general_planner::apply_action_cont(float *x, float *y, 
						 float *theta, 
						 int *steps, action *a, 
						 float grid_resolution)
{
  float ctheta, stheta, start_x, start_y, l;
  int start_xi, start_yi, xi, yi;
  int i = 0;

  ctheta = cos(*theta);
  stheta = sin(*theta);
  start_x = *x;
  start_y = *y;
  start_xi = (int)floor(start_x / grid_resolution);
  start_yi = (int)floor(start_y / grid_resolution);

  i = 0;
  l = 0;
  while(l + a->step_size < grid_resolution) {
    l += a->step_size;
    i++;
  }
  do {
    i++;
    l += a->step_size;
    *x = start_x + a->pose[i].x * ctheta - a->pose[i].y * stheta;
    *y = start_y + a->pose[i].x * stheta + a->pose[i].y * ctheta;
    xi = (int)floor(*x / grid_resolution);
    yi = (int)floor(*y / grid_resolution);
  } while(xi == start_xi && yi == start_yi);
  *theta += a->pose[i].theta;
  *steps = i + 1;
  
  if(a->reverse)
    return l * param.reverse_travel_penalty;
  else
    return l;
}

void general_planner::plan(double robot_x, double robot_y, 
			   double robot_theta, bool robot_reverse, 
			   double robot_vel, gpp_goal *goal, 
			   int use_fine, bool use_smoother, bool breadth_first,
			   obstacle_info *obs, gpp_path *path, 
			   bool *improved, int pop_limit)
{
  grid_cell *current, *next, *final_goal = NULL, *best_sofar = NULL,
    *best_subgoal = NULL;
  float current_x, current_y, current_theta, next_g, l, theta;
  double start_smooth_x, start_smooth_y, start_smooth_theta;
  double start_x, start_y, start_theta, hold_dist, prev_cost = 0;
  bool start_reverse, reversing_action, blocked;
  double obstacle_score;
  int i, start_i, start_action;
  float x, y;
#ifdef CONTINUOUS
  int steps;
#endif
  double x0, y0, theta0;
  int valid;
  int favorite_goal_i, min_i;
  double min_d, d, last_x, last_y, last_theta;

  blocked = false;
  pop_count = 0;
  this->breadth_first = breadth_first;

  if(use_fine) {
    xy_size_ = fine_xy_size;
    xy_resolution_ = fine_xy_resolution;
    theta_size_ = fine_theta_size;
    theta_resolution_ = fine_theta_resolution;
    grid = fine_grid;
    reverse_grid = fine_reverse_grid;
    width_buffer = 1.0;
    length_buffer = 1.0;
    check_width_buffer = 0.5;
    check_length_buffer = 0.5;
  } 
  else {
    xy_size_ = standard_xy_size;
    xy_resolution_ = standard_xy_resolution;
    theta_size_ = standard_theta_size;
    theta_resolution_ = standard_theta_resolution;
    grid = standard_grid;
    reverse_grid = standard_reverse_grid;
    width_buffer = param.astar_width_buffer;
    length_buffer = param.astar_length_buffer;
    check_width_buffer = param.pathcheck_width_buffer;
    check_length_buffer = param.pathcheck_length_buffer;
  }

  favorite_goal_i = 0;
  for(i = 0; i < goal->num_goals(); i++)
    if(goal->goal[i].favorite) {
      favorite_goal_i = i;
      break;
    }

  htable->set_goal(goal->goal[favorite_goal_i].x,
		   goal->goal[favorite_goal_i].y,
		   goal->goal[favorite_goal_i].theta);

  if(path->num_waypoints() > 0) {
    /* move path forward and chop off end of path */
    path->advance_path(robot_x, robot_y, robot_vel);
    if(path->num_waypoints() == 0)
      dgc_die("Error in advance path\n");

    hold_dist = dgc_fmax(6.0, fabs(robot_vel) * 1);
    hold_dist = 4.0;
    //    hold_dist = fabs(robot_vel) * 2;

    if(fabs(robot_vel) < dgc_mph2ms(0.1)) 
      hold_dist = 0;

    path->current_i = path->start_index(robot_x, robot_y, 0.0);
    i = path->start_index(robot_x, robot_y, hold_dist);
    if(i + 1 < path->num_waypoints())
      path->waypoint.erase(path->waypoint.begin() + i + 1, 
			   path->waypoint.end());

    /* start from end of chopped path */
    start_i = path->num_waypoints() - 1;
    start_x = path->waypoint[start_i].x;
    start_y = path->waypoint[start_i].y;
    start_theta = path->waypoint[start_i].theta;
    start_smooth_x = path->waypoint[start_i].smooth_x;
    start_smooth_y = path->waypoint[start_i].smooth_y;
    start_smooth_theta = path->waypoint[start_i].smooth_theta;
    start_reverse = path->waypoint[start_i].reverse;
    start_action = path->waypoint[start_i].action;
    prev_cost = path->total_cost - path->waypoint[start_i].cost;
    path->start_i = i;
  }
  else {
    path->current_i = 0;
    start_i = -1;
    start_x = robot_x;
    start_y = robot_y;
    start_theta = robot_theta;
    start_smooth_x = robot_x;
    start_smooth_y = robot_y;
    start_smooth_theta = robot_theta;
    start_reverse = robot_reverse;
    start_action = -1;
    path->start_i = 0;
  }

  clear_grid();
  center_grid(start_x, start_y);

  glsColor3f(gpp_gls, 1, 1, 1);
  glsBegin(gpp_gls, GLS_LINE_LOOP);
  glsVertex2f(gpp_gls, 
		  start_x - xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_x,
		  start_y - xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_y);
  glsVertex2f(gpp_gls, 
		  start_x + xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_x,
		  start_y - xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_y);
  glsVertex2f(gpp_gls, 
		  start_x + xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_x,
		  start_y + xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_y);
  glsVertex2f(gpp_gls, 
		  start_x - xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_x,
		  start_y + xy_size_ * xy_resolution_ / 2 - gpp_gls->origin_y);
  glsEnd(gpp_gls);

  /* set the start_point */
  add_start(start_x, start_y, start_theta, start_reverse, start_action);
  if(start_i == -1) 
    add_start(start_x, start_y, start_theta, !start_reverse, start_action);

  /* set the goal & subgoals */
  for(i = 0; i < goal->num_goals(); i++) {
    x0 = goal->goal[i].x;
    y0 = goal->goal[i].y;
    theta0 = goal->goal[i].theta;
    add_goal(x0, y0, theta0, false);

    for(x = x0 - goal->goal[i].xy_slop;
	x <= x0 + goal->goal[i].xy_slop; x += xy_resolution_ / 2.0)
      for(y = y0 - goal->goal[i].xy_slop; 
	  y <= y0 + goal->goal[i].xy_slop; y += xy_resolution_ / 2.0)
	for(theta = theta0 - goal->goal[i].theta_slop;
	    theta <= theta0 + goal->goal[i].theta_slop; 
	    theta += theta_resolution_ / 2.0)
	  if(goal->goal[i].picky)
	    add_subgoal(x, y, theta, false);
	  else
	    add_goal(x, y, theta, false);
  }

#ifdef DRAW    
    int drawing_reverse = 0;
    glsLineWidth(gpp_gls, 1);
    glsBegin(gpp_gls, GLS_LINES);
    glsColor3f(gpp_gls, 0, 0, 1);
#endif

  while(!open.empty()) {
    current = open.pop();
    pop_count++;

    if(reset_planning)
      break;

    if(current->closed) 
      continue;

#ifdef DRAW    
    if(current->parent != NULL) {
      if(current->parent->reverse && !drawing_reverse) {
	glsColor3f(gpp_gls, 1, 0.5, 0);
	drawing_reverse = 1;
      }
      else if(!current->parent->reverse && drawing_reverse) {
	glsColor3f(gpp_gls, 0, 0, 1);
	drawing_reverse = 0;
      }
#ifdef CONTINUOUS
      {
	glsVertex2f(gpp_gls, 
			current->xc - gls->origin_x, 
			current->yc - gpp_gls->origin_y);
	glsVertex2f(gpp_gls, 
			current->parent->xc - gls->origin_x,
			current->parent->yc - gpp_gls->origin_y);
      }
#else
      {
	float x, y;
	cell_to_pose(current, &x, &y, &theta);
	glsVertex2f(gpp_gls, x - gls->origin_x, y - gpp_gls->origin_y);
	cell_to_pose(current->parent, &x, &y, &theta);
	glsVertex2f(gpp_gls, x - gls->origin_x, y - gpp_gls->origin_y);
      }
#endif
    }
#endif

    if(best_sofar != NULL && pop_count > pop_limit) 
      break;

    current->closed = true;
    mark_changed(current);

 #ifdef CONTINUOUS
    current_x = current->xc;
    current_y = current->yc;
    current_theta = current->thetac;
#else
    cell_to_pose(current, &current_x, &current_y, &current_theta);
#endif

    obstacle_score = 0;
    valid = valid_state(obs, current_x, current_y, current_theta, 
			check_width_buffer, check_length_buffer, 
			width_buffer, length_buffer,
			&obstacle_score);
    if(!valid)
      continue;
    if(valid && obstacle_score == 0)
      current->clean = 1;

    if(current->goal) {
      final_goal = current;
      break;
    }

    if(current->subgoal && 
       (best_subgoal == NULL || current->h < best_subgoal->h)) 
      best_subgoal = current;

    if(best_sofar == NULL || current->g + current->h < best_sofar->g +
       best_sofar->h)
      best_sofar = current;

    for(i = 0; i < 6; i++) {
      reversing_action = (i >= 3);
      if(reversing_action && !param.allow_reverse)
	continue;

#ifdef CONTINUOUS
      x = current_x;
      y = current_y;
      theta = current_theta;
      l = apply_action_cont(&x, &y, &theta, &steps, 
			    actions[i], xy_resolution_);
      next = lookup_cell_cont(x, y, theta, reversing_action);
#else
      apply_action(current, &next, &l, i);
#endif

      if(next != NULL) {
	if(next->closed)
	  continue;

	//	if(obstacle_score != 0)
	//	  fprintf(stderr, "OS %f\n", obstacle_score);

        next_g = current->g + l + 
	  ((reversing_action != current->reverse) ? 
	   param.reverse_direction_penalty : 0);

	if(current->clean)
	  next_g += 100 * obstacle_score;

	if(next->index == -1 || (next_g < next->g)) {
#ifdef CONTINUOUS
	  next->xc = x;
	  next->yc = y;
	  next->thetac = theta;
#endif
          next->action = i;
          next->g = next_g;
	  next->clean = current->clean;

          if(next->goal)
            next->h = 0;
          else 
	    compute_heuristic(next);
          next->parent = current;

	  if(next->index == -1)  
	    open.push(next);
	  else 
	    open.update_priority(next);
        }
      }
    }
  }

#ifdef DRAW
  glsEnd(gpp_gls);
#endif

  /* clear priority queue after use */
  open.clear();

  /* get path out of A* */
  if(final_goal != NULL) 
    extract_path(path, final_goal, start_i, prev_cost, improved);
  else {
    if(best_subgoal != NULL) 
      extract_path(path, best_subgoal, start_i, prev_cost, improved);
    else 
      extract_path(path, best_sofar, start_i, prev_cost, improved);
  }

  path->reached_goal = (final_goal != NULL || best_subgoal != NULL);
  path->goal_id = goal->id;
  
  if(final_goal != NULL) 
    plan_status = GPP_REACHED_GOAL;
  else if(best_subgoal != NULL) 
    plan_status = GPP_REACHED_SUBGOAL;
  else
    plan_status = GPP_REACHED_OTHER;

  if(plan_status != GPP_REACHED_OTHER) {
    last_x = path->waypoint[path->num_waypoints() - 1].smooth_x;
    last_y = path->waypoint[path->num_waypoints() - 1].smooth_y;
    last_theta = path->waypoint[path->num_waypoints() - 1].smooth_theta;
    min_i = 0;
    min_d = 0;
    for(i = 0; i < goal->num_goals(); i++) {
      d = sqrt(dgc_square(goal->goal[i].x - last_x) +
	       dgc_square(goal->goal[i].y - last_y) +
	       0*dgc_square(goal->goal[i].theta - last_theta));
      if(i == 0 || d < min_d) {
	min_d = d;
	min_i = i;
      }
    }
    path->goal_rndf_wp = goal->goal[min_i].rndf_wp;
  }
  else
    path->goal_rndf_wp = NULL;

  //  path->print("astar path");

  if(start_i != -1) {
    glsColor3f(gpp_gls, 1, 1, 0);
    glsCircle(gpp_gls, start_smooth_x - gpp_gls->origin_x,
		  start_smooth_y - gpp_gls->origin_y, 0.20, 
		  10);
    glsColor3f(gpp_gls, 0, 1, 1);
    glsCircle(gpp_gls, 
		  path->waypoint[path->start_i].smooth_x +
		  DGC_PASSAT_WHEEL_BASE * 
		  cos(start_smooth_theta) -
		  gpp_gls->origin_x,
		  start_smooth_y +
		  DGC_PASSAT_WHEEL_BASE *
		  sin(start_smooth_theta) - 
		  gpp_gls->origin_y,
		  0.25, 10);
  }

  /* smooth path */
  if(final_goal != NULL)
    smooth_bidirectional_path(path, start_i, obs, start_smooth_x, 
			      start_smooth_y, start_smooth_theta,
			      0, 0, 0,
			      use_smoother, true, false);
  else {
    if(best_subgoal != NULL) 
      smooth_bidirectional_path(path, start_i, obs, start_smooth_x, 
				start_smooth_y, start_smooth_theta,
				0, 0, 0, use_smoother,
				true, false);
    else
      smooth_bidirectional_path(path, start_i, obs, start_smooth_x, 
				start_smooth_y, start_smooth_theta,
				0, 0, 0, use_smoother, true, false);
  }

  /* start with max velocities */
  path->blank_velocities(dgc_mph2ms(param.max_vel_mph), dgc_mph2ms(2.0));

  path->mark_obstacles(obs, check_width_buffer, check_length_buffer,
		       width_buffer, length_buffer);
  
  //  path->print("planned path");
  //    fprintf(stderr, "total cost %f\n", path->total_cost);
  last_planned_from_x = start_x;
  last_planned_from_y = start_y;
  last_planned_from_theta = start_theta;

#ifdef DRAW
  /* draw the location the planner planned from */
  glsLineWidth(gpp_gls, 2);
  glsColor3f(gpp_gls, 0, 1, 0);
  gls_draw_robot(gpp_gls, last_planned_from_x, last_planned_from_y,
		 last_planned_from_theta, 0.5);
  glsLineWidth(gpp_gls, 1);
#endif
}

void general_planner::recompute_velocities(gpp_path *path, 
					   obstacle_info *obs)
{
  /* start with max velocities */
  path->blank_velocities(dgc_mph2ms(param.max_vel_mph), dgc_mph2ms(2.0));
  path->mark_obstacles(obs, check_width_buffer, check_length_buffer,
		       width_buffer, length_buffer);
}

void general_planner::smooth(double current_x, double current_y, 
			     __attribute__ ((unused)) double current_theta, 
			     double current_vel, 
			     double goal_x, double goal_y, double goal_theta,
			     bool use_smoother,
			     obstacle_info *obs, gpp_path *path)
{
  double hold_dist;

  path->advance_path(current_x, current_y, current_vel);

  hold_dist = 4.0;
  if(fabs(current_vel) < dgc_mph2ms(0.1)) 
    hold_dist = 0;

  path->start_i = path->start_index(current_x, current_y, hold_dist);

  glsColor3f(gpp_gls, 1, 1, 0);
  glsCircle(gpp_gls, path->waypoint[path->start_i].smooth_x - 
		gpp_gls->origin_x,
		path->waypoint[path->start_i].smooth_y - 
		gpp_gls->origin_y, 0.20, 
		10);
  glsColor3f(gpp_gls, 0, 1, 1);
  glsCircle(gpp_gls, 
		path->waypoint[path->start_i].smooth_x +
		DGC_PASSAT_WHEEL_BASE * 
		cos(path->waypoint[path->start_i].smooth_theta) -
		gpp_gls->origin_x,
		path->waypoint[path->start_i].smooth_y +
		DGC_PASSAT_WHEEL_BASE *
		sin(path->waypoint[path->start_i].smooth_theta) - 
		gpp_gls->origin_y,
		0.25, 10);

  smooth_bidirectional_path(path, path->start_i, obs, 
			    path->waypoint[path->start_i].smooth_x,
			    path->waypoint[path->start_i].smooth_y,
			    path->waypoint[path->start_i].smooth_theta,
			    goal_x, goal_y, goal_theta, use_smoother, false,
			    false);

  path->current_i = path->start_index(current_x, current_y, 0.0);

  /* start with max velocities */
  path->blank_velocities(dgc_mph2ms(param.max_vel_mph), dgc_mph2ms(2.0));
  path->mark_obstacles(obs, check_width_buffer, check_length_buffer,
		       width_buffer, length_buffer);

#ifdef DRAW
  /* draw the location the planner planned from */
  glsLineWidth(gpp_gls, 2);
  glsColor3f(gpp_gls, 0, 1, 0);
  gls_draw_robot(gpp_gls, last_planned_from_x, last_planned_from_y,
		 last_planned_from_theta, 0.5);
  glsLineWidth(gpp_gls, 1);
#endif
}

