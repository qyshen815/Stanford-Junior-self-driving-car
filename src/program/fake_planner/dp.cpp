#include <roadrunner.h>
#include <gl_support.h>
#include <rndf.h>
#include <queue>
#include "planner_data.h"
#include "dp.h"

#define      DP_INITIAL_COST          1e6
#define      DP_NO_DET_OPTION_COST    1e5

using std::queue;
using namespace dgc;

static queue<rndf_waypoint *> openlist, stop_openlist;
static rndf_waypoint *current_goal = NULL;
static double lane_change_prob = 0;

void dp_compute_stop_dist(rndf_file *rndf)
{
  int i, j, k;
  rndf_waypoint *w, *current;
  double d;

  /* initialize all distances to stop signs */
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) 
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        if(w->stop()) {
          PDATA(w->data)->dist_to_stop = 0;
          stop_openlist.push(w);
        }
        else
          PDATA(w->data)->dist_to_stop = 1e6;
      }

  /* do dynamic programming */
  while(!stop_openlist.empty()) {
    /* pop and item off the open list */
    current = stop_openlist.front();
    stop_openlist.pop();

    /* consider zones dead ends for now */
    if(current->in_zone())
      continue;

    /* check predecessors in lane */
    if(current->prev() != NULL) {
      d = PDATA(current->data)->dist_to_stop + 
        PDATA(current->prev()->data)->bt.length;
      if(d < PDATA(current->prev()->data)->dist_to_stop) {
        PDATA(current->prev()->data)->dist_to_stop = d;
        stop_openlist.push(current->prev());
      }
    }

    /* check entries */
    for(i = 0; i < current->num_entries(); i++) {
      d = PDATA(current->data)->dist_to_stop + current->entry_length(i);
      if(d < PDATA(current->entry(i)->data)->dist_to_stop) {
        PDATA(current->entry(i)->data)->dist_to_stop = d;
        stop_openlist.push(current->entry(i));
      }
    }
  }
}

void initialize_dp(rndf_file *rndf, double lane_change_pr)
{
  int i, j, k;

  lane_change_prob = lane_change_pr;
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) 
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        PDATA(rndf->segment(i)->lane(j)->waypoint(k)->data)->cost = 
          DP_INITIAL_COST;
}

float waypoint_cost(rndf_waypoint *w)
{
  float best_det_cost, best_cost = DP_INITIAL_COST, cost;
  int i, found_det_cost;

  /* don't traverse waypoints that aren't in lanes */
  if(!w->in_lane())
    return DP_INITIAL_COST;

  /* find the minimum deterministic cost */
  found_det_cost = 0;
  best_det_cost = DP_INITIAL_COST;
  if(w->next() != NULL) {
    cost = PDATA(w->next()->data)->cost + w->length();
    if(cost < best_det_cost) {
      best_det_cost = cost;
      found_det_cost = 1;
    }
  }
  for(i = 0; i < w->num_exits(); i++) {
    cost = PDATA(w->exit(i)->data)->cost + w->exit_length(i);
    if(cost < best_det_cost) {
      best_det_cost = cost;
      found_det_cost = 1;
    }
  }
  if(!found_det_cost)
    best_det_cost = DP_NO_DET_OPTION_COST;

  /* find best non-deterministic cost */
  best_cost = best_det_cost;
  for(i = 0; i < w->num_exits(lanechange); i++) {
    cost = (PDATA(w->exit(i)->data)->cost + 
	    w->exit_length(i)) * lane_change_prob +
      (1 - lane_change_prob) * best_det_cost;
    if(cost < best_cost)
      best_cost = cost;
  }
  return best_cost;
}

void do_dp(rndf_waypoint *goal)
{
  rndf_waypoint *current;
  float cost;
  int i;

  current_goal = goal;
  
  PDATA(goal->data)->cost = 0;
  openlist.push(goal);

  while(!openlist.empty()) {
    /* pop and item off the open list */
    current = openlist.front();
    openlist.pop();

    /* consider zones dead ends for now */
    if(current->in_zone())
      continue;

    /* check predecessors in lane */
    if(current->prev() != NULL) {
      cost = waypoint_cost(current->prev());
      if(cost < PDATA(current->prev()->data)->cost) {
        PDATA(current->prev()->data)->cost = cost;
        openlist.push(current->prev());
      }
    }

    /* check entries */
    for(i = 0; i < current->num_entries(); i++) {
      cost = waypoint_cost(current->entry(i));
      if(cost < PDATA(current->entry(i)->data)->cost) {
        PDATA(current->entry(i)->data)->cost = cost;
        openlist.push(current->entry(i));
      }
    }
  }
}

global_plan *plan_from(rndf_waypoint *src)
{
  global_plan *p;
  rndf_waypoint *best;
  double best_cost = 1e9;
  int i, done = 0;

  if(current_goal == NULL) {
    fprintf(stderr, "Error: no goal currently set.\n");
    return NULL;
  }
  p = new global_plan;
  p->append_waypoint(src);
  do {
    best = NULL;
    if(src->next() != NULL) {
      if(best == NULL || PDATA(src->next()->data)->cost + src->length() < 
         best_cost) {
        best = src->next();
        best_cost = PDATA(best->data)->cost + src->length();
      }
    }
    for(i = 0; i < src->num_exits(); i++)
      if(best == NULL || PDATA(src->exit(i)->data)->cost + 
         src->exit_length(i) < best_cost) {
        best = src->exit(i);
        best_cost = PDATA(best->data)->cost + src->exit_length(i);
      }

    if(best == NULL || best_cost > PDATA(src->data)->cost + 0.1) 
      done = 1;
    else {
      p->append_waypoint(best);
      src = best;
    }
  } while(!done);
  if(p->num_waypoints() == 0 || 
     p->waypoint(p->num_waypoints() - 1) != current_goal) {
    delete p;
    return NULL;
  }
  return p;
}

#ifdef blah
void draw_dp(rndf_file *rndf, double origin_x, double origin_y)
{
  int i, j, k;
  rndf_waypoint *w;
  float c;
  
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  for(i = 0; i < rndf->num_segments(); i++) 
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) 
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        c = PDATA(w->data)->cost;
        if(c > 5000)
          c = 5000;
        glVertex3f(w->utm_x() - origin_x,
                   w->utm_y() - origin_y, 0);
        glVertex3f(w->utm_x() - origin_x,
                   w->utm_y() - origin_y, 50 * c / 5000.0);
      }
  glEnd();
}

void draw_global_plan(global_plan *p, double origin_x, double origin_y)
{
  int i;
  
  if(p == NULL)
    return;
  glColor3f(0, 1, 0);
  glPushMatrix();
  glTranslatef(0, 0, 1);
  for(i = 0; i < p->num_waypoints() - 1; i++)
    if(i == p->num_waypoints() - 2 || p->waypoint(i + 1)->original())
      draw_arrow(p->waypoint(i)->utm_x() - origin_x,
                 p->waypoint(i)->utm_y() - origin_y,
                 p->waypoint(i + 1)->utm_x() - origin_x,
                 p->waypoint(i + 1)->utm_y() - origin_y, 1, 2);
    else
      draw_line(p->waypoint(i)->utm_x() - origin_x,
                p->waypoint(i)->utm_y() - origin_y,
                p->waypoint(i + 1)->utm_x() - origin_x,
                p->waypoint(i + 1)->utm_y() - origin_y);
  glPopMatrix();
}
#endif
