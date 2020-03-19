#ifndef DGC_KDTREE_H
#define DGC_KDTREE_H

/* internal kdtree structures */

typedef struct {
  double min_x, min_y, max_x, max_y;
} dgc_hyperrect;

typedef struct {
  int id;
  double x, y;
} dgc_tagged_point;

typedef struct dgc_kdtree_struct {
  char axis;
  double axis_split;
  struct dgc_kdtree_struct *left, *right;
  int num_points, max_points;
  dgc_tagged_point *data;
} dgc_kdtree_t, *dgc_kdtree_p;

/* external structures */

typedef struct dgc_dlist_node_struct {
  double distance;
  int id;
  struct dgc_dlist_node_struct *next;
} dgc_dlist_node_t, *dgc_dlist_node_p;

typedef struct dgc_priority_queue_node {
  int id;
  double distance;
  struct dgc_priority_queue_node *prev, *next;
} dgc_priority_queue_node_t, *dgc_priority_queue_node_p;

typedef struct {
  int length, max_length;
  dgc_priority_queue_node_p first, last;
} dgc_priority_queue_t, *dgc_priority_queue_p;

#ifdef __cplusplus
extern "C" {
#endif

/* linked list functions */

void dgc_dlist_add(dgc_dlist_node_p *list, int id, double distance);

void dgc_dlist_free(dgc_dlist_node_p *list);       /* necessary functions are from
                                              here down */
int dgc_dlist_length(dgc_dlist_node_p list);

void dgc_dlist_print(dgc_dlist_node_p list);

/* kd-tree functions */

dgc_kdtree_p dgc_kdtree_build_even(double max_x, double max_y, 
				   double min_size);

void dgc_even_kdtree_clear(dgc_kdtree_p kdtree);

void dgc_even_kdtree_add_points(dgc_kdtree_p kdtree, double *x, double *y, 
				int num_points);

dgc_kdtree_p dgc_kdtree_build_balanced(double *x, double *y, int num_points);

void dgc_kdtree_nearest_neighbor(dgc_kdtree_p kdtree, double x, double y, 
                                 int *which, double *distance);

dgc_dlist_node_p dgc_kdtree_range_search(dgc_kdtree_p kdtree, 
                                         double x, double y, double max_range);
  
dgc_priority_queue_p dgc_kdtree_k_nearest_neighbors(dgc_kdtree_p kdtree, 
                                                    double x, double y, int k);

void dgc_kdtree_free(dgc_kdtree_p *kdtree);

/* priority queue functions */

void dgc_priority_queue_print(dgc_priority_queue_p queue, int backward);

void dgc_priority_queue_free(dgc_priority_queue_p queue);

#ifdef __cplusplus
}
#endif

#endif
