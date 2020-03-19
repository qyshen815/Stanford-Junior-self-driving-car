#ifndef DGC_FSMDRAW_H
#define DGC_FSMDRAW_H

#include <roadrunner.h>
#include <vector>

typedef struct {
  int num;
  double x, y, width, height;
  char label[100];
} fsm_node_t;

typedef struct {
  int num_points;
  float *point;
} fsm_edge_t, *fsm_edge_p;

typedef struct {
  double width, height;
  std::vector <fsm_node_t> node;
  std::vector <fsm_edge_t> edge;
} fsm_graph_t, *fsm_graph_p;

fsm_graph_p
fsm_read_graph_from_file(FILE *fp);

fsm_graph_p
fsm_read_graph_from_memory(char *filename);

void
fsm_draw_graph(fsm_graph_p graph, double x, double y, double size, 
	       int highlight_node);

#endif
