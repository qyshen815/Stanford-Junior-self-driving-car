#include <roadrunner.h>
#include <pswrap.h>
#include "kdtree.h"

#define N 100000
#define M 1000

#define PSX(x) ((x)/150.0*10.0)
#define PSY(y) ((y)/150.0*10.0)

ps_doc_p doc;

void draw_splits(dgc_kdtree_p kdtree, float left, float bottom,
		 float right, float top)
{
  if(kdtree == NULL)
    return;
  else if(kdtree->left == NULL && kdtree->right == NULL)
    return;
  else if(kdtree->axis == 'x') {
    ps_draw_line(doc, PSX(kdtree->axis_split), PSY(bottom),
		 PSX(kdtree->axis_split), PSY(top));
    draw_splits(kdtree->left, left, bottom, kdtree->axis_split, top);
    draw_splits(kdtree->right, kdtree->axis_split, bottom, right, top);
  }
  else if(kdtree->axis == 'y') {
    ps_draw_line(doc, PSX(left), PSY(kdtree->axis_split),
		 PSX(right), PSY(kdtree->axis_split));
    draw_splits(kdtree->left, left, bottom, right, kdtree->axis_split);
    draw_splits(kdtree->right, left, kdtree->axis_split, right, top);
  }
}

void draw_points(dgc_kdtree_p kdtree)
{
  int i;

  if(kdtree == NULL)
    return;
  else {
    for(i = 0; i < kdtree->num_points; i++)
      ps_draw_circle(doc, 1, PSX(kdtree->data[i].x), PSY(kdtree->data[i].y), PSX(0.1));
    if(kdtree->axis == 'x') {
      draw_points(kdtree->left);
      draw_points(kdtree->right);
    }
    else if(kdtree->axis == 'y') {
      draw_points(kdtree->left);
      draw_points(kdtree->right);
    }
  }
}

void draw_kdtree(dgc_kdtree_p kdtree)
{
  doc = ps_open("kdtree.ps", 10, 10, 0);
  ps_set_color(doc, 0, 0, 0);
  draw_splits(kdtree, 0, 0, 150, 150);
  ps_set_color(doc, 0, 0, 255);
  draw_points(kdtree);
  ps_close(doc);
}

int main(int argc, char **argv)
{
  double *x, *y;
  int i;
  dgc_kdtree_p kdtree, even_kdtree;
  double test_x[M], test_y[M];
  dgc_dlist_node_p dlist;

  //  int j, min_j = 0;
  //  double d, min_dist;

  x = (double *)calloc(N, sizeof(double));
  dgc_test_alloc(x);
  y = (double *)calloc(N, sizeof(double));
  dgc_test_alloc(y);

  dgc_randomize(&argc, &argv);
  for(i = 0; i < N; i++) {
    x[i] = dgc_uniform_random(0, 150);
    y[i] = dgc_uniform_random(0, 150);

    x[i] = dgc_gaussian_random(75, 1.5);
    y[i] = dgc_gaussian_random(75, 1.5);
  }

  dgc_time_code(kdtree = dgc_kdtree_build_balanced(x, y, N);,"BUILD BALANCED");

  dgc_time_code(even_kdtree = dgc_kdtree_build_even(150, 150, 1);,"BUILD EVEN");
  dgc_time_code(dgc_even_kdtree_add_points(even_kdtree, x, y, N);,"FILL EVEN");
  dgc_time_code(dgc_even_kdtree_clear(even_kdtree);,"CLEAR EVEN");
  dgc_time_code(dgc_even_kdtree_add_points(even_kdtree, x, y, N);,"FILL EVEN");

    draw_kdtree(kdtree);

  for(i = 0; i < M; i++) {
    test_x[i] = dgc_uniform_random(0, 150);
    test_y[i] = dgc_uniform_random(0, 150);
  }


  dgc_time_code(
  for(i = 0; i < M; i++) {
    dlist = dgc_kdtree_range_search(kdtree, test_x[i], test_y[i], 3.0);
    if(dlist != NULL)
      dgc_dlist_free(&dlist);
  }
  ,"BALANCED LOOKUP");

  dgc_time_code(
  for(i = 0; i < M; i++) {
    dlist = dgc_kdtree_range_search(even_kdtree, test_x[i], test_y[i], 3.0);
    if(dlist != NULL)
      dgc_dlist_free(&dlist);
  }
  ,"EVEN LOOKUP");

    /*
    for(j = 0; j < N; j++) {
      d = hypot(test_x - x[j], test_y - y[j]);
      if(j == 0 || d < min_dist) {
	min_dist = d;
	min_j = j;
      }
    }
    
    if(which != min_j)
      fprintf(stderr, "%d: kdtree %d %.3f raw %d %.3f\n", i, which, distance,
      min_j, min_dist);*/

  return 0;
}
