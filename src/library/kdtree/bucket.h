#ifndef DGC_BUCKET_H
#define DGC_BUCKET_H

#include <roadrunner.h>

typedef struct {
  double x, y;
  int id;
} bucket_point;

typedef struct dgc_neighbor_list_struct {
  float distance;
  double x, y;
  int id;
  struct dgc_neighbor_list_struct *next;
} dgc_neighbor_list_t, *dgc_neighbor_list_p;

struct bucket_t {
  int num_points, max_points;
  bucket_point *point;
};

class bucket_grid {
public:
  bucket_grid(double resolution, double width, double height);
  ~bucket_grid();

  void clear(void);
  void build_grid(double *x, double *y, int num_points);
  void build_grid(double *x, double *y, int num_points,
		  double min_x, double min_y);
  void nearest_neighbor(double x, double y, double max_range,
			double *closest_x, double *closest_y, int *which,
			double *distance);
  dgc_neighbor_list_p range_search(double x, double y, double r);
  void range_search(double x, double y, double range, 
		    bucket_point **neighbor_list,
		    int *max_n);

  void print_stats(void);

private:
  double min_x, min_y, resolution;
  int x_size, y_size;
  bucket_t **grid;
};

void dgc_neighbor_list_free(dgc_neighbor_list_p *list);

#endif 
