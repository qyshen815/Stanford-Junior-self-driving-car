/*
 *  Created on: Jul 27, 2009
 *      Author: duhadway
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <assert.h>

#include <grid.h>

#include "perception_types.h"

void cell_to_coord( dgc_grid_p  grid, dgc_perception_map_cell_p cell, short *x, short *y );
void cell_to_coord( dgc_grid_p  grid, dgc_perception_map_cell_p cell, double *x, double *y);

void  include_cell( dgc_perception_map_cell_p cell, unsigned short counter );
int   change_cell( dgc_perception_map_cell_p cell, unsigned short counter );
int   inside_car( float x, float y, dgc_pose_t robot );

unsigned char cell_eval( dgc_perception_map_cell_p cell );
float cell_height( dgc_perception_map_cell_p cell );

void perception_map_reset( dgc_grid_p map );

double sample(double b);

inline int
mywrap(int x, int max)
{
  if(x >= max) {
    return(x%max);
  } else if(x < 0) {
    return((x+max)%max);
  }
  return x;
}

inline void
grid_xy_to_rc(dgc_grid_p grid, float x, float y, int *r, int *c)
{
  *c = (int)floor(x / grid->resolution) - grid->map_c0;
  *r = (int)floor(y / grid->resolution) - grid->map_r0;
}

inline void
grid_rc_to_xy(dgc_grid_p grid, int r, int c, float* x, float* y)
{
  *x = (c + grid->map_c0 + 0.5) * grid->resolution;
  *y = (r + grid->map_r0 + 0.5) * grid->resolution;
}

inline void
grid_rc_to_xy(dgc_grid_p grid, int r, int c, double* x, double* y)
{
  *x = (c + grid->map_c0 + 0.5) * grid->resolution;
  *y = (r + grid->map_r0 + 0.5) * grid->resolution;
}

inline void *
grid_get_rc(dgc_grid_p grid, int r, int c)
{
  if(r < 0 || c < 0 || r >= grid->rows || c >= grid->cols)
    return NULL;
  else {
    r = mywrap(r + grid->array_r0, grid->rows);
    c = mywrap(c + grid->array_c0, grid->cols);
    return (void *)((char*)grid->cell +
        grid->bytes_per_cell * (r * grid->cols + c));
  }
}

inline void *
grid_get_xy(dgc_grid_p grid, float x, float y)
{
  int r, c;
  assert(grid != NULL);
  grid_xy_to_rc(grid, x, y, &r, &c);
  return grid_get_rc(grid, r, c);
}

inline double linear(double x, double slope, double intercept) {
  return (slope * x + intercept);
}

inline double angle_diff(double t1, double t2)
{
  double dt = t1 - t2;
  while (dt > M_PI) dt -= 2 * M_PI;
  while (dt < -M_PI) dt += 2 * M_PI;
  return dt;
}

inline double theta(double x, double y)
{
  return atan2(y,x);
}

inline double range(double x, double y, double z)
{
  return sqrt(x*x + y*y + z*z);
}

inline double range(double x, double y)
{
  return hypot(x,y);
}

inline double logistic(double x, double max_x, double max, double min){
  return (max - (max - min)/(1.0 + exp(max_x / 2.0 - x)));
}

#endif /* UTILS_H_ */
