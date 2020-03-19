#include <roadrunner.h>
#include "heuristic.h"
#include <kdtree.h>
#include <vector>

using std::vector;

heuristic_table::heuristic_table(double min_x, double min_y, 
				 double xy_resolution, int x_size, 
				 int y_size, int theta_size)
{
  int x, y, theta, i;

  this->min_x = min_x;
  this->min_y = min_y;
  this->xy_resolution = xy_resolution;
  this->x_size = x_size;
  this->y_size = y_size;
  this->theta_size = theta_size;
  this->min_h = min_h;
  this->max_h = max_h;

  theta_resolution = 2 * M_PI / this->theta_size;

  data = (hcell **)calloc(x_size * y_size, sizeof(hcell *));
  dgc_test_alloc(data);
  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) {
      data[y * x_size + x] = (hcell *)calloc(theta_size, sizeof(hcell));
      dgc_test_alloc(data[y * x_size + x]);
    }

  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) {
      i = y * x_size + x;
      for(theta = 0; theta < theta_size; theta++) {
	data[i][theta].cost = 0;
	data[i][theta].rev_cost = 0;
	data[i][theta].computed = false;
	data[i][theta].rev_computed = false;
      }
    }
}

void heuristic_table::precompute_mins(void)
{
  int xi, yi, thetai, dx, dy, dtheta, i, i2;
  double min_cost;

  for(xi = 0; xi < x_size; xi++) 
    for(yi = 0; yi < y_size; yi++) {
      i = yi * x_size + xi;
      for(thetai = 0; thetai < theta_size; thetai++) {
	min_cost = data[i][thetai].cost;
	for(dx = -1; dx <= 1; dx++)
	  for(dy = -1; dy <= 1; dy++)
	    for(dtheta = -1; dtheta <= 1; dtheta++) 
	      if(xi + dx >= 0 && xi + dx < x_size &&
		 yi + dy >= 0 && yi + dy < y_size &&
		 thetai + dtheta >= 0 && thetai + dtheta < theta_size) {
		i2 = (yi + dy) * x_size + (xi + dx);
		if(data[i2][thetai + dtheta].cost < min_cost)
		  min_cost = data[i2][thetai + dtheta].cost;
	      }
	data[i][thetai].min_cost = min_cost;

	min_cost = data[i][thetai].rev_cost;
	for(dx = -1; dx <= 1; dx++)
	  for(dy = -1; dy <= 1; dy++)
	    for(dtheta = -1; dtheta <= 1; dtheta++) 
	      if(xi + dx >= 0 && xi + dx < x_size &&
		 yi + dy >= 0 && yi + dy < y_size &&
		 thetai + dtheta >= 0 && thetai + dtheta < theta_size) {
		i2 = (yi + dy) * x_size + (xi + dx);
		if(data[i2][thetai + dtheta].rev_cost < min_cost)
		  min_cost = data[i2][thetai + dtheta].rev_cost;
	      }
	data[i][thetai].min_rev_cost = min_cost;
      }
    }
}

heuristic_table::heuristic_table(char *filename)
{
  FILE *fp;
  int x, y, theta, i;
  int computed, rev_computed;

  fp = fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);

  fscanf(fp, "%lf %lf %lf %d %d %d\n", &min_x, &min_y, &xy_resolution, &x_size,
	 &y_size, &theta_size);

  theta_resolution = 2 * M_PI / theta_size;

  data = (hcell **)calloc(x_size * y_size, sizeof(hcell *));
  dgc_test_alloc(data);
  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) {
      data[y * x_size + x] = (hcell *)calloc(theta_size, sizeof(hcell));
      dgc_test_alloc(data[y * x_size + x]);
    }
  
  for(y = 0; y < y_size; y++)
    for(x = 0; x < x_size; x++) {
      i = y * x_size + x;
      for(theta = 0; theta < theta_size; theta++) {
	fscanf(fp, "%d %d %f %f\n", &computed, &rev_computed,
	       &data[i][theta].cost, &data[i][theta].rev_cost);
	data[i][theta].computed = (computed != 0);
	data[i][theta].rev_computed = (rev_computed != 0);
      }
    }
  fclose(fp);

  recompute_extrema();
  precompute_mins();

  goal_x = 0;
  goal_y = 0;
  goal_theta = 0;
}

void heuristic_table::recompute_extrema(void)
{
  int x, y, theta, i;

  min_h = 1e6;
  max_h = 0;

  for(y = 0; y < y_size; y++)
    for(x = 0; x < x_size; x++) {
      i = y * x_size + x;
      for(theta = 0; theta < theta_size; theta++) {
	if(data[i][theta].computed) {
	  if(data[i][theta].cost < min_h)
	    min_h = data[i][theta].cost;
	  if(data[i][theta].cost > max_h)
	    max_h = data[i][theta].cost;
	}
	if(data[i][theta].rev_computed) {
	  if(data[i][theta].rev_cost < min_h)
	    min_h = data[i][theta].rev_cost;
	  if(data[i][theta].rev_cost > max_h)
	    max_h = data[i][theta].rev_cost;
	}
      }
    }
}

void heuristic_table::fill_uncomputed(void)
{
  vector <double> filled_x, filled_y, filled_cost;
  int x, y, theta, which, i;
  double real_x, real_y, distance, cost;
  dgc_kdtree_p kdtree;

  for(theta = 0; theta < theta_size; theta++) {
    filled_x.clear();
    filled_y.clear();
    filled_cost.clear();

    for(x = 0; x < x_size; x++)
      for(y = 0; y < y_size; y++) {
	i = y * x_size + x;
	real_x = min_x + x * xy_resolution;
	real_y = min_y + y * xy_resolution;
	if(data[i][theta].computed) {
	  cost = data[i][theta].cost;
	  filled_x.push_back(real_x);
	  filled_y.push_back(real_y);
	  filled_cost.push_back(cost);
	}
      }
    
    kdtree = dgc_kdtree_build_balanced(&(filled_x[0]), &(filled_y[0]),
				       filled_x.size());

    for(x = 0; x < x_size; x++)
      for(y = 0; y < y_size; y++) {
	i = y * x_size + x;
	if(!data[i][theta].computed) {
	  real_x = min_x + x * xy_resolution;
	  real_y = min_y + y * xy_resolution;
	
	  dgc_kdtree_nearest_neighbor(kdtree, real_x, 
				      real_y, &which, &distance);

	  data[i][theta].cost = distance + filled_cost[which];
	}
      }
    dgc_kdtree_free(&kdtree);
  }

  for(theta = 0; theta < theta_size; theta++) {
    filled_x.clear();
    filled_y.clear();
    filled_cost.clear();

    for(x = 0; x < x_size; x++)
      for(y = 0; y < y_size; y++) {
	i = y * x_size + x;
	real_x = min_x + x * xy_resolution;
	real_y = min_y + y * xy_resolution;
	if(data[i][theta].rev_computed) {
	  cost = data[i][theta].rev_cost;
	  filled_x.push_back(real_x);
	  filled_y.push_back(real_y);
	  filled_cost.push_back(cost);
	}
      }
    
    kdtree = dgc_kdtree_build_balanced(&(filled_x[0]), &(filled_y[0]),
				       filled_x.size());

    for(x = 0; x < x_size; x++)
      for(y = 0; y < y_size; y++) {
	i = y * x_size + x;
	if(!data[i][theta].rev_computed) {
	  real_x = min_x + x * xy_resolution;
	  real_y = min_y + y * xy_resolution;
	
	  dgc_kdtree_nearest_neighbor(kdtree, real_x, 
				      real_y, &which, &distance);

	  data[i][theta].rev_cost = distance + filled_cost[which];
	}
      }
    dgc_kdtree_free(&kdtree);
  }
}

void heuristic_table::save(char *filename)
{
  FILE *fp;
  int x, y, theta, i;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);
  
  fprintf(fp, "%f %f %f %d %d %d\n", min_x, min_y, xy_resolution, x_size,
	  y_size, theta_size);

  for(y = 0; y < y_size; y++) 
    for(x = 0; x < x_size; x++) {
      i = y * x_size + x;
      for(theta = 0; theta < theta_size; theta++)
	fprintf(fp, "%d %d %.2f %.2f\n", data[i][theta].computed, 
		data[i][theta].rev_computed, 
		data[i][theta].cost, data[i][theta].rev_cost);
    }
  fclose(fp);
}

heuristic_table::~heuristic_table()
{
  int x, y, i;

  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) {
      i = y * x_size + x;
      free(data[i]);
    }
  free(data);
}

void heuristic_table::set_goal(double goal_x, double goal_y, double goal_theta)
{
  this->goal_x = goal_x;
  this->goal_y = goal_y;
  this->goal_theta = goal_theta;
  ctheta = cos(-this->goal_theta);
  stheta = sin(-this->goal_theta);
}

double heuristic_table::get_value(double x, double y, double theta, 
				  bool reverse)
{
  double x2, y2, x3, y3, theta3;
  int xi, yi, thetai, xi2, yi2, i;

  //#define SIMPLE_HEURISTIC
#ifdef SIMPLE_HEURISTIC
  return hypot(x - goal_x, y - goal_y);
#endif  
  
  x2 = x - goal_x;
  y2 = y - goal_y;
  x3 = ctheta * x2 - stheta * y2;
  y3 = stheta * x2 + ctheta * y2;
  theta3 = theta - goal_theta;

  xi = (int)floor((x3 - min_x) / xy_resolution);
  yi = (int)floor((y3 - min_y) / xy_resolution);
  thetai = (int)rint((dgc_normalize_theta(theta3) + M_PI) / 
                     (2 * M_PI) * theta_size);
  if(thetai == theta_size)
    thetai = 0;

  if(xi >= 0 && yi >= 0 && xi < x_size && yi < y_size) {
    i = yi * x_size + xi;
    if(reverse) 
      return data[i][thetai].min_rev_cost;
    else 
      return data[i][thetai].min_cost;
  }
  else {
    xi2 = xi;
    if(xi2 < 0)
      xi2 = 0;
    if(xi2 >= x_size)
      xi2 = x_size - 1;
    yi2 = yi;
    if(yi2 < 0)
      yi2 = 0;
    if(yi2 >= y_size)
      yi2 = y_size - 1;
    i = yi2 * x_size + xi2;
    return hypot(xi2 - xi, yi2 - yi) * xy_resolution +
      dgc_fmin(data[i][thetai].cost,
	       data[i][thetai].rev_cost);
  }
}

