#include <roadrunner.h>
#include "grid.h"

typedef struct {
  double x, y;
} grid_cell_t, *grid_cell_p;

dgc_grid_p grid;
int lookup_failure = 0;

void clear_handler(void *cellp)
{
  grid_cell_p cell = (grid_cell_p)cellp;
  double xf, yf;

  dgc_grid_cell_to_xy(grid, cell, &xf, &yf);
  
  if(cell->x == -100 || cell->y == -100)
    return;

  if(xf != cell->x || yf != cell->y) {
    fprintf(stderr, "predicted location %f %f cached_location %f %f\n",
	    xf, yf, cell->x, cell->y);
    lookup_failure = 1;
  }
}

int main(void)
{
  grid_cell_p cell, default_cell;
  double x, y;
  int r, c;
  
  default_cell = (grid_cell_p)calloc(1, sizeof(grid_cell_t));
  dgc_test_alloc(default_cell);
  default_cell->x = -100;
  default_cell->y = -100;

  /* initialize a 10 x 10 grid, centered at 5 */
  grid = dgc_grid_initialize(1, 10, 10, sizeof(grid_cell_t), default_cell);
  dgc_grid_recenter_grid(grid, 5, 5);
  
  /* fill each cell with its global x y position */
  for(c = 0; c < grid->cols; c++) 
    for(r = 0; r < grid->rows; r++) {
      cell = dgc_grid_get_rc_local(grid, r, c);
      dgc_grid_rc_local_to_xy(grid, r, c, &x, &y);
      cell->x = x;
      cell->y = y;
    }
  
  /* install clear handler */
  dgc_grid_set_clear_handler(grid, clear_handler);

  /* recenter the grid in each direction */
  dgc_grid_recenter_grid(grid, 5, 4);
  dgc_grid_recenter_grid(grid, 5, 6);
  dgc_grid_recenter_grid(grid, 4, 5);
  dgc_grid_recenter_grid(grid, 6, 5);

  if(!lookup_failure)
    fprintf(stderr, "All tests pass!\n");
  return 0;
}
