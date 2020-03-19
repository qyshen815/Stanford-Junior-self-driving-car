#include <roadrunner.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <ibeo_interface.h>
#include <perception_interface.h>
#include <param_interface.h>
#include <grid.h>
#include "simplemapper_ipc.h"
#include "simplemapper.h"

#define    LASER_X_OFFSET       4.07
#define    LASER_Y_OFFSET      -0.52
#define    LASER_YAW_OFFSET      1.5
#define    USE_COMPRESSION         1

int received_applanix_pose = 0;
dgc_applanix_pose_message applanix_pose;

int received_localize_pose = 0;
dgc_localize_pose_message localize_pose;

dgc_ibeo_laser_message laser1;

double laser_x, laser_y, laser_yaw;

dgc_grid_p grid;
map_cell_p default_map_cell = NULL;

int no_changes = 1;
double min_x_change, min_y_change, max_x_change, max_y_change;

void update_change_boundary(double x, double y)
{
  if(no_changes) {
    min_x_change = x;
    min_y_change = y;
    max_x_change = x;
    max_y_change = y;
    no_changes = 0;
  }
  else {
    if(x < min_x_change)
      min_x_change = x;
    if(x > max_x_change)
      max_x_change = x;
    if(y < min_y_change)
      min_y_change = y;
    if(y > max_y_change)
      max_y_change = y;
  }
}

void applanix_handler(void)
{
  received_applanix_pose = 1;
}

void localize_handler(void)
{
  received_localize_pose = 1;
}

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} carmen_bresenham_param_t;

void 
carmen_get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, 
				carmen_bresenham_param_t *params) 
{
  params->UsingYIndex = 0;

  if(fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
    (params->UsingYIndex)++;

  if(params->UsingYIndex) {
    params->Y1 = p1x;
    params->X1 = p1y;
    params->Y2 = p2x;
    params->X2 = p2y;
  } 
  else {
    params->X1 = p1x;
    params->Y1 = p1y;
    params->X2 = p2x;
    params->Y2 = p2y;
  }

  if((p2x - p1x) * (p2y - p1y) < 0) {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  } 
  else
    params->Flipped = 0;
  
  if(params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;
  
  params->DeltaX = params->X2-params->X1;
  params->DeltaY = params->Y2-params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

inline void 
carmen_get_current_point(carmen_bresenham_param_t *params, int *x, int *y) 
{ 
  if(params->UsingYIndex) {
    *y = params->XIndex;
    *x = params->YIndex;
    if(params->Flipped)
      *x = -*x;
  } 
  else {
    *x = params->XIndex;
    *y = params->YIndex;
    if(params->Flipped)
      *y = -*y;
  }
}

inline int 
carmen_get_next_point(carmen_bresenham_param_t *params) 
{
  if(params->XIndex == params->X2)
    return 0;
  params->XIndex += params->Increment;
  if(params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))     
    params->DTerm += params->IncrE;
  else {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

void laser1_handler(void)
{
  double robot_x, robot_y, p_x, p_y, p_z, s_x, s_y, s_z;
  dgc_transform_t t;
  map_cell_p cell;
  int i, x_i, y_i;
  carmen_bresenham_param_t bp;

  if(!received_applanix_pose || !received_localize_pose)
    return;

  robot_x = applanix_pose.smooth_x;
  robot_y = applanix_pose.smooth_y;

  /* roll the grid */
  dgc_grid_recenter_grid(grid, robot_x, robot_y);

  /* start with identity transform */
  dgc_transform_identity(t);

  /* transform from IMU -> laser */
  dgc_transform_translate(t, LASER_X_OFFSET, LASER_Y_OFFSET, 0);
  dgc_transform_rotate_z(t, dgc_d2r(LASER_YAW_OFFSET));

  /* transform from world -> IMU */
  dgc_transform_rotate_x(t, applanix_pose.roll);
  dgc_transform_rotate_y(t, applanix_pose.pitch);
  dgc_transform_rotate_z(t, applanix_pose.yaw);

  /* put laser data in the map */
  for(i = 0; i < laser1.num_points; i++)
    if(laser1.point[i].status == DGC_IBEO_STATUS_OK &&
       (fabs(laser1.point[i].x) > 0.4 ||
	laser1.point[i].y > 1.5 ||
	laser1.point[i].y < 0.0)) {
      /* do 3D transformation */
      s_x = 0;
      s_y = 0;
      s_z = 0;
      dgc_transform_point(&s_x, &s_y, &s_z, t);
      s_x += applanix_pose.smooth_x;
      s_y += applanix_pose.smooth_y;
      s_z += applanix_pose.smooth_z;

      /* do 3D transformation */
      p_x = laser1.point[i].x;
      p_y = laser1.point[i].y;
      p_z = laser1.point[i].z;
      dgc_transform_point(&p_x, &p_y, &p_z, t);
      p_x += applanix_pose.smooth_x;
      p_y += applanix_pose.smooth_y;
      p_z += applanix_pose.smooth_z;

      carmen_get_bresenham_parameters((int)floor(s_x / grid->resolution),
				      (int)floor(s_y / grid->resolution),
				      (int)floor(p_x / grid->resolution),
				      (int)floor(p_y / grid->resolution), &bp);
      
      do {
	carmen_get_current_point(&bp, &x_i, &y_i);
	cell = dgc_grid_get_rc_global(grid, y_i, x_i);
	if(cell != NULL) {
	  if(cell->unknown || cell->value != 1) {
	    cell->value = 0;
	    cell->unknown = 0;
	    cell->changed = 1;
	    update_change_boundary(p_x, p_y);
	  }
	}
      } while(carmen_get_next_point(&bp));

      /* lookup map grid cell */
      cell = dgc_grid_get_xy(grid, p_x, p_y);
      if(cell != NULL) {
	if(cell->unknown || cell->value != 1) {
	  cell->value = 1;
	  cell->unknown = 0;
	  cell->changed = 1;
	  update_change_boundary(p_x, p_y);
	}
      }
    }

  /* publish map diff */
  mapper_publish_diff(grid, min_x_change, min_y_change,
		      max_x_change, max_y_change, USE_COMPRESSION, 0);
  no_changes = 1;
}

void requestmap_handler(__attribute__ ((unused)) 
			dgc_perception_requestmap_message request)
{
  map_cell_p cell;
  int r, c;

  for(r = 0; r < grid->rows; r++)
    for(c = 0; c < grid->cols; c++) {
      cell = (map_cell_p)dgc_grid_get_rc_local(grid, r, c);
      if(cell != NULL) 
        cell->changed = 1;
    }
  mapper_publish_diff(grid, min_x_change, min_y_change,
		      max_x_change, max_y_change, USE_COMPRESSION, 1);
}

int main(int argc, char **argv)
{
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  mapper_register_ipc_messages();

  /* allocate rolling grid */
  default_map_cell = (map_cell_p)calloc(1, sizeof(map_cell_t));
  dgc_test_alloc(default_map_cell);
  default_map_cell->value = 0;
  default_map_cell->changed = 0;
  default_map_cell->unknown = 1;
  grid = dgc_grid_initialize(0.2, 400, 400, sizeof(map_cell_t),
			     default_map_cell);

  dgc_applanix_subscribe_pose_message(&applanix_pose, (dgc_handler_t)
				      applanix_handler, DGC_SUBSCRIBE_LATEST, 
				      NULL);
  dgc_localize_subscribe_pose_message(&localize_pose, (dgc_handler_t)
				      localize_handler, DGC_SUBSCRIBE_LATEST,
				      NULL);
  dgc_ibeo_subscribe_laser1_message(&laser1, (dgc_handler_t)
				    laser1_handler, DGC_SUBSCRIBE_LATEST,
				    NULL);
  dgc_perception_subscribe_requestmap_message(NULL, (dgc_handler_t)
					      requestmap_handler, 
					      DGC_SUBSCRIBE_ALL, NULL);
  dgc_ipc_dispatch();
  return 0;
}
