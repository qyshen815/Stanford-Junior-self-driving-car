#include "perception.h"

using namespace dgc;

restricted_2d_area_t    ibeo_box[NUM_IBEO_LASERS] = 
  { { -5.0,  0.8,  -2.0,  0.5, -1.3, 1.15 },
    { -5.0,  0.8,  -0.5,  2.0, -1.3, 1.15 } };

dgc_transform_t                 ibeo_offset[NUM_IBEO_LASERS];

#define    NUM_BINS_PER_REV             180
#define    MAX_BEAMS_IN_BIN             20
#define    MAX_DIST_SAME_OBSTACLE       1.0

typedef struct {
  IbeoLaserPoint     *point;
  double                    dist;
  double                    angle;
  short                     valid;
} ibeo_point_t;

typedef struct {
  int                    num_beams;
  ibeo_point_t           beam[MAX_BEAMS_IN_BIN];
} ibeo_bin_t;

double
ibeo_beam_dist_2d( IbeoLaserPoint *p1, IbeoLaserPoint *p2 )
{
  return(hypot( p1->x-p2->x, p1->y-p2->y )); 
}

void 
integrate_ibeo( IbeoLaser *ibeo, dgc_transform_t *ibeo_offset, 
		restricted_2d_area_t *area, int nr, unsigned short counter )
{
  static ibeo_bin_t            ibbin[NUM_BINS_PER_REV][NUM_IBEO_LASERS];
  int                          i;
  dgc_transform_t              t;
  double                       dist, angle, px, py, pz;
  dgc_perception_map_cell_p    cell = NULL, terrain_cell = NULL;
  ApplanixPose               * pose;

  for (i=0; i<NUM_BINS_PER_REV; i++) {
    ibbin[i][nr].num_beams = 0;
  }
  
  for(i = 0; i < ibeo->num_points; i++) {
    if( ibeo->point[i].status == DGC_IBEO_STATUS_OK &&
	ibeo->point[i].level>=0 && 
	ibeo->point[i].level<NUM_IBEO_BEAMS ) {
      px = ibeo->point[i].x;
      py = ibeo->point[i].y;
      pz = ibeo->point[i].z;
      dist = hypot( px, py );
      angle = atan2(py, px );
      if ( dist>settings.ibeo_min_dist && 
	   dist<settings.ibeo_max_dist &&
	   angle>settings.ibeo_min_angle &&
	   angle<settings.ibeo_max_angle &&
	   angle>area->min_angle && angle<area->max_angle &&
	   !(px>area->x1 && px<area->x2 &&
	     py>area->y1 && py<area->y2) ) {
	
	pose = applanix_pose(ibeo->timestamp);
	dgc_transform_copy( t, *ibeo_offset );
	dgc_transform_rotate_x( t, pose->roll );
	dgc_transform_rotate_y( t, pose->pitch );
	dgc_transform_rotate_z( t, pose->yaw );
	dgc_transform_translate( t, 
				 pose->smooth_x, 
				 pose->smooth_y, 
				 pose->smooth_z );
	dgc_transform_point(&px, &py, &pz, t);
	cell = (dgc_perception_map_cell_p)grid_get_xy(grid, px, py );
	if (cell!=NULL) {
	  if (cell->last_use!=counter) {
	    obstacles_s->cell[obstacles_s->num] = cell;
	    if (obstacles_s->num<MAX_NUM_POINTS)
	      obstacles_s->num++;
	    cell->last_use = counter;
	  }
	  terrain_cell = 
	    (dgc_perception_map_cell_p)grid_get_xy(terrain_grid, px, py );
	  set_cell_min( cell, pz, counter );
	  set_cell_max( cell, pz, counter );
	  set_cell_min( terrain_cell, pz, counter );
	  set_cell_max( terrain_cell, pz, counter );
	  sync_with_terrain( terrain_cell, cell );
	  cell->hits ++;
	  include_cell(cell,counter);
	}
      }
    }
  }

}
  
