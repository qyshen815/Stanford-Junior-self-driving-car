#include "perception.h"
#include "utils.h"

using namespace dgc;

#define PASSAT_WIDTH                     2.00
#define PASSAT_LENGTH                    4.78
#define PASSAT_HEIGHT                    1.52
#define DISTANCE_IMU_REF_POINT_TO_BACK   1.65
#define MIN_CAR_DISTANCE_TO_BE_INSIDE    4.0

dgc_transform_t                 ldlrs_offset[NUM_LDLRS_LASERS];

// TODO: why is this different from the beam_inside_car inside velocore?
short
beam_inside_car_ldlrs( double x, double y )
{
  if ( fabs(y) < ((DGC_PASSAT_WIDTH+0.3)/2)    &&
      x >  -DGC_PASSAT_IMU_TO_R_BUMPER        &&
      x <  0.5 + DGC_PASSAT_LENGTH - DGC_PASSAT_IMU_TO_R_BUMPER ) {
    return(1);
  } else {
    return(0);
  }
}

void 
integrate_ldlrs( LdlrsLaser *ldlrs, dgc_transform_t offset,
    unsigned short counter )
{
  int                          i;
  dgc_transform_t              t;
  double                       px, py, pz, a;
  dgc_perception_map_cell_p    cell = NULL; //, terrain_cell = NULL;
  ApplanixPose               * pose;
  float                        td, totalt;
  double                       stime;

  pose = applanix_pose(ldlrs->timestamp);
  if (ldlrs->sector_end_ts>ldlrs->sector_start_ts) {
    totalt = (ldlrs->sector_end_ts-ldlrs->sector_start_ts)/1000.0;
  } else {
    totalt = (SHRT_MAX+ldlrs->sector_end_ts-ldlrs->sector_start_ts)/1000.0;
  }
  td = totalt/(float)ldlrs->num_range;

  stime = ldlrs->timestamp-totalt;

  for(i = 0; i < ldlrs->num_range; i++) {
    if(ldlrs->range[i]>settings.ldlrs_min_dist && 
        ldlrs->range[i]<settings.ldlrs_max_dist) {
      pose = applanix_pose(stime+i*td);
      dgc_transform_identity( t );
      dgc_transform_rotate_x( t, pose->roll );
      dgc_transform_rotate_y( t, pose->pitch );
      dgc_transform_rotate_z( t, pose->yaw );
      dgc_transform_translate( t, pose->smooth_x, pose->smooth_y, pose->smooth_z );
      a = ldlrs->start_angle+i*ldlrs->angular_resolution;
      px = cos(-a)*ldlrs->range[i];
      py = sin(-a)*ldlrs->range[i];
      pz = 0.0;
      dgc_transform_point(&px, &py, &pz, offset);
      if ( px<DGC_PASSAT_LENGTH && !beam_inside_car_ldlrs( px, py ) ) {
        dgc_transform_point(&px, &py, &pz, t);
        cell = (dgc_perception_map_cell_p)grid_get_xy(grid, px, py );
        if (cell!=NULL) {
          cell->hits ++;
          cell->last_observed = counter;
          if (cell->last_obstacle!=counter) {
            obstacles_s->cell[obstacles_s->num] = cell;
            if (obstacles_s->num<MAX_NUM_POINTS)
              obstacles_s->num++;
            cell->last_obstacle = counter;
          }
//          terrain_cell =
//              (dgc_perception_map_cell_p)grid_get_xy(terrain_grid, px, py );
          set_cell_min( cell, pz, counter );
          set_cell_max( cell, pz, counter );
//          set_cell_min( terrain_cell, pz, counter );
//          set_cell_max( terrain_cell, pz, counter );
//          sync_with_terrain( terrain_cell, cell );
        }
      }
    }
  }
}

