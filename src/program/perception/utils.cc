#include "utils.h"
#include "perception.h"
#include "passat_constants.h"

using namespace dgc;

#define AVG_OVER_N_VALUES     5

void
dgc_transform_integrate_pose( dgc_transform_t t, dgc_pose_t pose )
{
  dgc_transform_rotate_x( t, pose.roll );
  dgc_transform_rotate_y( t, pose.pitch );
  dgc_transform_rotate_z( t, pose.yaw );
  dgc_transform_translate( t, pose.x, pose.y, pose.z );
}

void cell_to_coord( dgc_grid_p  grid, dgc_perception_map_cell_p cell, double *x, double *y)
{
  short r, c;
  cell_to_coord(grid, cell, &r, &c);
  grid_rc_to_xy(grid, c, r, x, y);
}

void
cell_to_coord( dgc_grid_p  grid,  dgc_perception_map_cell_p cell, short *x, short *y )
{
  int n = ((char*)(cell) - (char*)(grid->cell))/grid->bytes_per_cell;
  int r = n/grid->cols;
  int c = n-r*grid->cols;
  
  r -= grid->array_r0;
  c -= grid->array_c0;;
  if (r<0) r += grid->rows;
  if (c<0) c += grid->cols;
  
  *y = r;
  *x = c;
}


void
perception_map_reset( dgc_grid_p map )
{ 
  int r,c;
  dgc_perception_map_cell_p   cell;
  for (r=0; r<map->rows; r++) {
    for (c=0; c<map->cols; c++) {
      cell = (dgc_perception_map_cell_p)((char *)(map->cell)+map->bytes_per_cell*(r*map->cols+c));
      memcpy( cell, map->default_value, sizeof(dgc_perception_map_cell_t) );
    }
  }
}

int
inside_car( float x, float y, dgc_pose_t robot )
{
  if (hypotf(robot.x-x,robot.y-y)<3.0) {
    return(TRUE);
  } else {
    return(FALSE);
  }
}

float
cell_height( dgc_perception_map_cell_p cell )
{
  return(cell->max-cell->min);
}

unsigned char
cell_eval( dgc_perception_map_cell_p cell )
{
  float h, seen;
  unsigned char  t;
  if (cell->seen==0) {
    seen = 1;
  } else {
    seen = cell->seen;
  }
  if ((cell->hits > settings.map_cell_min_hits) || (cell->hits/seen > settings.map_cell_threshold)) {
    h = cell_height(cell);
    if (h<0.4) {    
      t = PERCEPTION_MAP_OBSTACLE_LOW;
    } else if (h<1.8) {
      t = PERCEPTION_MAP_OBSTACLE_HIGH;
    } else {
      t = PERCEPTION_MAP_OBSTACLE_UNKNOWN;
    }
  } else {
    t = PERCEPTION_MAP_OBSTACLE_FREE;
  }
  return t;
}

int
change_cell( dgc_perception_map_cell_p cell, unsigned short counter )
{ 
  if (cell->last_mod==counter)
    return 0;
  if (cell->obstacle != cell_eval(cell)) {
    cell->last_mod = counter;
    return 1;
  }
  return 0;
}

char 
rotor( void )
{ 
  static int r = 0;
  int    rotor_chr[4] = { 47, 45, 92, 124 };
  if (++r>3) r = 0;
  return(rotor_chr[r]);
}

double
avg_time( double time, int cnt )
{
  static int     avgctr = 0, avgnum, idx;
  static double  avgtime = 0.0, avgtimes[AVG_OVER_N_VALUES];
  static int     avgcnt = 0, avgcnts[AVG_OVER_N_VALUES];
  
  if (avgctr<AVG_OVER_N_VALUES) {
    avgnum = avgctr+1;
    idx = avgctr;
  } else {
    avgnum = AVG_OVER_N_VALUES;
    idx = avgctr%AVG_OVER_N_VALUES;
    avgtime -= avgtimes[idx];
    avgcnt -= avgcnts[idx];
  }
  avgtimes[idx] = time;
  avgcnts[idx]  = cnt;
  avgtime += time;
  avgcnt  += cnt;
  avgctr++;
  return( avgcnt / avgtime );
}

unsigned short
counter_diff( unsigned short last, unsigned short now )
{
  if (now < last) {
    return(USHRT_MAX - (last - now));
  }
  return(now - last);
}

double 
beam_dist( dgc_velodyne_point_p pt1, dgc_velodyne_point_p pt2 )
{
  int dx = pt1->x - pt2->x;
  int dy = pt1->y - pt2->y;
  int dz = pt1->z - pt2->z;
  return(sqrt(dx*dx+dy*dy+dz*dz));
}

double
pose_dist( ApplanixPose *p1, ApplanixPose *p2 )
{
  return(hypotf(p2->smooth_x-p1->smooth_x,
		p2->smooth_y-p1->smooth_y));
}

double sample(double b) {
  double sum = 0;
  int i;
  for(i = 0; i < 12; i++)
    sum += ((double)(rand()) / RAND_MAX) * 2 * b - b;
  return .5 * sum;
}

