#include "perception.h"
#include "utils.h"

using namespace dgc;
using namespace vlr;

#define MAX_NUM_LINE_GRID_POINTS   1000

double round(double x);

void
free_space_ray_tracing( unsigned short counter )
{
  static int                             firsttime = 1;
  static PerceptionScan                  vscan;
  static grid_line_t                     line;

  float                       angle, dist;
  short                       px, py;
  double                      x, y, ll_x, ll_y;
  int                         i, l, b = 0;
  double                      vscan_res = dgc_d2r(PERCEPTION_RAY_TRACING_RES);
  ApplanixPose               *pose = applanix_current_pose();
  ivec2_t                     s, e;
  dgc_perception_map_cell_p   cell = NULL;

  if (firsttime) {
    vscan.num = (int) ceil(2*M_PI/vscan_res);
    vscan.p = (PerceptionScanPoint *) 
	    malloc(vscan.num*sizeof(PerceptionScanPoint));
    dgc_test_alloc(vscan.p);
    line.max      = MAX_NUM_LINE_GRID_POINTS;
    line.numgrids = 0;
    line.grid     = (ivec2_p) malloc(line.max*sizeof(ivec2_t));
    firsttime = 0;

  }

  if (obstacles_s->num==0) {
    return;
  }

  vscan.origin_x = pose->smooth_x;
  vscan.origin_y = pose->smooth_y;

  for (i=0; i<vscan.num; i++) {
    vscan.p[i].dist  = FLT_MAX;
    vscan.p[i].angle = i * vscan_res;
    vscan.p[i].x     = vscan.origin_x+
        PERCEPTION_RAY_TRACING_MAX*cos(vscan.p[i].angle);
    vscan.p[i].y     = vscan.origin_y+
        PERCEPTION_RAY_TRACING_MAX*sin(vscan.p[i].angle);
  }

  ll_x  = (grid->map_c0)*grid->resolution;
  ll_y  = (grid->map_r0)*grid->resolution;

  for (i=0; i<obstacles_s->num; i++) {
    cell_to_coord( grid,  obstacles_s->cell[i], &px, &py );
    x     = ll_x + (px+0.5) * grid->resolution;
    y     = ll_y + (py+0.5) * grid->resolution;
    dist  = hypot( x-vscan.origin_x, y-vscan.origin_y );
    angle = atan2( y-vscan.origin_y, x-vscan.origin_x );
    b     = (vscan.num + (int) (angle/vscan_res)) % vscan.num;
    if (dist<vscan.p[b].dist) {
      vscan.p[b].dist  = dist;
      vscan.p[b].x     = x;
      vscan.p[b].y     = y;
    }
  }

  if (settings.gls_output && settings.show_ray_tracing) {
    /* setup GLS header */
    gls->coordinates = GLS_SMOOTH_COORDINATES;
    gls->origin_x = 0.0;
    gls->origin_y = 0.0;
    gls->origin_z = applanix_current_pose()->smooth_z+0.6-DGC_PASSAT_HEIGHT;
    /* draw the candidates */
    glsPushMatrix(gls);
    glsLineWidth(gls, 1.0);
    glsColor3f(gls, 0.0, 1.0, 0.0);
    glsBegin(gls, GLS_LINES);
  }


  for (b=0; b<vscan.num; b++) {

    x = vscan.origin_x+
        cos(vscan.p[b].angle)*PERCEPTION_RAY_TRACING_MIN_DIST;
    y = vscan.origin_y+
        sin(vscan.p[b].angle)*PERCEPTION_RAY_TRACING_MIN_DIST;

    if (settings.gls_output && settings.show_ray_tracing) {
      glsVertex3f(gls, x, y, 0.0 );
    }

    s.x = (int)(x / grid->resolution);
    s.y = (int)(y / grid->resolution);

    if (vscan.p[b].dist<PERCEPTION_RAY_TRACING_MAX) {
      x = vscan.origin_x+
          cos(vscan.p[b].angle)*
          (vscan.p[b].dist-PERCEPTION_RAY_TRACING_SHORTEN);
      y = vscan.origin_y+
          sin(vscan.p[b].angle)*
          (vscan.p[b].dist-PERCEPTION_RAY_TRACING_SHORTEN);
    } else {
      x = vscan.origin_x+
          cos(vscan.p[b].angle)*PERCEPTION_RAY_TRACING_FREESPACE;
      y = vscan.origin_y+
          sin(vscan.p[b].angle)*PERCEPTION_RAY_TRACING_FREESPACE;
    }
    e.x = (int)(x / grid->resolution);
    e.y = (int)(y / grid->resolution);

    if (settings.gls_output && settings.show_ray_tracing) {
      glsVertex3f(gls, vscan.p[b].x, vscan.p[b].y, 0.0 );
    } 

    grid_line( s, e, &line );
    for (l=0; l<line.numgrids; l++) {
      x = (line.grid[l].x+0.5)*grid->resolution;
      y = (line.grid[l].y+0.5)*grid->resolution;
      cell = (dgc_perception_map_cell_p)grid_get_xy(grid, x, y );
      if (cell!=NULL) {
        if (cell->hits>=1) {
          cell->hits -= 1;
        }
      }
    }

  }

  if (settings.gls_output && settings.show_ray_tracing) {
    glsEnd(gls);
    glsLineWidth(gls, 1.0);
    glsPopMatrix(gls);
  }

}
