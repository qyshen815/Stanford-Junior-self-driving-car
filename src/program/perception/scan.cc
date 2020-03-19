#include "perception.h"
#include "utils.h"

using namespace dgc;
using namespace vlr;

double round(double x);

void
perception_allocate_virtual_scan( PerceptionScan *scan, float resolution )
{
  int  i;

  scan->resolution = resolution;
  scan->num = (int) ceil(2*M_PI/scan->resolution);
  if (scan->p != NULL) {
    free(scan->p);
  }
  scan->p = (PerceptionScanPoint *) malloc(scan->num*sizeof(PerceptionScanPoint));
  dgc_test_alloc(scan->p);
  for (i=0; i<scan->num; i++) {
    scan->p[i].angle = i * scan->resolution;
  }
}

#define MAX_RANGE   150.0

void
perception_compute_virtual_scan( PerceptionScan *scan, 
    dgc_pose_t pose,  dgc_transform_t offset,
    unsigned short counter )
{
  double                       angle, dist;
  double                       ll_x, ll_y;
  int                          i, j, b, b_min, b_max;
  short                        px, py, pxi = 0, pyi = 0;
  double                       x, y, z;
  dgc_transform_t              t;
  ApplanixPose                *ap_pose = applanix_current_pose();

  x = 0.0; y = 0.0; z = 0.0;
  dgc_transform_copy( t, offset );
  dgc_transform_integrate_pose( t, pose );
  dgc_transform_point(&x, &y, &z, t);
  scan->origin_x                = x;
  scan->origin_y                = y;
  pthread_mutex_lock(&applanix_mutex);
  pthread_mutex_lock(&localize_mutex);
  scan->robot.localize.x_offset = localize_pos.x_offset;
  scan->robot.localize.y_offset = localize_pos.y_offset;  
  scan->robot.pose              = pose;
  scan->timestamp		= applanix_current_pose()->timestamp;

  scan->origin_x = global_pos.x-localize_pos.x_offset;
  scan->origin_y = global_pos.y-localize_pos.y_offset;
  scan->robot.localize.x_offset = localize_pos.x_offset;
  scan->robot.localize.y_offset = localize_pos.y_offset;  
  pthread_mutex_unlock(&localize_mutex);
  pthread_mutex_unlock(&applanix_mutex);
  scan->robot.pose.x            = ap_pose->smooth_x;
  scan->robot.pose.y            = ap_pose->smooth_y;
  scan->robot.pose.z            = ap_pose->smooth_z;
  scan->robot.pose.yaw          = ap_pose->yaw;
  scan->robot.pose.pitch        = ap_pose->pitch;
  scan->robot.pose.roll         = ap_pose->roll;
  scan->timestamp      	       	= ap_pose->timestamp;



  for (i=0; i<scan->num; i++) {
    scan->p[i].dist  = FLT_MAX;
    scan->p[i].x     = scan->origin_x;
    scan->p[i].y     = scan->origin_y;
  }

  ll_x = (grid->map_c0)*grid->resolution;
  ll_y = (grid->map_r0)*grid->resolution;

  for (i=0; i<obstacles_s->num; i++) {

    if (cell_eval( obstacles_s->cell[i] ) == PERCEPTION_MAP_OBSTACLE_HIGH) {
      cell_to_coord( grid,  obstacles_s->cell[i], &px, &py );
      b_min = INT_MAX;
      b_max = -INT_MAX;
      x     = ll_x + ( (px-0.5) * grid->resolution );
      y     = ll_y + ( (py-0.5) * grid->resolution );
      dist  = hypot(x-scan->origin_x, y-scan->origin_y);
      for (j=0; j<4; j++) {
        switch(j) {
        case 0:
          pxi = px-1;
          pyi = py-1;
          break;
        case 1:
          pxi = px-1;
          pyi = py+0;
          break;
        case 2:
          pxi = px+0;
          pyi = py+0;
          break;
        case 3:
          pxi = px+0;
          pyi = py-1;
          break;
        }
        x     = ll_x + ( pxi * grid->resolution );
        y     = ll_y + ( pyi * grid->resolution );
        angle = atan2(y-scan->origin_y, x-scan->origin_x);
        b     = (scan->num + (int) (angle/scan->resolution)) % scan->num;
        if (b<b_min) b_min = b;
        if (b>b_max) b_max = b;
      }
      if (b_max-b_min>scan->num/2) {
        for (j=b_max; j<=b_min+scan->num; j++) {
          b = j%scan->num;
          if (dist<scan->p[b].dist) {
            scan->p[b].dist  = dist;
            scan->p[b].x     = scan->origin_x + dist * cos(scan->p[b].angle);
            scan->p[b].y     = scan->origin_y + dist * sin(scan->p[b].angle);
          }
        }
      } else {
        for (j=b_min; j<=b_max; j++) {
          if (dist<scan->p[j].dist) {
            scan->p[j].dist  = dist;
            scan->p[j].x     = scan->origin_x + dist * cos(scan->p[j].angle);
            scan->p[j].y     = scan->origin_y + dist * sin(scan->p[j].angle);
          }
        }
      }
    }
  }

  b_min = (scan->num + (int) (-25.0/scan->resolution)) % scan->num;
  b_max = (scan->num + (int) ( 25.0/scan->resolution)) % scan->num;
  for (j=b_max; j<=b_min; j++) {
    scan->p[j].dist  = 0.0;
  }

  if (settings.gls_output && settings.show_virtual_scan) {
    /* setup GLS header */
    gls->coordinates = GLS_SMOOTH_COORDINATES;
    gls->origin_x = 0;
    gls->origin_y = 0;
    gls->origin_z = applanix_current_pose()->smooth_z;
    /* draw the candidates */
    {
      glsPushMatrix(gls);
      glsTranslatef( gls, 0, 0, -DGC_PASSAT_HEIGHT+0.2 );
      glsBegin(gls, GLS_LINES);
      glsLineWidth(gls, 1.0);
      for (i=0; i<scan->num; i++) {
        glsVertex3f(gls, scan->origin_x, scan->origin_y, 0);
        if (scan->p[i].dist>MAX_RANGE) {
          glsColor3f(gls, 1.0, 1.0, 0.0);
          x = scan->origin_x + MAX_RANGE*cos(scan->p[i].angle);
          y = scan->origin_y + MAX_RANGE*sin(scan->p[i].angle);
          glsVertex3f(gls, x, y, 0);
        } else {
          glsColor3f(gls, 0.0, 1.0, 0.0);
          glsVertex3f(gls, scan->p[i].x, scan->p[i].y, 0);
        }
      }
      glsEnd(gls);
      glsLineWidth(gls, 1.0);
      glsPopMatrix(gls);
    }
    {
      glsPushMatrix(gls);
      glsTranslatef( gls, 0, 0, -DGC_PASSAT_HEIGHT+0.25 );
      glsColor3f(gls, 1.0, 1.0, 0.0);
      glsSquare( gls, scan->origin_x, scan->origin_y, 0.5 );
      glsPopMatrix(gls);
    }
    glsPopMatrix(gls);
  }

  scan->counter = counter;

}
