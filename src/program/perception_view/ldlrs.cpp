#include "view.h"
#include <limits.h>

void
draw_ldlrs( LdlrsLaser *ldlrs, 
	    ApplanixPose *cur_pose,
	    dgc_transform_t offset, pthread_mutex_t *mutex )
{
  ApplanixPose                 *pose;
  dgc_transform_t               t;
  double                        a, px, py, pz;
  double                        sx, sy, sz;
  int                           i;
  double                        stime;
  float                         td, totalt;

  if (ldlrs->sector_end_ts>ldlrs->sector_start_ts) {
    totalt = (ldlrs->sector_end_ts-ldlrs->sector_start_ts)/1000.0;
  } else {
    totalt = (SHRT_MAX+ldlrs->sector_end_ts-ldlrs->sector_start_ts)/1000.0;
  }
  td = totalt/(float)ldlrs->num_range;

  stime = ldlrs->timestamp-totalt;

  glPushMatrix();
  if (show_point_size>1) {
    glPointSize(7.0);
  } else {
    glPointSize(3.0);
  }
  if (plain_mode)
    glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);
  if (show_beams) { 
    glBegin(GL_LINES);
  } else {
    glBegin(GL_POINTS);
  }
  pthread_mutex_lock( mutex );
  for(i = 0; i < ldlrs->num_range; i++) {
    pose = applanix_pose(stime+i*td);
    dgc_transform_copy( t, offset );
    dgc_transform_rotate_x( t, pose->roll );
    dgc_transform_rotate_y( t, pose->pitch );
    dgc_transform_rotate_z( t, pose->yaw );
    dgc_transform_translate( t, 
			     pose->smooth_x-cur_pose->smooth_x, 
			     pose->smooth_y-cur_pose->smooth_y, 
			     pose->smooth_z-cur_pose->smooth_z);
    if (show_beams) { 
      sx = 0; sy = 0; sz = 0;
      dgc_transform_point(&sx, &sy, &sz, t);
      myglVertex3f( sx, sy, sz );
    }
    a = ldlrs->start_angle+i*ldlrs->angular_resolution;
    px = cos(-a)*ldlrs->range[i];
    py = sin(-a)*ldlrs->range[i];
    pz = 0.0;
    dgc_transform_point(&px, &py, &pz, t);
    myglVertex3f( px, py, pz );
   
  }

  pthread_mutex_unlock( mutex );
  glEnd();
  glPopMatrix();
}
