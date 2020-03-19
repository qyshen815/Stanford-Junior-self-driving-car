#include "view.h"
#include <limits.h>

#define LMS_MAX_RANGE     75.0

void 
draw_lms( LaserLaser *laser,
	  ApplanixPose *cur_pose __attribute__ ((unused)),
	  dgc_transform_t offset,
	  dgc_laser_mesh_data_p mesh, pthread_mutex_t *mutex )
{
  ApplanixPose                * pose;
  dgc_transform_t               t;
  double                        angle, x, y, z;
  int                           i;

  pthread_mutex_lock(mutex);
  pose = applanix_pose(laser->timestamp);

  /* compute the global pose of the laser */
  dgc_transform_copy(t, offset);
  dgc_transform_rotate_x(t, pose->roll);
  dgc_transform_rotate_y(t, pose->pitch);
  dgc_transform_rotate_z(t, pose->yaw);
  
  check_mesh_data( mesh, laser->num_range );

  if (mesh->size==0) {
    pthread_mutex_unlock(mutex);
    return;
  }
  
  /* project the new scan */
  for(i = 0; i < laser->num_range; i++) {
    angle = -laser->fov / 2.0 + 
      laser->fov * i / (double)(laser->num_range - 1);
    x = laser->range[i] * cos(angle);
    y = laser->range[i] * sin(angle);
    z = 0.0;
    dgc_transform_point(&x, &y, &z, t);
    
    mesh->scan->point[i].x = x + pose->smooth_x;
    mesh->scan->point[i].y = y + pose->smooth_y;
    mesh->scan->point[i].z = z + pose->smooth_z;
    mesh->scan->range[i]   = laser->range[i];

    if(laser->num_intensity == laser->num_range) {
      mesh->scan->intensity[i] = 
	pow(laser->intensity[i] / MAX_INTENSITY, 1 / 1.0);
    } else {
      mesh->scan->intensity[i] = 1;
    }
  }

  laser_add_points(mesh->scan, &(mesh->pl));
  
  if(hypot(pose->smooth_x - mesh->last_x, 
	   pose->smooth_y - mesh->last_y) >= MIN_DIST_BETWEEN_SCANS) {
    /* turn last two scans into polygons */
    if(mesh->firsttime) {
      mesh->firsttime = FALSE;
    } else {
      laser_add_polygons(mesh->last_scan, mesh->scan, &(mesh->fl));
    }
    laser_scan_copy(mesh->last_scan, mesh->scan);
    mesh->last_x = pose->smooth_x;
    mesh->last_y = pose->smooth_y;
  }

  if (show_intensity) {
    dgc_facelist_draw_mesh(&(mesh->fl), 
			   pose->smooth_x, 
			   pose->smooth_y, 
			   pose->smooth_z );
  } else {
    
    dgc_pointlist_draw(&(mesh->pl), 1, 1, 1, 
		       pose->smooth_x, 
		       pose->smooth_y, 
		       pose->smooth_z );
  }

  pthread_mutex_unlock(mutex);
}

void
draw_lms( LaserLaser  *laser, 
	  ApplanixPose *cur_pose,
	  dgc_transform_t offset, pthread_mutex_t *mutex )
{
  ApplanixPose                 *pose;
  dgc_transform_t               t;
  int                           i, s_i;
  double                        a, px, py, pz, sx = 0, sy = 0, sz = 0;
  float                         v;

  pose = applanix_pose(laser->timestamp);

  dgc_transform_copy( t, offset );
  dgc_transform_rotate_x(t, pose->roll);
  dgc_transform_rotate_y(t, pose->pitch);
  dgc_transform_rotate_z(t, pose->yaw);
  dgc_transform_translate
    (t, 
     pose->smooth_x-cur_pose->smooth_x,
     pose->smooth_y-cur_pose->smooth_y,
     pose->smooth_z-cur_pose->smooth_z);

  glPushMatrix();
  if (show_point_size>1) {
    glPointSize(7.0);
  } else {
    glPointSize(3.0);
  }
  if (plain_mode)
    glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);

  if (show_intensity && laser->num_intensity == laser->num_range)
    s_i = 1;
  else
    s_i = 0;

  if (show_beams) { 
    sx = 0; sy = 0; sz = 0;
    dgc_transform_point(&sx, &sy, &sz, t);
    glBegin(GL_LINES);
  } else {
    glBegin(GL_POINTS);
  }
  pthread_mutex_lock( mutex );
  for (i=0; i<laser->num_range; i++) {
    if (laser->range[i]<LMS_MAX_RANGE) {
      if (s_i) {
	v = laser->intensity[i]/255.0;
	glColor3f( v, v, v );
      }
      if (show_beams) { 
	myglVertex3f( sx, sy, sz );
      }
      a = -laser->fov / 2.0 + 
	laser->fov * i / (double)(laser->num_range - 1);
      px = laser->range[i] * cos(a);
      py = laser->range[i] * sin(a);
      pz = 0.0;
      dgc_transform_point(&px, &py, &pz, t);
      myglVertex3f( px, py, pz );
    }
  }
  pthread_mutex_unlock( mutex );
  glEnd();
  glPopMatrix();

}
