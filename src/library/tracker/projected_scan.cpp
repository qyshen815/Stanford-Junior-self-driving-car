#include <roadrunner.h>
#include <gls_interface.h>
#include "projected_scan.h"

using namespace dgc;

extern GlsOverlay *lasertrack_gls;

projected_scan::projected_scan( void ) 
{
  num_points=0;
  point=NULL;
  timestamp=0;
}

projected_scan::~projected_scan()
{
  if (point) delete point;
  point=NULL;
}

void 
projected_scan::gls_render()
{
  glsPointSize(lasertrack_gls, 3.0);
  glsColor3f(lasertrack_gls, 0, 0, 1);
  glsBegin(lasertrack_gls, GLS_POINTS);
  for(int i = 0; i < num_points; i++) {
    if(!point[i].use) {
      glsVertex3f(lasertrack_gls, 
		      point[i].x - lasertrack_gls->origin_x, 
		      point[i].y - lasertrack_gls->origin_y, 
		      point[i].z - lasertrack_gls->origin_z);
    }
  }
  glsEnd(lasertrack_gls);

  glsColor3f(lasertrack_gls, 1, 1, 1);
  glsBegin(lasertrack_gls, GLS_POINTS);
  for(int i = 0; i < num_points; i++) { 
    if(point[i].use) {
      glsVertex3f(lasertrack_gls, 
		      point[i].x - lasertrack_gls->origin_x, 
		      point[i].y - lasertrack_gls->origin_y, 
		      point[i].z - lasertrack_gls->origin_z);
    }
  }
  glsEnd(lasertrack_gls);
  glsPointSize(lasertrack_gls, 1.0);
}

projected_velodyne::projected_velodyne(PerceptionRobotPose *pose,
				       PerceptionScan *velodyne,
				       dgc_transform_t offset)
{
  dgc_transform_t t;
  int i;

  double x = pose->pose.x; 
  double y = pose->pose.y;
  double z = pose->pose.z;
  double roll = pose->pose.roll;
  double pitch = pose->pose.pitch;
  double yaw = pose->pose.yaw;
  
  /* position and orientation of robot */
  robotPos.v[0] = x;
  robotPos.v[1] = y;
  robotPos.v[2] = z;
  robotOri.v[0] = yaw;
  robotOri.v[1] = pitch;
  robotOri.v[2] = roll;
  

  /* compute the global pose of the laser */
  dgc_transform_copy(t, offset);
  dgc_transform_rotate_x(t, roll);
  dgc_transform_rotate_y(t, pitch);
  dgc_transform_rotate_z(t, yaw);
  dgc_transform_translate(t, x, y, z);

  /* compute position/orientation of the laser origin */
  dgc_transform_point(&laserPos.v[0], &laserPos.v[1], &laserPos.v[2], t);
  Vec3 fwdPos(10,0,0);
  dgc_transform_point(&fwdPos.v[0], &fwdPos.v[1], &fwdPos.v[2], t);
  fwdPos.sub(laserPos);
  laserOri = robotOri;
  laserOri.v[0] = atan2(fwdPos.v[1], fwdPos.v[0]);

  /* compute position of laser endpoints */
  point = new projected_point[velodyne->num];
  num_points = 0;
  for(i = 0; i < velodyne->num; i++) {
      point[num_points].x = velodyne->p[i].x;
      point[num_points].y = velodyne->p[i].y;
      point[num_points].z = 0;
      point[num_points].r = sqrt(dgc_square(point[num_points].x) + 
				 dgc_square(point[num_points].y) + 
				 dgc_square(point[num_points].z));

      //      dgc_transform_point(&point[num_points].x, &point[num_points].y,
      //			  &point[num_points].z, t);
      double r=point[num_points].r;
      point[num_points].use = (r>0 && r<500); //maxfloat means max rng, i.e. free 
      num_points++;
  }

  timestamp = velodyne->timestamp;
}

projected_velodyne::~projected_velodyne()
{
}
