#ifndef DGC_PROJECT_SENSORS_H
#define DGC_PROJECT_SENSORS_H

#include <roadrunner.h>
#include <laser_interface.h>
#include <ibeo_interface.h>
#include <ldlrs_interface.h>
#include <riegl_interface.h>
#include <transform.h>
#include <pointlist.h>
#include <facelist.h>
#include <gl_support.h>
#include <project.h>

typedef struct {
  int num_points;
  dgc_point3D_p point;
  float *range, *intensity;
} projected_scan_t, *projected_scan_p;

typedef struct {
  char level, status;
  double x, y, z;
} projected_ibeo_point_t, *projected_ibeo_point_p;

typedef struct {
  int num_points;
  projected_ibeo_point_p point;
} projected_ibeo_t, *projected_ibeo_p;

class sick_laser_data : public timed_data {
public:
  sick_laser_data(dgc_laser_laser_message *l);
  ~sick_laser_data(void);
  dgc_laser_laser_message laser;
};

class riegl_laser_data : public timed_data {
public:
  riegl_laser_data(dgc_riegl_laser_message *l);
  ~riegl_laser_data(void);
  dgc_riegl_laser_message laser;
};

void
project_riegl(reference_queue *pose_queue, sensor_queue *queue, 
	      dgc_transform_t offset, projected_scan_p *scan, 
	      projected_scan_p *last_scan, double *last_x, double *last_y,
	      dgc_pointlist_p pl, dgc_facelist_p fl);

void
project_sick(reference_queue *pose_queue, sensor_queue *queue, 
	     dgc_transform_t offset, projected_scan_p *scan, 
	     projected_scan_p *last_scan, double *last_x,
	     double *last_y, dgc_pointlist_p pl, dgc_facelist_p fl);

void
project_ldlrs(dgc_ldlrs_laser_message *l, dgc_transform_t offset,
	      double x, double y, double z, double roll, double pitch,
	      double yaw, projected_scan_p *scan);

void
project_ibeo(dgc_ibeo_laser_message *l, dgc_transform_t offset,
	     double x, double y, double z, double roll, double pitch,
	     double yaw, projected_ibeo_p *projected);

#endif
