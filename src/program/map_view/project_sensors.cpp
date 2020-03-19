#include <roadrunner.h>
#include "project_sensors.h"

#define     MAX_INTENSITY              255.0
#define     MIN_DIST_BETWEEN_SCANS     0.1
#define     MAX_FILL_SIZE              1.0

projected_scan_p scan_alloc(int num_points)
{
  projected_scan_p scan = NULL;
  
  scan = (projected_scan_p)calloc(1, sizeof(projected_scan_t));
  dgc_test_alloc(scan);
  scan->num_points = num_points;
  scan->point = (dgc_point3D_p)calloc(num_points, sizeof(dgc_point3D_t));
  dgc_test_alloc(scan->point);
  scan->range = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->range);
  scan->intensity = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->intensity);
  return scan;
}

void scan_copy(projected_scan_p dest, projected_scan_p src)
{
  int i;

  if(src->num_points != dest->num_points)
    dgc_die("Error: source and destination scans are different sizes.\n");

  for(i = 0; i < src->num_points; i++) {
    dest->point[i] = src->point[i];
    dest->range[i] = src->range[i];
    dest->intensity[i] = src->intensity[i];
  }
}

void project_ldlrs_scan(dgc_ldlrs_laser_message *l, 
			dgc_transform_t t, projected_scan_p scan)
{
  double angle, px, py, pz;
  int i;

  for(i = 0; i < l->num_range; i++) {
     angle = l->start_angle + i * l->angular_resolution;
     px = cos(-angle) * l->range[i];
     py = sin(-angle) * l->range[i];
     pz = 0.0;
     dgc_transform_point(&px, &py, &pz, t);
     scan->point[i].x = px;
     scan->point[i].y = py;
     scan->point[i].z = pz;
  }
}

void project_ldlrs(dgc_ldlrs_laser_message *l, dgc_transform_t offset,
		   double x, double y, double z, double roll, double pitch,
		   double yaw, projected_scan_p *scan)
{
  dgc_transform_t t;

  /* allocate the projected scans */
  if(*scan == NULL) {
    *scan = scan_alloc(l->num_range);
    free((*scan)->range);
    free((*scan)->intensity);
  }
  else {
    (*scan)->num_points = l->num_range;
    (*scan)->point = 
      (dgc_point3D_p)realloc((*scan)->point, (*scan)->num_points * 
			     sizeof(dgc_point3D_t));
    if((*scan)->num_points > 0)
      dgc_test_alloc((*scan)->point);
  }

  /* compute the global pose of the laser */
  dgc_transform_copy(t, offset);
  dgc_transform_rotate_x(t, roll);
  dgc_transform_rotate_y(t, pitch);
  dgc_transform_rotate_z(t, yaw);
  dgc_transform_translate(t, x, y, z);

  project_ldlrs_scan(l, t, *scan);
}

void project_ibeo(dgc_ibeo_laser_message *l, dgc_transform_t offset,
		  double x, double y, double z, double roll, double pitch,
		  double yaw, projected_ibeo_p *projected)
{
  dgc_transform_t t;
  int i;

  if(*projected == NULL) {
    *projected = (projected_ibeo_p)calloc(1, sizeof(projected_ibeo_t));
    dgc_test_alloc(*projected);
  }

  /* compute the global pose of the laser */
  dgc_transform_copy(t, offset);
  dgc_transform_rotate_x(t, roll);
  dgc_transform_rotate_y(t, pitch);
  dgc_transform_rotate_z(t, yaw);
    
  (*projected)->num_points = l->num_points;
  (*projected)->point = 
    (projected_ibeo_point_p)realloc((*projected)->point,
				    (*projected)->num_points * 
				    sizeof(projected_ibeo_point_t));
  if((*projected)->num_points > 0)
    dgc_test_alloc((*projected)->point);

  for(i = 0; i < (*projected)->num_points; i++) {
    (*projected)->point[i].x = l->point[i].x;
    (*projected)->point[i].y = l->point[i].y;
    (*projected)->point[i].z = l->point[i].z;
    (*projected)->point[i].level = l->point[i].level;
    (*projected)->point[i].status = l->point[i].status;
    dgc_transform_point(&(*projected)->point[i].x, &(*projected)->point[i].y,
			&(*projected)->point[i].z, t);
    (*projected)->point[i].x += x;
    (*projected)->point[i].y += y;
    (*projected)->point[i].z += z;
  }
}

void add_points(projected_scan_p scan, dgc_pointlist_p pointlist)
{
  static double *x = NULL, *y = NULL, *z = NULL;
  static float *range = NULL;
  static int n = 0;
  int i;

  if(n < scan->num_points) {
    n = scan->num_points;
    x = (double *)realloc(x, n * sizeof(double));
    dgc_test_alloc(x);
    y = (double *)realloc(y, n * sizeof(double));
    dgc_test_alloc(y);
    z = (double *)realloc(z, n * sizeof(double));
    dgc_test_alloc(z);
    range = (float *)realloc(range, n * sizeof(float));
    dgc_test_alloc(range);
  }
  for(i = 0; i < scan->num_points; i++) {
    x[i] = scan->point[i].x;
    y[i] = scan->point[i].y;
    z[i] = scan->point[i].z;
    range[i] = scan->range[i];
  }
  dgc_pointlist_add_clipped_points(pointlist, x, y, z, range, 
                                   scan->num_points, 0.5, 40.0);
}

void add_polygons(projected_scan_p old_scan, projected_scan_p new_scan,
                  dgc_facelist_p facelist)
{
  static dgc_polygon_p poly = NULL;
  static int n = 0;
  int i;

  if(old_scan->num_points != new_scan->num_points)
    dgc_die("Error: add_new_polygons called with different sized scans.\n");

  if(n < new_scan->num_points) {
    n = new_scan->num_points;
    poly = (dgc_polygon_p)realloc(poly, n * sizeof(dgc_polygon_t));
    dgc_test_alloc(poly);
  }

  for(i = 0; i < new_scan->num_points - 1; i++) {
    poly[i].v1.x = new_scan->point[i].x;
    poly[i].v1.y = new_scan->point[i].y;
    poly[i].v1.z = new_scan->point[i].z;
    poly[i].v1.range = new_scan->range[i];
    poly[i].v1.r = poly[i].v1.g = poly[i].v1.b = new_scan->intensity[i];

    poly[i].v2.x = new_scan->point[i + 1].x;
    poly[i].v2.y = new_scan->point[i + 1].y;
    poly[i].v2.z = new_scan->point[i + 1].z;
    poly[i].v2.range = new_scan->range[i + 1];
    poly[i].v2.r = poly[i].v2.g = poly[i].v2.b = new_scan->intensity[i + 1];

    poly[i].v3.x = old_scan->point[i + 1].x;
    poly[i].v3.y = old_scan->point[i + 1].y;
    poly[i].v3.z = old_scan->point[i + 1].z;
    poly[i].v3.range = old_scan->range[i + 1];
    poly[i].v3.r = poly[i].v3.g = poly[i].v3.b = old_scan->intensity[i + 1];

    poly[i].v4.x = old_scan->point[i].x;
    poly[i].v4.y = old_scan->point[i].y;
    poly[i].v4.z = old_scan->point[i].z;
    poly[i].v4.range = old_scan->range[i];
    poly[i].v4.r = poly[i].v4.g = poly[i].v4.b = old_scan->intensity[i];
  }
  dgc_facelist_add_color_points(facelist, poly, new_scan->num_points, 
                                MAX_FILL_SIZE);
}

void project_riegl_scan(dgc_riegl_laser_message *laser, dgc_transform_t t,
                        projected_scan_p scan, double x, double y, double z)
{
  int i;
  double angle;

  for(i = 0; i < laser->num_range; i++) {
    angle = -laser->fov / 2.0 + 
      laser->fov * i / (double)(laser->num_range - 1);
    scan->point[i].x = laser->range[i] * cos(angle);
    scan->point[i].y = laser->range[i] * sin(angle);
    scan->point[i].z = 0.0;
    dgc_transform_point(&scan->point[i].x, &scan->point[i].y, 
                        &scan->point[i].z, t);
    scan->point[i].x += x;
    scan->point[i].y += y;
    scan->point[i].z += z;
    scan->range[i] = laser->range[i];

    if(laser->num_intensity == laser->num_range)
      scan->intensity[i] = pow(laser->intensity[i] / MAX_INTENSITY, 1 / 1.0);
    else
      scan->intensity[i] = 1;
  }
}

void project_riegl(reference_queue *pose_queue, sensor_queue *queue, 
		   dgc_transform_t offset, projected_scan_p *scan, 
		   projected_scan_p *last_scan, double *last_x, double *last_y,
		   dgc_pointlist_p pl, dgc_facelist_p fl)
{
  riegl_laser_data *l;
  dgc_transform_t t;
  pose_data pose;
  int first = 0;

  while(queue->posestamped_data(pose_queue, (timed_data **)&l, &pose) != -1) {
    /* compute the global pose of the laser */
    dgc_transform_copy(t, offset);
    dgc_transform_rotate_x(t, pose.roll);
    dgc_transform_rotate_y(t, pose.pitch);
    dgc_transform_rotate_z(t, pose.yaw);
    
    /* allocate the projected scans */
    if(*scan == NULL) {
      *scan = scan_alloc(l->laser.num_range);
      *last_scan = scan_alloc(l->laser.num_range);
      first = 1;
    }
    /* project the new scan */
    project_riegl_scan(&(l->laser), t, *scan, pose.x, pose.y, pose.z);
    add_points(*scan, pl);

    if(hypot(pose.x - *last_x, pose.y - *last_y) >= MIN_DIST_BETWEEN_SCANS) {
      /* turn last two scans into polygons */
      if(!first)
	add_polygons(*last_scan, *scan, fl);
      scan_copy(*last_scan, *scan);
      *last_x = pose.x;
      *last_y = pose.y;
    }
    delete l;
  }
}

void project_sick_scan(dgc_laser_laser_message *laser, dgc_transform_t t,
                       projected_scan_p scan, double x, double y, double z)
{
  int i;
  double angle;

  for(i = 0; i < laser->num_range; i++) {
    angle = -laser->fov / 2.0 + 
      laser->fov * i / (double)(laser->num_range - 1);
    scan->point[i].x = laser->range[i] * cos(angle);
    scan->point[i].y = laser->range[i] * sin(angle);
    scan->point[i].z = 0.0;
    dgc_transform_point(&scan->point[i].x, &scan->point[i].y, 
                        &scan->point[i].z, t);
    scan->point[i].x += x;
    scan->point[i].y += y;
    scan->point[i].z += z;
    scan->range[i] = laser->range[i];

    if(laser->num_intensity == laser->num_range)
      scan->intensity[i] = pow(laser->intensity[i] / MAX_INTENSITY, 1 / 1.0);
    else
      scan->intensity[i] = 1;
  }
}

void project_sick(reference_queue *pose_queue, sensor_queue *queue, 
		  dgc_transform_t offset, projected_scan_p *scan, 
		  projected_scan_p *last_scan, double *last_x,
		  double *last_y, dgc_pointlist_p pl, dgc_facelist_p fl)
 
{
  sick_laser_data *l;
  dgc_transform_t t;
  pose_data pose;
  int first = 0;

  while(queue->posestamped_data(pose_queue, (timed_data **)&l, &pose) != -1) {
    /* compute the global pose of the laser */
    dgc_transform_copy(t, offset);
    dgc_transform_rotate_x(t, pose.roll);
    dgc_transform_rotate_y(t, pose.pitch);
    dgc_transform_rotate_z(t, pose.yaw);
    
    /* allocate the projected scans */
    if(*scan == NULL) {
      *scan = scan_alloc(l->laser.num_range);
      *last_scan = scan_alloc(l->laser.num_range);
      first = 1;
    }
    /* project the new scan */
    project_sick_scan(&(l->laser), t, *scan, pose.x, pose.y, pose.z);
    
    add_points(*scan, pl);
    /* turn last two scans into polygons */
    if(hypot(pose.x - *last_x, pose.y - *last_y) >= MIN_DIST_BETWEEN_SCANS) {
      if(!first)
        add_polygons(*last_scan, *scan, fl);
      scan_copy(*last_scan, *scan);
      *last_x = pose.x;
      *last_y = pose.y;
    }
    delete l;
  }
}

