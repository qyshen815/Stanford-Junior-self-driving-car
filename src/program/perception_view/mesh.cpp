#include "view.h"

class sick_laser_data : public timed_data {
public:
  sick_laser_data(LaserLaser *l);
  ~sick_laser_data(void);
  LaserLaser laser;
};

sick_laser_data::sick_laser_data(LaserLaser *l)
{
  laser = *l;

  laser.range = (float *)calloc(laser.num_range, sizeof(float));
  dgc_test_alloc(laser.range);
  memcpy(laser.range, l->range, laser.num_range * sizeof(float));
  laser.intensity = (unsigned char *)calloc(laser.num_intensity, 1);
  dgc_test_alloc(laser.intensity);
  memcpy(laser.intensity, l->intensity, laser.num_intensity);
  timestamp = laser.timestamp;
}

sick_laser_data::~sick_laser_data(void)
{
  if(laser.range != NULL)
    free(laser.range);
  if(laser.intensity != NULL)
    free(laser.intensity);
}

void 
laser_add_points(projected_scan_p scan, dgc_pointlist_p pointlist)
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

void 
laser_add_polygons( projected_scan_p old_scan, 
		    projected_scan_p new_scan,
		    dgc_facelist_p facelist )
{
  static dgc_polygon_p poly = NULL;
  static int n = 0;
  int        i;

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

projected_scan_p 
scan_alloc(int num_points)
{
  projected_scan_p scan = NULL;
  
  scan = (projected_scan_p)calloc(1, sizeof(projected_scan_t));
  dgc_test_alloc(scan);
  scan->num_points = num_points;
  scan->point = (vlr::dgc_point3D_p)calloc(num_points, sizeof(vlr::dgc_point3D_t));
  dgc_test_alloc(scan->point);
  scan->range = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->range);
  scan->intensity = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->intensity);
  return scan;
}

void 
laser_scan_copy(projected_scan_p dest, projected_scan_p src)
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

void
check_mesh_data( dgc_laser_mesh_data_p mesh, int size )
{ 
  if (mesh->size!=0 && mesh->size == size) {
    return;
  } else {
    if (size) {
      if (mesh->size>0) {
	free(mesh->scan);
	free(mesh->last_scan);
      }
      mesh->scan      = scan_alloc(size);
      mesh->last_scan = scan_alloc(size);
      mesh->size      = size;
    }
  }
}

void
init_mesh_data( dgc_laser_mesh_data_p mesh ) 
{
  dgc_facelist_new(&(mesh->fl), NUM_MESH_POLYGONS );
  dgc_pointlist_new(&(mesh->pl), NUM_MESH_POINTS );
  mesh->size = 0;
  mesh->scan = NULL;
  mesh->last_scan = NULL;
  mesh->firsttime = TRUE;
  mesh->last_x    = 0;
  mesh->last_y    = 0;
}
