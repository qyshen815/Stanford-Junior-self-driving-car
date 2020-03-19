#ifndef DGC_FACELIST_H
#define DGC_FACELIST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double x, y, z;
  float r, g, b;
  float range;
} dgc_vertex_t, *dgc_vertex_p;

typedef struct {
  int fill;
  dgc_vertex_t v1, v2, v3, v4;
} dgc_polygon_t, *dgc_polygon_p;

typedef struct {
  dgc_polygon_p points;
  int num_points;
  int mark, full;
} dgc_facelist_t, *dgc_facelist_p;

void dgc_facelist_new(dgc_facelist_p points, int num_points);

void dgc_facelist_add_color_points(dgc_facelist_p points, dgc_polygon_p poly,
                                   int n, double max_size);

void dgc_facelist_reset(dgc_facelist_p points);

void dgc_facelist_draw_mesh(dgc_facelist_p points, double origin_x, 
                            double origin_y, double origin_z);

void dgc_facelist_draw_2D(dgc_facelist_p points, 
                          double origin_x, double origin_y);

#ifdef __cplusplus
}
#endif

#endif
