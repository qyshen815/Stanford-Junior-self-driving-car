#ifndef VLR_IMAGERY_H
#define VLR_IMAGERY_H

#include <bil.h>

namespace vlr {

typedef struct {
  int type, x, y, res, zone;
  char zone_letter;
} image_tile_id;

typedef enum {
  BOTTOM_LEFT,
  BOTTOM_RIGHT
} dgc_imagery_rose_location;

#define        DGC_NUM_IMAGERY_TYPES         7
#define        DGC_IMAGERY_TYPE_NONE         0
#define        DGC_IMAGERY_TYPE_COLOR        1
#define        DGC_IMAGERY_TYPE_TOPO         2
#define        DGC_IMAGERY_TYPE_LASER        3
#define        DGC_IMAGERY_TYPE_GSAT         4
#define        DGC_IMAGERY_TYPE_DARPA        5
#define        DGC_IMAGERY_TYPE_BW           6

#define        DGC_IMAGERY_COLOR_MIN_RES     0.25
#define        DGC_IMAGERY_TOPO_MIN_RES      2.0
#define        DGC_IMAGERY_LASER_MIN_RES     0.15
#define        DGC_IMAGERY_GSAT_MIN_RES      0.3
#define        DGC_IMAGERY_DARPA_MIN_RES     0.1
#define        DGC_IMAGERY_BW_MIN_RES        1.0

#define        DGC_IMAGERY_COLOR_MAX_RES     256
#define        DGC_IMAGERY_TOPO_MAX_RES      256
#define        DGC_IMAGERY_LASER_MAX_RES     38.4
#define        DGC_IMAGERY_GSAT_MAX_RES      512
#define        DGC_IMAGERY_DARPA_MAX_RES     512
#define        DGC_IMAGERY_BW_MAX_RES        256

void dgc_imagery_set_offset(int imagery_type, double x_offset, double y_offset);

void dgc_imagery_set_min_resolution(int image_type, double min_resolution);

void dgc_imagery_set_max_resolution(int image_type, double max_resolution);

float lookup_height(shortmap_p ned, double utm_x, double utm_y);

void dgc_imagery_draw_2D(const char* imagery_root, double window_width, double window_height, double zoom, double x_origin,
    double y_origin, double camera_x_offset, double camera_y_offset, const char* utmzone, int prefetch);

void dgc_imagery_draw_3D(const char* imagery_root, double camera_distance, double camera_x_offset,
    double camera_y_offset, double x_origin, double y_origin, const char* utmzone, int flat, double height_factor,
    int prefetch);

void dgc_imagery_cycle_imagery_type(void);
int dgc_imagery_current_imagery_type(void);
void dgc_imagery_set_imagery_type(int imagery_type);
int dgc_imagery_update(void);
void dgc_imagery_stop(void);
void dgc_imagery_draw_compass_rose(double window_width, double window_height, double rotation, double zoom,
                                   dgc_imagery_rose_location location);

} // namespace vlr

#endif

