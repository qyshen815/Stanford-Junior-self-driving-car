#ifndef BIL_H
#define BIL_H

namespace vlr {

typedef struct {
  double min_lat, min_lon, resolution, resolution_degrees;
  int rows, cols;
  unsigned char **map;
} charmap_t, *charmap_p;

typedef struct {
  double min_x, min_y;
  char utmzone[3];
  double x_resolution, y_resolution;
  double min_lat, min_lon, resolution_degrees;
  int rows, cols;
  short int **map;
} shortmap_t, *shortmap_p;

typedef struct {
  float x, y, z;
} vector_t, *vector_p;

typedef struct {
  float x1, y1, x2, y2;
} marchinggrid_line_t, *marchinggrid_line_p;

typedef struct {
  int num_lines;
  marchinggrid_line_p line;
} marchinggrid_results_t, *marchinggrid_results_p;

void marching_squares(shortmap_p map, float isolevel, 
                      marchinggrid_results_p results);

shortmap_p read_shortmap_bil(char *filename);

void shortmap_to_pgm(shortmap_p map, char *filename);

charmap_p read_charmap_bil(char *filename);

void charmap_to_ppm(charmap_p map, char *filename);

vector_t **compute_map_normals(char *map_filename, shortmap_p map);

void free_shortmap(shortmap_p *map);

} // namespace vlr

#endif
