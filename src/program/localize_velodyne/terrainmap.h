#ifndef DGC_TERRAINMAP_H
#define DGC_TERRAINMAP_H

#define TERRAIN_TILE_SIZE       500
#define TERRAIN_TILE_RESOLUTION 0.15
#define TERRAIN_GRID_NUM_TILES  3

typedef struct {
  float z;
  unsigned int z_count;
  float intensity;
  unsigned int i_count;
  float current_highest;
  float z_response;
  int n;
  float xa;
  float sxi2;
  float stdev;
} terrain_tile_cell;

typedef struct {
  float curb;
  float line_lat;
  float blur;
  float orig;
} vision_tile_cell;

class vision_tile {
public:
  vision_tile(int rows, int cols);
  ~vision_tile();
  int load(char *filename);
  int save(char *filename);

  int rows, cols;
  double utm_x0, utm_y0, resolution, min_z;
  char utmzone[10], filename[200];
  vision_tile_cell **cell;
  unsigned int dl;
};

class terrain_tile {
public:
  terrain_tile(int rows, int cols);
  ~terrain_tile();
  int load(char *filename);
  int save(char *filename);
  void do_average(void);

  int rows, cols;
  double utm_x0, utm_y0, resolution, min_z;
  char utmzone[10], filename[200];
  terrain_tile_cell **cell;
  unsigned int dl;
};

#endif
