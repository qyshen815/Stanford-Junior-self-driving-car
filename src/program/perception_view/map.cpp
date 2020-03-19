#include "view.h"

void
draw_quad( float x, float y, float z, float d )
{
  myglVertex3f( x-d, y-d, z );
  myglVertex3f( x-d, y+d, z );
  myglVertex3f( x+d, y+d, z );
  myglVertex3f( x+d, y-d, z );
}

void
draw_bbox( dgc_grid_p map, ApplanixPose *pose )
{
  float             sx, sy, ex, ey;

  if (show_inverse) {
    glColor3f(0.7, 0.7, 0.7);
  } else {
    glColor3f(0.4, 0.4, 0.4);
  }

  sx = map->map_c0 * map->resolution - pose->smooth_x;
  sy = map->map_r0 * map->resolution - pose->smooth_y;

  ex = (map->map_c0+map->cols) * map->resolution - pose->smooth_x;
  ey = (map->map_r0+map->rows) * map->resolution - pose->smooth_y;
  
  glBegin(GL_LINES);
  
  myglVertex3f( sx, sy, -DGC_PASSAT_HEIGHT );
  myglVertex3f( ex, sy, -DGC_PASSAT_HEIGHT );

  myglVertex3f( ex, sy, -DGC_PASSAT_HEIGHT );
  myglVertex3f( ex, ey, -DGC_PASSAT_HEIGHT );

  myglVertex3f( ex, ey, -DGC_PASSAT_HEIGHT );
  myglVertex3f( sx, ey, -DGC_PASSAT_HEIGHT );

  myglVertex3f( sx, ey, -DGC_PASSAT_HEIGHT );
  myglVertex3f( sx, sy, -DGC_PASSAT_HEIGHT );

  glEnd();
}

inline void 
draw_map_quad( float x, float y, float s, unsigned char type )
{
  dgc_rgb_t  rgb1={0.0,0.0,0.0}, rgb2;
  
  switch(type) {
  case PERCEPTION_MAP_OBSTACLE_LOW:
    rgb2.r = 1.0;
    rgb2.g = 0.5;
    rgb2.b = 0.0;
    draw_cube( x, y, 0.0-DGC_PASSAT_HEIGHT, 0.4-DGC_PASSAT_HEIGHT, s, rgb1, rgb2, 0.75 );
    occupied_map_cells++;
    break;
  case PERCEPTION_MAP_OBSTACLE_HIGH:
    rgb2.r = 1.0;
    rgb2.g = 1.0;
    rgb2.b = 0.0;
    draw_cube( x, y, 0.0-DGC_PASSAT_HEIGHT, 1.0-DGC_PASSAT_HEIGHT, s, rgb1, rgb2, 0.75 );
    occupied_map_cells++;
    break;
  case PERCEPTION_MAP_OBSTACLE_UNKNOWN:
    rgb2.r = 0.0;
    rgb2.g = 1.0;
    rgb2.b = 0.0;
    draw_cube( x, y, 0.0-DGC_PASSAT_HEIGHT, 2.0-DGC_PASSAT_HEIGHT, s, rgb1, rgb2, 0.75 );
    break;
  default:
    break;
  }
}
  
inline void 
draw_map_quad_simple( float x, float y, float s, unsigned char type )
{
  glColor3f(0.0,1.0,0.2);
  switch(type) {
  case PERCEPTION_MAP_OBSTACLE_LOW:
    draw_quad( x, y, 0.4, s );
    occupied_map_cells++;
    break;
  case PERCEPTION_MAP_OBSTACLE_HIGH:
    draw_quad( x, y, 0.4, s );
    occupied_map_cells++;
    break;
  case PERCEPTION_MAP_OBSTACLE_UNKNOWN:
    draw_quad( x, y, 0.4, s );
    break;
  default:
    break;
  }
}
  
void 
draw_grid_map( dgc_grid_p map, ApplanixPose *pose,
	       char *data, short offset )
{
  float                        sx, sy, gx, gy, smooth_x, smooth_y;
  float                        x, y;
  int                          r, c;
  unsigned char               *ch;
  float                        res = map->resolution;

  glPushMatrix(); 

  draw_bbox( map, pose );

  glBegin(GL_QUADS);

  smooth_x = pose->smooth_x;
  smooth_y = pose->smooth_y;

  if (show_inverse)
    glColor3f(0.0, 0.0, 1.0);
  else
    glColor3f(1.0, 1.0, 0.0);

  sx = map->map_c0 * res - smooth_x;
  sy = map->map_r0 * res - smooth_y;
  gx = sx + (map->cols-map->array_c0) * res;
  gy = sy + (map->rows-map->array_r0) * res;

  occupied_map_cells = 0;
  for (c=0; c<map->cols; c++) { 
    for (r=0; r<map->rows; r++) { 
      ch = (unsigned char *)(data+(map->bytes_per_cell*(r*map->cols+c))+offset);
      if ( *ch==PERCEPTION_MAP_OBSTACLE_LOW  ||
	   *ch==PERCEPTION_MAP_OBSTACLE_HIGH ||
	   *ch==PERCEPTION_MAP_OBSTACLE_UNKNOWN ) {
	if (c<map->array_c0) {
	  x = gx+(c+0.5)*res;
	} else {
	  x = sx+(c+0.5-map->array_c0)*res;
	}
	if (r<map->array_r0) {
	  y = gy+(r+0.5)*res;
	} else {
	  y = sy+(r+0.5-map->array_r0)*res;	
	}
	draw_map_quad( x, y, 2*res-0.04, *ch );
      }
    }
  }

  glEnd();
  glPopMatrix(); 
}

