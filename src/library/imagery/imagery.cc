#include <roadrunner.h>
#include <gl_support.h>
#include <bil.h>
#include <textures.h>
#include <image.h>
#include <lltransform.h>
#include "texturecache.h"
#include "imagery.h"
#include "compassrose.h"
#include "imagery_proj.h"
#include "imagery_tcp.h"
#include <sys/dir.h>

namespace vlr {

static dgc_gl_texture_t* rose_texture = NULL;

static shortmap_p ned = NULL;

static char *detected_subdir = NULL;

static int has_imagery_type[DGC_NUM_IMAGERY_TYPES];
static int current_imagery_type = DGC_IMAGERY_TYPE_NONE;

static double min_imagery_resolution[DGC_NUM_IMAGERY_TYPES] = {
  DGC_IMAGERY_COLOR_MIN_RES,
  DGC_IMAGERY_COLOR_MIN_RES,
  DGC_IMAGERY_TOPO_MIN_RES,
  DGC_IMAGERY_LASER_MIN_RES,
  DGC_IMAGERY_GSAT_MIN_RES,
  DGC_IMAGERY_DARPA_MIN_RES,
  DGC_IMAGERY_BW_MIN_RES
};

static double max_imagery_resolution[DGC_NUM_IMAGERY_TYPES] = {
  DGC_IMAGERY_COLOR_MAX_RES,
  DGC_IMAGERY_COLOR_MAX_RES,
  DGC_IMAGERY_TOPO_MAX_RES,
  DGC_IMAGERY_LASER_MAX_RES,
  DGC_IMAGERY_GSAT_MAX_RES,
  DGC_IMAGERY_DARPA_MAX_RES,
  DGC_IMAGERY_BW_MAX_RES
};

int isnt_dirlink(de_const_ struct direct *entry)
{
  if(strcmp(entry->d_name, "..") == 0)
    return FALSE;
  else
    return TRUE;
}

int inside_subdir(char *dir, double lat, double lon)
{
  double min_lat, min_lon, max_lat, max_lon;
  char filename[200];
  FILE *fp;

  sprintf(filename, "%s/bound.txt", dir);
  fp = fopen(filename, "r");
  if(fp == NULL)
    return 0;
  if( fscanf(fp, "%lf\n%lf\n%lf\n%lf\n", &min_lat, &min_lon,
	 &max_lat, &max_lon) != 4) {
    dgc_error("Formatting error in bound.txt!");
    return 0;
  }
  fclose(fp);

  if(lat > min_lat && lat < max_lat &&
     lon > min_lon && lon < max_lon)
    return 1;
  return 0;
}

int detect_imagery_subdir(const char* imagery_root, double lat, double lon)
{
  static double last_lat = 0, last_lon = 0, last_check = 0;
  static int firsttime_scandir_error = 1;
  char filename[200], ned_filename[200], dirname[200];
  struct dirent **namelist;
  int n, found_subdir = 0;
  struct stat file_stat;
  static int first = 1;
  double current_time;

  /* dont check more than once per second */
  current_time = dgc_get_time();
  if(current_time - last_check < 1.0) 
    return 0;

  /* only check if it is the first time, or we have jumped a lot */
  if(!first && detected_subdir != NULL && fabs(lat - last_lat) < 0.0005 &&
     fabs(lon - last_lon) < 0.0005) {
    last_lon = lon;
    last_lat = lat;
    last_check = current_time;
    return 0;
  }

  if(strncmp(imagery_root, "http://", 7) == 0) {
    if(detected_subdir == NULL) {
      detected_subdir = (char *)calloc(1, 1);
      dgc_test_alloc(detected_subdir);
      detected_subdir[0] = '\0';
    }
    has_imagery_type[DGC_IMAGERY_TYPE_NONE] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_COLOR] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_TOPO] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_LASER] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_GSAT] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_DARPA] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_BW] = 1;
    dgc_texture_cache_set_version(1);
    return 0;
  }
  else if(strncmp(imagery_root, "tcp://", 6) == 0) {
    if(detected_subdir == NULL) {
      detected_subdir = (char *)calloc(1, 1);
      dgc_test_alloc(detected_subdir);
      detected_subdir[0] = '\0';
    }
    has_imagery_type[DGC_IMAGERY_TYPE_NONE] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_COLOR] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_TOPO] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_LASER] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_GSAT] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_DARPA] = 1;
    has_imagery_type[DGC_IMAGERY_TYPE_BW] = 1;
    dgc_texture_cache_set_version(2);
    return 0;
  }
  else
    dgc_texture_cache_set_version(0);

  if(detected_subdir != NULL) {
    sprintf(filename, "%s/%s", imagery_root, detected_subdir);
    if(inside_subdir(filename, lat, lon)) {
      last_lon = lon;
      last_lat = lat;
      last_check = current_time;
      return 0;
    }
  }

  n = scandir(imagery_root, &namelist, isnt_dirlink, NULL);
  if(n < 0) {
    if (firsttime_scandir_error) {
      perror("scandir");
      firsttime_scandir_error = 0;
    }
  } else {
    while(n--) {
      sprintf(filename, "%s/%s", imagery_root, namelist[n]->d_name);
      stat(filename, &file_stat);
      if(S_ISDIR(file_stat.st_mode)) {
	sprintf(filename, "%s/%s", imagery_root, namelist[n]->d_name);
	if(inside_subdir(filename, lat, lon)) {
	  found_subdir = 1;
	  if(detected_subdir != NULL)
	    free(detected_subdir);
	  detected_subdir = (char *)malloc(strlen(namelist[n]->d_name) + 1);
	  dgc_test_alloc(detected_subdir);
	  strcpy(detected_subdir, namelist[n]->d_name);
	  fprintf(stderr, "INFO: Using %s imagery directory.\n", 
		  detected_subdir);
	  
	  /* load new NED file */
	  if(ned != NULL)
	    free_shortmap(&ned);
	  sprintf(ned_filename, "%s/%s/ned/NED1.BIL", imagery_root, 
		  detected_subdir);
	  ned = read_shortmap_bil(ned_filename);

	  /* check imagery availability */
	  has_imagery_type[DGC_IMAGERY_TYPE_NONE] = 1;

	  sprintf(dirname, "%s/%s/200", imagery_root, 
		  detected_subdir);
	  has_imagery_type[DGC_IMAGERY_TYPE_COLOR] = 
	    has_imagery_type[DGC_IMAGERY_TYPE_TOPO] = dgc_file_exists(dirname);

	  sprintf(dirname, "%s/%s/laser", imagery_root, 
		  detected_subdir);
	  has_imagery_type[DGC_IMAGERY_TYPE_LASER] = dgc_file_exists(dirname);

	  sprintf(dirname, "%s/%s/gsat", imagery_root, 
		  detected_subdir);
	  has_imagery_type[DGC_IMAGERY_TYPE_GSAT] = dgc_file_exists(dirname);

	  sprintf(dirname, "%s/%s/darpa", imagery_root, 
		  detected_subdir);
	  has_imagery_type[DGC_IMAGERY_TYPE_DARPA] = dgc_file_exists(dirname);

	  has_imagery_type[DGC_IMAGERY_TYPE_BW] = 0;
	}
      }
      free(namelist[n]);
    }
    free(namelist);
  }

  first = 0;
  last_check = dgc_get_time();
  last_lon = lon;
  last_lat = lat;
  return 1;
}

void dgc_imagery_cycle_imagery_type(void)
{
  if(detected_subdir == NULL)
    return;
  do {
    current_imagery_type++;
    if(current_imagery_type >= DGC_NUM_IMAGERY_TYPES)
      current_imagery_type = 0;
  } while(!has_imagery_type[current_imagery_type]);
}

int dgc_imagery_current_imagery_type(void)
{
  return current_imagery_type;
}

void dgc_imagery_set_imagery_type(int imagery_type)
{
  current_imagery_type = imagery_type;
}

int detect_imagery_subdir_utm(const char* imagery_root, double utm_x,
			      double utm_y, const char* utmzone)
{
  double lat, lon;

  utmToLatLong(utm_x, utm_y, utmzone, &lat, &lon);
  return detect_imagery_subdir(imagery_root, lat, lon);
}

void fill_rose_texture(void)
{
  rose_texture = dgc_gl_load_texture_rgba_from_bytes(COMPASSROSE_SIZE,
						     compassrose_data,
						     512, 0, 255, 255, 1);
}

void dgc_imagery_stop(void)
{
  dgc_texture_cache_stop();
}

void draw_rose_texture(dgc_gl_texture_t* texture, double x, double y,
                       double rotation, double scale, double zoom)
{
  char str[50];

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(rotation, 0, 0, 1);
  glScalef(scale, scale, scale);
  dgc_gl_draw_texture(texture, -texture->image_width / 2.0, 
		      -texture->image_height / 2.0, texture->image_width / 2.0, 
		      texture->image_height / 2.0, 1);
  glPopMatrix();

  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex2f(x - 65, 10);
  glVertex2f(x + 65, 10);
  glVertex2f(x - 65, 15);
  glVertex2f(x - 65, 5);
  glVertex2f(x + 65, 15);
  glVertex2f(x + 65, 5);
  glEnd();
  glPushMatrix();
  glColor3f(1, 1, 1);
  sprintf(str, "%.2f meters", 130 / zoom);
  glTranslatef(x - stroke_string_width(GLUT_STROKE_ROMAN, str) * 
               0.1 / 2.0, 15, 0);
  glScalef(0.1, 0.1, 1);
  render_stroke_string(GLUT_STROKE_ROMAN, str);
  glPopMatrix();
  glLineWidth(1);
}

void dgc_imagery_draw_compass_rose(double window_width, double window_height,
                                   double rotation, double zoom,
				   dgc_imagery_rose_location location)
{
  if(rose_texture == NULL)
    fill_rose_texture();
  set_display_mode_2D((int)window_width, (int)window_height);
  if(rose_texture != NULL) {
    if(location == BOTTOM_LEFT)
      draw_rose_texture(rose_texture, 70, 80, dgc_r2d(rotation), 0.5, zoom);
    else if(location == BOTTOM_RIGHT)
      draw_rose_texture(rose_texture, window_width - 70, 80, 
			dgc_r2d(rotation), 0.5, zoom);
  }
}

inline float lookup_height(shortmap_p ned, double utm_x, double utm_y)
{
  double x, y, t, u;
  int x1, y1, x2, y2;
  float h;
  
  if(ned == NULL)
    return 0;

  x = (utm_x - ned->min_x) / ned->x_resolution;
  y = (utm_y - ned->min_y) / ned->y_resolution;

  x1 = (int)floor(x);
  y1 = (int)floor(y);
  x2 = x1 + 1;
  y2 = y1 + 1;

  t = x - x1;
  u = y - y1;
  
  if(x1 < 0 || y1 < 0 || x2 >= ned->cols - 1 || y2 >= ned->rows - 1)
    return 0;
  else {
    h = (1 - t) * (1 - u) * ned->map[x1][y1] +
      t * (1 - u) * ned->map[x2][y1] +
      t * u * ned->map[x2][y2] +
      (1 - t) * u * ned->map[x1][y2];
    return h;
  }
}

#define NED_N 30

void draw_wireframe_ned3D(double x1, double y1, double x2, double y2,
                          double origin_x, double origin_y,
                          shortmap_p ned, double height_factor)
{
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;

  xd = (x2 - x1) / NED_N;
  yd = (y2 - y1) / NED_N;

  for(x_i = 0; x_i <= NED_N; x_i++)
    for(y_i = 0; y_i <= NED_N; y_i++) 
      h[x_i][y_i] = lookup_height(ned, origin_x + x1 + xd * x_i, 
                                  origin_y + y1 + yd * y_i) * height_factor;

  glColor3f(1, 1, 1);
  for(x_i = 0; x_i <= NED_N; x_i++) {
    glBegin(GL_LINE_STRIP);
    for(y_i = 0; y_i <= NED_N; y_i++) 
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);
    glEnd();
  }
  
  for(y_i = 0; y_i <= NED_N; y_i++) {
    glBegin(GL_LINE_STRIP);
    for(x_i = 0; x_i <= NED_N; x_i++)
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);
    glEnd();
  }
}

void draw_texture_3D(dgc_gl_texture_t* t, dgc_image_state state,
		     double x1, double y1, double x2, double y2, int smooth)
{
  double dx, dy;

  if(t == NULL)
    return;
  
  if(state == REQUESTED) {
    glBegin(GL_LINE_LOOP);
    glColor3f(0, 0, 1);
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glVertex2f(x1, y2);
    glEnd();
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t->texture_id);

  if(smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t->texture_width);
    dy = 1. / (2 * t->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);

  glTexCoord2f(dx, t->max_v - dy);
  glVertex2f(x1, y1);
  glTexCoord2f(t->max_u - dx, t->max_v - dy);
  glVertex2f(x2, y1);
  glTexCoord2f(t->max_u - dx, dy);
  glVertex2f(x2, y2);
  glTexCoord2f(dx, dy);
  glVertex2f(x1, y2);

  glEnd();
  
  glDisable(GL_TEXTURE_2D);
}

void draw_texture_ned3D(dgc_gl_texture_t* t, dgc_image_state state,
			double x1, double y1, double x2, double y2, 
			double origin_x, double origin_y,
                        int smooth, shortmap_p ned, double height_factor)
{
  double dx, dy;
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;

  if(t == NULL)
    return;
  
  if(state == REQUESTED) {
    draw_wireframe_ned3D(x1, y1, x2, y2, origin_x, origin_y,
                         ned, height_factor);
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t->texture_id);

  if(smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t->texture_width);
    dy = 1. / (2 * t->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  xd = (x2 - x1) / NED_N;
  yd = (y2 - y1) / NED_N;

  for(x_i = 0; x_i <= NED_N; x_i++)
    for(y_i = 0; y_i <= NED_N; y_i++) 
      h[x_i][y_i] = lookup_height(ned, origin_x + x1 + xd * x_i, 
                                  origin_y + y1 + yd * y_i) * height_factor;
  
  glColor3f(1, 1, 1);
  for(x_i = 0; x_i < NED_N; x_i++) {
    glBegin(GL_QUAD_STRIP);
    for(y_i = 0; y_i <= NED_N; y_i++) {
      glTexCoord2f(dx + x_i / (double)NED_N * (t->max_u - 2 * dx),
                   t->max_v - 
                   (dy + y_i / (double)NED_N * (t->max_v - 2 * dy)));
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);
      
      glTexCoord2f(dx + (x_i + 1) / (double)NED_N * (t->max_u - 2 * dx),
                   t->max_v - 
                   (dy + y_i / (double)NED_N * (t->max_v - 2 * dy)));
      glVertex3f(x1 + xd * (x_i + 1), y1 + yd * y_i, h[x_i + 1][y_i]);
    }
    glEnd();
  }
  
  glDisable(GL_TEXTURE_2D);
}

void draw_texture_3D_fourpt(dgc_gl_texture_t* t, dgc_image_state state,
			    double x1, double y1, double x2, double y2, 
			    double x3, double y3, double x4, double y4, 
			    int smooth)
{
  double dx, dy;
  
  if(t == NULL)
    return;
  
  if(state == REQUESTED) {
    glBegin(GL_LINE_LOOP);
    glColor3f(0, 0, 1);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glVertex2f(x3, y3);
    glVertex2f(x4, y4);
    glEnd();
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t->texture_id);

  if(smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t->texture_width);
    dy = 1. / (2 * t->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);

  glTexCoord2f(dx, t->max_v - dy);
  glVertex2f(x1, y1);
  glTexCoord2f(t->max_u - dx, t->max_v - dy);
  glVertex2f(x2, y2);
  glTexCoord2f(t->max_u - dx, dy);
  glVertex2f(x3, y3);
  glTexCoord2f(dx, dy);
  glVertex2f(x4, y4);

  glEnd();
  
  glDisable(GL_TEXTURE_2D);
}

void draw_texture_ned3D_fourpt(dgc_gl_texture_t* t, dgc_image_state state,
			       double x1, double y1, double x2, double y2,
			       double x3, double y3, double x4, double y4,
			       double origin_x, double origin_y, int smooth,
			       shortmap_p ned, double height_factor)
{
  double dx, dy;
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;
  double xb, yb, xe, ye, xc, yc, x2_frac, x_frac, y_frac;

  if(t == NULL)
    return;
  
  if(state == REQUESTED) {
    draw_wireframe_ned3D(x1, y1, x2, y2, origin_x, origin_y,
                         ned, height_factor);
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t->texture_id);

  if(smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1 / (2 * t->texture_width);
    dy = 1 / (2 * t->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  xd = (x3 - x1) / NED_N;
  yd = (y3 - y1) / NED_N;

  for(x_i = 0; x_i <= NED_N; x_i++) {
    for(y_i = 0; y_i <= NED_N; y_i++) {
      h[x_i][y_i] = lookup_height(ned, origin_x + x1 + xd * x_i, 
                                  origin_y + y1 + yd * y_i) * height_factor;
    }
  }
  
  glColor3f(1, 1, 1);
  for(x_i = 0; x_i < NED_N; x_i++) {
    x_frac = x_i / (double)NED_N;
    x2_frac = (x_i + 1) / (double)NED_N;

    glBegin(GL_QUAD_STRIP);
    for(y_i = 0; y_i <= NED_N; y_i++) {
      y_frac = y_i / (double)NED_N;

      glTexCoord2f(dx + x_frac * (t->max_u - 2 * dx),
                   t->max_v - (y_frac * (t->max_v - 2 * dy)));

      xb = x1 + x_frac * (x2 - x1);
      yb = y1 + x_frac * (y2 - y1);
      xe = x4 + x_frac * (x3 - x4);
      ye = y4 + x_frac * (y3 - y4);
      xc = xb + y_frac * (xe - xb);
      yc = yb + y_frac * (ye - yb);

      glVertex3f(xc, yc, h[x_i][y_i]);

      glTexCoord2f(dx + x2_frac * (t->max_u - 2 * dx),
                   t->max_v - (y_frac * (t->max_v - 2 * dy)));

      xb = x1 + x2_frac * (x2 - x1);
      yb = y1 + x2_frac * (y2 - y1);
      xe = x4 + x2_frac * (x3 - x4);
      ye = y4 + x2_frac * (y3 - y4);
      xc = xb + y_frac * (xe - xb);
      yc = yb + y_frac * (ye - yb);
      
      glVertex3f(xc, yc, h[x_i + 1][y_i]);
    }
    glEnd();
  }
  
  glDisable(GL_TEXTURE_2D);
}

inline void clamp_tile_interval(int *min, int *max, int w)
{
  int center = (*max + *min) / 2;
  if(*max - *min + 1 > w) {
    *min = center - w / 2;
    *max = *min + w - 1;
  }
}

void draw_terra_tiles(const char* imagery_root, int imagery_type, int min_tile_x,
		      int min_tile_y, int max_tile_x, int max_tile_y, 
		      int terra_zone, int terra_res, double x_origin, 
		      double y_origin, int use_ned, shortmap_p ned, 
		      double height_factor, int fetch_only)
{
  dgc_image_state state;
  double x3, y3, x4, y4;
  dgc_gl_texture_t* t;
  image_tile_id id;
  int r, c;

  id.type = imagery_type;
  id.res = terra_res;
  id.zone = terra_zone;
  id.zone_letter = ' ';
  for(r = min_tile_y; r <= max_tile_y; r++)
    for(c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      t = dgc_texture_cache_get(imagery_root, detected_subdir, id,
				!fetch_only, &state);
      if(!fetch_only) {
	dgc_terra_tile_bounds_utm(c, r, terra_res, &x3, &y3, &x4, &y4);
	if(use_ned)
	  draw_texture_ned3D(t, state, x3 - x_origin, y3 - y_origin, 
			     x4 - x_origin, y4 - y_origin, 
			     x_origin, y_origin, 1, ned, height_factor);
	else
	  draw_texture_3D(t, state, x3 - x_origin, y3 - y_origin, 
			  x4 - x_origin, y4 - y_origin, 1);
      }
    }
}

void draw_laser_tiles(const char* imagery_root, int min_tile_x, int min_tile_y,
		      int max_tile_x, int max_tile_y, const char* utmzone,
		      double image_resolution, double x_origin, double y_origin,
		      int use_ned, shortmap_p ned, double height_factor,
		      int fetch_only)

{
  dgc_image_state state;
  double x3, y3, x4, y4;
  dgc_gl_texture_t* t;
  char *zone_letter;
  image_tile_id id;
  int r, c;

  id.type = DGC_IMAGERY_TYPE_LASER;
  id.res = (int)rint(image_resolution * 100);
  id.zone = strtoul(utmzone, &zone_letter, 10);
  id.zone_letter = zone_letter[0];

  for(r = min_tile_y; r <= max_tile_y; r++)
    for(c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      t = dgc_texture_cache_get(imagery_root, detected_subdir, id,
 				!fetch_only, &state);
      if(!fetch_only) {
	dgc_laser_tile_bounds_utm(c, r, image_resolution, 
				  &x3, &y3, &x4, &y4);
	if(use_ned)
	  draw_texture_ned3D(t, state, x3 - x_origin, y3 - y_origin, 
			     x4 - x_origin, y4 - y_origin, 
			     x_origin, y_origin, 1, ned, height_factor);
	  else
	    draw_texture_3D(t, state, x3 - x_origin, y3 - y_origin, 
			    x4 - x_origin, y4 - y_origin, 1);
      }
    }
}

void draw_gsat_tiles(const char* imagery_root, int min_tile_x, int min_tile_y,
		     int max_tile_x, int max_tile_y, int gmaps_zoom,
		     double x_origin, double y_origin, int use_ned, 
		     shortmap_p ned, double height_factor, int fetch_only)
{
  double x1, y1, x2, y2, x3, y3, x4, y4;
  char utmzone1[10], utmzone2[10], utmzone3[10], utmzone4[10];
  dgc_image_state state;
  dgc_gl_texture_t* t;
  image_tile_id id;
  int r, c;
  
  id.type = DGC_IMAGERY_TYPE_GSAT;
  id.res = gmaps_zoom;
  id.zone = 0;
  id.zone_letter = ' ';
  for(r = min_tile_y; r <= max_tile_y; r++)
    for(c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      t = dgc_texture_cache_get(imagery_root, detected_subdir, id,
				!fetch_only, &state);
      if(!fetch_only) {
	dgc_gmaps_tile_bounds_utm(c, r, gmaps_zoom, &x1, &y1, utmzone1,
				  &x2, &y2, utmzone2, &x3, &y3, utmzone3,
				  &x4, &y4, utmzone4);
	if(strcmp(utmzone1, utmzone2) != 0 ||
	   strcmp(utmzone1, utmzone3) != 0 ||
	   strcmp(utmzone1, utmzone4) != 0)
	  continue;
	if(use_ned)
	  draw_texture_ned3D_fourpt(t, state, 
				    x1 - x_origin, y1 - y_origin, 
				    x2 - x_origin, y2 - y_origin, 
				    x3 - x_origin, y3 - y_origin,  
				    x4 - x_origin, y4 - y_origin, 
				    x_origin, y_origin, 1, ned, 
				    height_factor);
	else
	  draw_texture_3D_fourpt(t, state, x1 - x_origin, y1 - y_origin, 
				 x2 - x_origin, y2 - y_origin, 
				 x3 - x_origin, y3 - y_origin, 
				 x4 - x_origin, y4 - y_origin, 1);
      }
    }
}

void draw_darpa_tiles(const char* imagery_root, int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y,
    int darpa_resolution, double x_origin, double y_origin, int use_ned, shortmap_p ned, double height_factor,
    int fetch_only) {
  double x1, y1, x2, y2, x3, y3, x4, y4;
  dgc_image_state state;
  dgc_gl_texture_t* t;
  image_tile_id id;
  int r, c;

  id.type = DGC_IMAGERY_TYPE_DARPA;
  id.res = darpa_resolution;
  id.zone = 0;
  id.zone_letter = ' ';
  for (r = min_tile_y; r <= max_tile_y; r++)
    for (c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      t = dgc_texture_cache_get(imagery_root, detected_subdir, id, !fetch_only, &state);
      if (!fetch_only) {
        dgc_darpa_tile_bounds_utm(c, r, darpa_resolution, &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4);
        if (use_ned)
          draw_texture_ned3D_fourpt(t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin,
              x3 - x_origin, y3 - y_origin, x4 - x_origin, y4 - y_origin, x_origin, y_origin, 1, ned, height_factor);
        else
          draw_texture_3D_fourpt(t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin, x3 - x_origin,
              y3 - y_origin, x4 - x_origin, y4 - y_origin, 1);
      }
    }
}

inline void imagery_core(const char* imagery_root, double image_resolution,
			 double x_origin, double y_origin, double x1, 
			 double y1, double x2, double y2, const char* utmzone,
			 int use_ned, double height_factor, int fetch_only)
{
  int gmaps_zoom, darpa_resolution, terra_res, terra_zone, max_tiles = 14;
  int min_tile_x, min_tile_y, max_tile_x, max_tile_y;
  double xc, yc;

  dgc_texture_cache_initialize();

  xc = (x1 + x2) / 2.0;
  yc = (y1 + y2) / 2.0;
  detect_imagery_subdir_utm(imagery_root, xc, yc, utmzone);

  if(detected_subdir == NULL)
    return;

  switch(current_imagery_type) {
  case DGC_IMAGERY_TYPE_NONE:
    return;
    break;
  case DGC_IMAGERY_TYPE_COLOR: 
  case DGC_IMAGERY_TYPE_TOPO:
  case DGC_IMAGERY_TYPE_BW:
    if(dgc_texture_cache_last_grayscale() && image_resolution < 1.0)
      image_resolution = 1.0;
    dgc_utm_to_terra_tile(x1, y1, utmzone, image_resolution, 
			  &terra_res, &min_tile_x, &min_tile_y, &terra_zone);
    dgc_utm_to_terra_tile(x2, y2, utmzone, image_resolution, 
			  &terra_res, &max_tile_x, &max_tile_y, &terra_zone);
    max_tiles = 14;
    break;
  case DGC_IMAGERY_TYPE_LASER:
    dgc_utm_to_laser_tile(x1, y1, image_resolution, &min_tile_x, &min_tile_y);
    dgc_utm_to_laser_tile(x2, y2, image_resolution, &max_tile_x, &max_tile_y);
    max_tiles = 5;
    break;
  case DGC_IMAGERY_TYPE_GSAT:
    dgc_utm_to_gmaps_tile(x1, y1, utmzone, image_resolution, 
			  &min_tile_x, &max_tile_y, &gmaps_zoom);
    dgc_utm_to_gmaps_tile(x2, y2, utmzone, image_resolution, 
			  &max_tile_x, &min_tile_y, &gmaps_zoom);
    max_tiles = 14;
    break;
  case DGC_IMAGERY_TYPE_DARPA:
    dgc_utm_to_darpa_tile(x1, y1, utmzone, image_resolution, 
			  &min_tile_x, &min_tile_y, &darpa_resolution);
    dgc_utm_to_darpa_tile(x2, y2, utmzone, image_resolution, 
			  &max_tile_x, &max_tile_y, &darpa_resolution);
    max_tiles = 14;
    break;
  }

  clamp_tile_interval(&min_tile_x, &max_tile_x, max_tiles);
  clamp_tile_interval(&min_tile_y, &max_tile_y, max_tiles);

  if(current_imagery_type == DGC_IMAGERY_TYPE_COLOR ||
     current_imagery_type == DGC_IMAGERY_TYPE_TOPO ||
     current_imagery_type == DGC_IMAGERY_TYPE_BW)
    draw_terra_tiles(imagery_root, current_imagery_type, min_tile_x, 
		     min_tile_y, max_tile_x, max_tile_y, terra_zone, 
		     terra_res, x_origin, y_origin, use_ned, ned, 
		     height_factor, fetch_only);
  else if(current_imagery_type == DGC_IMAGERY_TYPE_LASER)
    draw_laser_tiles(imagery_root, min_tile_x, min_tile_y, 
		     max_tile_x, max_tile_y, utmzone, image_resolution,
		     x_origin, y_origin, use_ned, ned, height_factor, 
		     fetch_only);
  else if(current_imagery_type == DGC_IMAGERY_TYPE_GSAT)
    draw_gsat_tiles(imagery_root, min_tile_x, min_tile_y, max_tile_x, 
		    max_tile_y, gmaps_zoom, x_origin, y_origin, use_ned, 
		    ned, height_factor, fetch_only);
  else if(current_imagery_type == DGC_IMAGERY_TYPE_DARPA)
    draw_darpa_tiles(imagery_root, min_tile_x, min_tile_y, max_tile_x, 
		     max_tile_y, darpa_resolution, x_origin, y_origin, use_ned,
		     ned, height_factor, fetch_only);
}

inline void imagery_2D_core(const char* imagery_root, double image_resolution,
			    double window_width, double window_height, 
			    double zoom, double x_origin, double y_origin,
			    double view_x_offset, double view_y_offset, 
			    const char* utmzone, int fetch_only)
{
  double x1, y1, x2, y2;

  /* find window boundaries in UTM */
  x1 = x_origin + view_x_offset - window_width / 2.0 / zoom * 1.5;
  y1 = y_origin + view_y_offset - window_height / 2.0 / zoom * 1.5;
  x2 = x_origin + view_x_offset + window_width / 2.0 / zoom * 1.5;
  y2 = y_origin + view_y_offset + window_height / 2.0 / zoom * 1.5;

  imagery_core(imagery_root, image_resolution, x_origin,
	       y_origin, x1, y1, x2, y2, utmzone, 0, 
	       1.0, fetch_only);
}

inline void imagery_3D_core(const char* imagery_root, double image_resolution,
			    double x_origin, double y_origin, 
			    double view_x_offset, double view_y_offset, 
			    const char* utmzone, double radius, int use_ned,
			    double height_factor, int fetch_only)
{
  double x1, y1, x2, y2;

  /* find window boundaries in UTM */
  x1 = x_origin + view_x_offset - radius;
  y1 = y_origin + view_y_offset - radius;
  x2 = x_origin + view_x_offset + radius;
  y2 = y_origin + view_y_offset + radius;

  imagery_core(imagery_root, image_resolution, x_origin,
	       y_origin, x1, y1, x2, y2, utmzone, use_ned, height_factor, 
	       fetch_only);
}

void dgc_imagery_draw_2D(const char* imagery_root, double window_width,
			 double window_height, double zoom, double x_origin, 
			 double y_origin, double view_x_offset, 
			 double view_y_offset, const char* utmzone, int prefetch)
{
  double image_resolution = 1;
  double min_res = 1, max_res = 256;

  //  if(dgc_texture_cache_get_version() == 1)
  //    prefetch = 0;

  min_res = min_imagery_resolution[current_imagery_type];
  max_res = max_imagery_resolution[current_imagery_type];

  image_resolution = min_res;
  while(image_resolution * 3.0 / 2 < 1 / zoom)
    image_resolution *= 2;
  if(image_resolution > max_res)
    image_resolution = max_res;

  imagery_2D_core(imagery_root, image_resolution, window_width, window_height, 
		  zoom, x_origin, y_origin, view_x_offset, view_y_offset, utmzone, 0);
  if(prefetch) {
    if(image_resolution * 2 <= max_res)
      imagery_2D_core(imagery_root, image_resolution * 2, window_width,
		      window_height, zoom * 2, x_origin, y_origin,
		      view_x_offset, view_y_offset, utmzone, 1);
    if(image_resolution / 2 >= min_res)
      imagery_2D_core(imagery_root, image_resolution / 2, window_width, 
		      window_height, zoom / 2, x_origin, y_origin,
		      view_x_offset, view_y_offset, utmzone, 1);
  }
}

void dgc_imagery_draw_3D(const char* imagery_root, double camera_distance,
			 double camera_x_offset, double camera_y_offset,
			 double x_origin, double y_origin, const char* utmzone,
			 int flat, double height_factor, int prefetch)
{
  double r, resc = 1, min_res = 1, max_res = 1, res;

  if(dgc_texture_cache_get_version())
    prefetch = 0;

  /* pick a viewing radius and resolution */
  r = camera_distance / 4000 * 3000;
  if(r < 100)
    r = 100;

  resc = camera_distance / 160.0 * 0.25;
  min_res = min_imagery_resolution[current_imagery_type];
  max_res = max_imagery_resolution[current_imagery_type];

  res = min_res;
  while(res * 2 < resc)
    res *= 2;
  if(res > max_res)
    res = max_res;

  glPushMatrix();
  if(flat) {
    glTranslatef(0, 0, -0.2);
    imagery_3D_core(imagery_root, res, x_origin, y_origin, camera_x_offset, 
		    camera_y_offset, utmzone, r, 0, 1.0, 0);
  }
  else {
    glTranslatef(0, 0, 
                 -lookup_height(ned, x_origin + camera_x_offset, 
                                y_origin + camera_y_offset) * 
		 height_factor - 1);
    imagery_3D_core(imagery_root, res, x_origin, y_origin, camera_x_offset, 
		    camera_y_offset, utmzone, r, 1, height_factor, 0);
  }
  glPopMatrix();
  
  /* prefetch lower and higher resolution images, if requested */
  if(prefetch) {
    if(res / 2.0 >= min_res)
      imagery_3D_core(imagery_root, res / 2, x_origin, y_origin, 
		      camera_x_offset, camera_y_offset, utmzone, r / 2, 0,
		      1.0, 1);
    if(res * 2 <= max_res)
      imagery_3D_core(imagery_root, res * 2, x_origin, y_origin, 
		      camera_x_offset, camera_y_offset, utmzone, r * 2, 0,
		      1.0, 1);
  }
}

int dgc_imagery_update(void)
{
  return dgc_texture_cache_sync();
}

void dgc_imagery_set_min_resolution(int image_type, double min_resolution)
{
  min_imagery_resolution[image_type] = min_resolution;
}

void dgc_imagery_set_max_resolution(int image_type, double max_resolution)
{
  max_imagery_resolution[image_type] = max_resolution;
}

} // namespace vlr
