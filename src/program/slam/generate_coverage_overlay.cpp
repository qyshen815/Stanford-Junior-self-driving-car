#include <roadrunner.h>
#include <gl_support.h>
#include <rndf.h>
#include <lltransform.h>
#include <logio.h>
#include <image.h>
//#include "rndfglll.h"
#include "slam_config.h"
#include <imagery_proj.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <velo_support.h>

#define        MIN_GMAPS_ZOOM       13
#define        MAX_GMAPS_ZOOM       20

using namespace dgc;

dgc_velodyne_index *velodyne_index = NULL;
dgc_velodyne_index *velodyne_fixed_index = NULL;
slam_config_p slam_config = NULL;

/*
rndf_file *rndf = NULL;
int rndf_valid = 0;
char *rndf_filename;
*/

void image_write_raw(unsigned char *image_data, int width, 
		     int height, char *filename)
{
  static int initialized = 0;
  Image *im, *im2;
  ImageInfo *info;
  ExceptionInfo exception;

  if(!initialized) {
    InitializeMagick("DGC");
    initialized = 1;
  }
  GetExceptionInfo(&exception);
  im = ConstituteImage(width, height, "RGBA", CharPixel, 
                       image_data, &exception);
  if(exception.severity != UndefinedException)
    CatchException(&exception);
  if(im == (Image *)NULL)
    return;
  im2 = FlipImage(im, &exception);
  strcpy(im2->filename, filename);
  info = CloneImageInfo((ImageInfo *)NULL);
  WriteImage(info, im2);

  DestroyImage(im);
  DestroyImage(im2);
  DestroyImageInfo(info);
  DestroyExceptionInfo(&exception);      
}

#define BACK_R 255
#define BACK_G 255
#define BACK_B 255

void frame_to_png(char *filename)
{
  static unsigned char buffer[256 * 256 * 4];
  int i, mark;

  glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, buffer);

  mark = 0;
  for(i = 0; i < 256 * 256; i++) {
    if(buffer[mark] == BACK_R && buffer[mark + 1] == BACK_G &&
       buffer[mark + 2] == BACK_B)
      buffer[mark + 3] = 0;
    else
      buffer[mark + 3] = 255;
    mark += 4;
  }
  image_write_raw(buffer, 256, 256, filename);
}

void display_paths(int gmaps_x, int gmaps_y, int gmaps_zoom, bool after)
{
  double x, y, theta, w;
  int i, j, last_j;
  double lat1, lon1, lat2, lon2;//, utm_x1, utm_y1, utm_x2, utm_y2;
  //  char utmzone[10];
  //  double x_scale, y_scale;
  dgc_velodyne_index *index;

  /* figure out bounds of current gmaps tiles */
  dgc_gmaps_tile_bounds(gmaps_x, gmaps_y, gmaps_zoom, 
			&lat1, &lon1, &lat2, &lon2);

  /*  if(rndf_valid) {
    glPushMatrix();
    latLongToUtm(lat1, lon1, &utm_x1, &utm_y1, utmzone);
    latLongToUtm(lat2, lon2, &utm_x2, &utm_y2, utmzone);

    x_scale = 256.0 / (lon2 - lon1);
    y_scale = 256.0 / (lat2 - lat1);
    glScalef(x_scale, y_scale, 1);
    draw_rndf_ll(rndf, 1, 1, 0, lon1, lat1, 1.0);
    glPopMatrix();
    }*/

  w = 3;
  if(!after)
    glColor4f(1, 0, 0, 0.75);
  else
    glColor4f(0, 0, 1, 0.75);
    
  for(i = 0; i < slam_config->num_files; i++) {
    if(!after)
      index = velodyne_index + i;
    else
      index = velodyne_fixed_index + i;

    last_j = 0;
    glBegin(GL_QUAD_STRIP);
    for(j = 0; j < index->num_spins; j++) 
      if(hypot(index->spin[j].pose[0].smooth_x -
	       index->spin[last_j].pose[0].smooth_x,
	       index->spin[j].pose[0].smooth_y -
	       index->spin[last_j].pose[0].smooth_y) > 1.0) {
	theta = atan2(index->spin[j].pose[0].smooth_y -
		      index->spin[last_j].pose[0].smooth_y,
		      index->spin[j].pose[0].smooth_x -
		      index->spin[last_j].pose[9].smooth_x);
	if(last_j == 0) {
	  x = (index->spin[last_j].pose[0].longitude - lon1) / 
	    (lon2 - lon1) * 256.0;
	  y = (index->spin[last_j].pose[0].latitude - lat1) / 
	    (lat2 - lat1) * 256.0;
	  glVertex2f(x + w * cos(theta + M_PI / 2.0),
		     y + w * sin(theta + M_PI / 2.0));
	  glVertex2f(x - w * cos(theta + M_PI / 2.0),
		     y - w * sin(theta + M_PI / 2.0));
	}
	x = (index->spin[j].pose[0].longitude - lon1) / 
	  (lon2 - lon1) * 256.0;
	y = (index->spin[j].pose[0].latitude - lat1) / 
	  (lat2 - lat1) * 256.0;
	glVertex2f(x + w * cos(theta + M_PI / 2.0),
		   y + w * sin(theta + M_PI / 2.0));
	glVertex2f(x - w * cos(theta + M_PI / 2.0),
		   y - w * sin(theta + M_PI / 2.0));
	last_j = j;
      }
    glEnd();
  }
}

void display(int gmaps_x, int gmaps_y, int gmaps_zoom, bool after)
{
  set_display_mode_2D(256, 256);

  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
  //  glEnable(GL_LINE_SMOOTH);

  display_paths(gmaps_x, gmaps_y, gmaps_zoom, after);
  glutSwapBuffers();
}

inline void render_image(double lat, double lon, int gmaps_zoom, bool after)
{
  static int last_gmaps_x = -1, last_gmaps_y = -1, last_gmaps_zoom = -1;
  int gmaps_x, gmaps_y;
  char filename[200];

  dgc_ll_to_gmaps_tile(lat, lon, gmaps_zoom, &gmaps_x, &gmaps_y);
  if(gmaps_x == last_gmaps_x && gmaps_y == last_gmaps_y &&
     gmaps_zoom == last_gmaps_zoom)
    return;

  if(!after)
    sprintf(filename, "before/overlay_%d_%d_%d.png", 
	    gmaps_zoom, gmaps_x, gmaps_y);
  else
    sprintf(filename, "after/overlay_%d_%d_%d.png", 
	    gmaps_zoom, gmaps_x, gmaps_y);
  if(!dgc_file_exists(filename)) {
    display(gmaps_x, gmaps_y, gmaps_zoom, after);
    fprintf(stderr, "writing %s\n", filename);
    frame_to_png(filename);
  }						
  
  last_gmaps_x = gmaps_x;
  last_gmaps_y = gmaps_y;
  last_gmaps_zoom = gmaps_zoom;
}

void generate_images(bool after) 
{
  dgc_velodyne_index *index;
  int i, j, gmaps_zoom;
  //  int k;
  //  rndf_waypoint *w;

  for(gmaps_zoom = MIN_GMAPS_ZOOM; gmaps_zoom <= MAX_GMAPS_ZOOM; gmaps_zoom++) {
    /* draw images along the paths */
    for(i = 0; i < slam_config->num_files; i++) {
      if(!after)
	index = velodyne_index + i;
      else
	index = velodyne_fixed_index + i;
      for(j = 0; j < index->num_spins; j++) 
	render_image(index->spin[j].pose[0].latitude,
		     index->spin[j].pose[0].longitude, gmaps_zoom, after);
    }

    /* draw images along the RNDF */
    /*    if(rndf_valid) 
      for(i = 0; i < rndf->num_segments(); i++)
	for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
	  for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
	    w = rndf->segment(i)->lane(j)->waypoint(k);
	    render_image(w->lat(), w->lon(), gmaps_zoom, after);
	    }*/
  }
}

void idle(void)
{
  generate_images(false);
  generate_images(true);
  exit(0);
}

/*
void read_parameters(int argc, char **argv)
{
  dgc_param_t params[] = {
    {"rndf", "rndf_file", DGC_PARAM_FILE, &rndf_filename, 0, NULL},
  };
  dgc_param_install_params(argc, argv, params, sizeof(params) / 
                           sizeof(params[0]));
}
*/

int main(int argc, char **argv)
{
  char *vlf_filename, *index_filename;
  int i;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s config-file\n", argv[0]);

  /* read configuration file */
  slam_config = read_slam_config(argv[1]);
  if(slam_config == NULL)
    dgc_die("Error: could not read config file %s\n", argv[1]);

  velodyne_index = new dgc_velodyne_index[slam_config->num_files];
  velodyne_fixed_index = new dgc_velodyne_index[slam_config->num_files];

  /* open the velodyne indexes and files */
  for(i = 0; i < slam_config->num_files; i++) {
    vlf_filename = find_vlf_filename(slam_config, i);
    if(vlf_filename == NULL)
      dgc_die("Error: could not find velodyne file %s\n", 
              slam_config->file[i].vlf_filename);
    
    index_filename = find_vlf_index_filename(slam_config, i);
    if(index_filename == NULL)
      dgc_die("Error: could not find velodyne index file for %s\n", 
              slam_config->file[i].vlf_filename);
    fprintf(stderr, "Loading %s... ", index_filename);
    if(velodyne_index[i].load(index_filename) < 0)
      dgc_die("Error: not found\n");
    else
      fprintf(stderr, "done.\n");

    index_filename = find_vlf_fixed_index_filename(slam_config, i);
    if(index_filename == NULL)
      dgc_die("Error: could not find velodyne index file for %s\n", 
              slam_config->file[i].vlf_filename);
    fprintf(stderr, "Loading %s... ", index_filename);
    if(velodyne_fixed_index[i].load(index_filename) < 0)
      fprintf(stderr, "not found.\n");
    else
      fprintf(stderr, "done.\n");
  }
    
  /* connect to IPC server, get parameters, and disconnect */
  /*  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);
  read_parameters(argc, argv);
  dgc_ipc_disconnect();*/

  /* load the RNDF file, if available */
  /*  rndf = new rndf_file;
  if(rndf->load(rndf_filename) == 0)
  rndf_valid = 1;*/

  if(!dgc_file_exists("before"))
    mkdir("before", 0755);
  if(!dgc_file_exists("after"))
    mkdir("after", 0755);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(256, 256);
  glutInitWindowPosition(0, 0);
  glutCreateWindow(argv[0]);
  glutIdleFunc(idle);
  glutMainLoop();
  return 0;
}
