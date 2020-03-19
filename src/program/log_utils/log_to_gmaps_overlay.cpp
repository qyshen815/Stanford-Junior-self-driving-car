#include <roadrunner.h>
#include <gl_support.h>
#include <lltransform.h>
#include <logio.h>
#include <image.h>
#include <imagery_proj.h>
#include <applanix_interface.h>
#include <vector>

#define        MIN_GMAPS_ZOOM       13
#define        MAX_GMAPS_ZOOM       20

#define        BACK_R               255
#define        BACK_G               255
#define        BACK_B               255

using namespace dgc;
using namespace vlr;
using std::vector;

typedef struct {
  double lat, lon, yaw, x, y;
} path_pose_t;

vector <path_pose_t> pose;
char overlaydir[200];

void write_js(char *filename)
{
  FILE *fp;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Erorr: could not open file %s for writing.\n", filename);

  fprintf(fp, "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Strict//EN\" \"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd\">\n");
  fprintf(fp, "<html xmlns=\"http://www.w3.org/1999/xhtml\">\n");
  fprintf(fp, "  <head>\n");
  fprintf(fp, "    <meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"/>\n");
  fprintf(fp, "    <title>Stanford DGC Logfile Google Maps Overlay</title>\n");
  fprintf(fp, "    <script src=\"http://maps.google.com/maps?file=api&amp;v=2&amp;key=ABQIAAAAypuZ3Y5VT1XpHJJt64vnCBT9M2TuMkImCT64mEtLBSdmM1p6GRTefs8NVMPETYJDmVZ9rH0mec8tFQ\" type=\"text/javascript\"></script>\n");
  fprintf(fp, "    <script type=\"text/javascript\">\n");
  fprintf(fp, "\n");
  fprintf(fp, "    //<![CDATA[\n");

  fprintf(fp, "function load() {\n");
  fprintf(fp, "  if(GBrowserIsCompatible()) {\n");
  fprintf(fp, "    var myCopyright = new GCopyrightCollection(\"(c) \");\n");
  fprintf(fp, "    myCopyright.addCopyright(new GCopyright('Demo',\n");
  fprintf(fp, "      new GLatLngBounds(new GLatLng(-90,-180), new GLatLng(90,180)),\n");
  fprintf(fp, "      0,'©2008 M. Montemerlo'));\n");
  fprintf(fp, "\n");
  fprintf(fp, "    var map = new GMap2(document.getElementById(\"map_canvas\"));\n");
  fprintf(fp, "    map.addControl(new GLargeMapControl());\n");
  fprintf(fp, "    map.addControl(new GMapTypeControl());\n");
  fprintf(fp, "    map.addControl(new GScaleControl());\n");
  fprintf(fp, "    map.addControl(new GOverviewMapControl());\n");
  fprintf(fp, "\n");
  fprintf(fp, "    map.setCenter(new GLatLng(%f, %f), 17);\n", pose[0].lat, 
	  pose[0].lon);
  fprintf(fp, "    var myOverlay = new GTileLayerOverlay(\n");
  fprintf(fp, "      new GTileLayer(null, null, null, {\n");
  fprintf(fp, "        tileUrlTemplate: './overlay_{Z}_{X}_{Y}.png', \n");
  fprintf(fp, "        isPng:true,\n");
  fprintf(fp, "        opacity:1.0\n");
  fprintf(fp, "      })\n");
  fprintf(fp, "    );\n");
  fprintf(fp, "\n");
  fprintf(fp, "    map.setMapType(G_SATELLITE_MAP);\n");
  fprintf(fp, "    map.addOverlay(myOverlay); \n");
  fprintf(fp, "  }\n");
  fprintf(fp, "}\n");
  fprintf(fp, "\n");
  fprintf(fp, "    //]]>\n");

  fprintf(fp, "    </script>\n");
  fprintf(fp, "  </head>\n");
  fprintf(fp, "  <body onload=\"load()\" onunload=\"GUnload()\">\n");
  fprintf(fp, "    <div id=\"map_canvas\" style=\"width: 800px; height: 600px\"></div>\n");
  fprintf(fp, "  </body>\n");
  fprintf(fp, "</html>\n");
  fclose(fp);
}

void frame_to_image(char *filename)
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
  dgc_image_rgba_write_raw(buffer, 256, 256, filename);
}

void display(int gmaps_x, int gmaps_y, int gmaps_zoom)
{
  double x, y, w;
  int i, last_i;
  double lat1, lon1, lat2, lon2;

  set_display_mode_2D(256, 256);

  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

  /* figure out bounds of current gmaps tiles */
  dgc_gmaps_tile_bounds(gmaps_x, gmaps_y, gmaps_zoom, 
			&lat1, &lon1, &lat2, &lon2);

  w = 3;
  glColor4f(0, 0, 1, 0.75);
    
  glBegin(GL_QUAD_STRIP);
  last_i = 0;
  for(i = 0; i < (int)pose.size(); i++) {
      if(i == 0 || hypot(pose[i].x - pose[last_i].x,
			 pose[i].y - pose[last_i].y) > 1.0) {
	x = (pose[i].lon - lon1) / (lon2 - lon1) * 256.0;
	y = (pose[i].lat - lat1) / (lat2 - lat1) * 256.0;
	glVertex2f(x + w * cos(pose[i].yaw + M_PI / 2.0),
		   y + w * sin(pose[i].yaw + M_PI / 2.0));
	glVertex2f(x - w * cos(pose[i].yaw + M_PI / 2.0),
		   y - w * sin(pose[i].yaw + M_PI / 2.0));
	last_i = i;
      }
  }
  glEnd();

  glutSwapBuffers();
}

inline void render_image(double lat, double lon, int gmaps_zoom)
{
  static int last_gmaps_x = -1, last_gmaps_y = -1, last_gmaps_zoom = -1;
  int gmaps_x, gmaps_y;
  char filename[200];

  dgc_ll_to_gmaps_tile(lat, lon, gmaps_zoom, &gmaps_x, &gmaps_y);
  if(gmaps_x == last_gmaps_x && gmaps_y == last_gmaps_y &&
     gmaps_zoom == last_gmaps_zoom)
    return;

  sprintf(filename, "%s/overlay_%d_%d_%d.png", overlaydir,
	  gmaps_zoom, gmaps_x, gmaps_y);
  if(!dgc_file_exists(filename)) {
    display(gmaps_x, gmaps_y, gmaps_zoom);
    fprintf(stderr, "writing %s\n", filename);
    frame_to_image(filename);
  }						
  
  last_gmaps_x = gmaps_x;
  last_gmaps_y = gmaps_y;
  last_gmaps_zoom = gmaps_zoom;
}

void idle(void)
{
  int i, gmaps_zoom;

  for(gmaps_zoom = MIN_GMAPS_ZOOM; gmaps_zoom <= MAX_GMAPS_ZOOM; gmaps_zoom++) 
    for(i = 0; i < (int)pose.size(); i++) 
      render_image(pose[i].lat, pose[i].lon, gmaps_zoom);

  exit(0);
}

void read_path(char *filename)
{
  ApplanixPose applanix_pose;
  LineBuffer *line_buffer = NULL;
  char *line = NULL, *s, utmzone[10];
  path_pose_t p;
  dgc_FILE *fp;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);

  line_buffer = new LineBuffer;

  do {
    /* read a complete line */
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	s = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	p.lat = applanix_pose.latitude;
	p.lon = applanix_pose.longitude;
	p.yaw = applanix_pose.yaw;
	latLongToUtm(p.lat, p.lon, &p.x, &p.y, utmzone);
	pose.push_back(p);
      }
    }
  } while(line != NULL);
  dgc_fclose(fp);
}

int main(int argc, char **argv)
{
  char filename[300];

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s log-filename overlay-dir\n", argv[0]);

  read_path(argv[1]);
  strcpy(overlaydir, argv[2]);

  if(!dgc_file_exists(overlaydir))
    mkdir(overlaydir, 0755);

  strcpy(filename, overlaydir);
  strcat(filename, "/overlay.html");
  write_js(filename);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(256, 256);
  glutInitWindowPosition(0, 0);
  glutCreateWindow(argv[0]);
  glutIdleFunc(idle);
  glutMainLoop();
  return 0;
}
