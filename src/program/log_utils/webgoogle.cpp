#include <roadrunner.h>
#include <gl_support.h>
#include <lltransform.h>
#include <logio.h>
#include <image.h>
#include <imagery_proj.h>
#include <applanix_interface.h>
#include <vector>
#include <wand/magick_wand.h>

using namespace dgc;
using namespace vlr;

#ifdef __cplusplus
extern "C" {
#endif

int create_google_webdir( char *logname, char *dirname, int force );

#ifdef __cplusplus
}
#endif


#define        MIN_GMAPS_ZOOM       13
#define        MAX_GMAPS_ZOOM       20
#define        TILE_WIDTH           256
#define        TILE_HEIGHT          256
#define        MAX_NAME_LENGTH      200

using std::vector;

typedef struct {
  double lat, lon, yaw, x, y;
} path_pose_t;

vector <path_pose_t> pose;
char overlaydir[MAX_NAME_LENGTH];


void 
write_semaphore(char *filename)
{
  FILE *fp;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Erorr: could not open file %s for writing.\n", filename);
  fclose(fp);
}

void 
write_js(char *filename)
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

void
generate_png(int gmaps_x, int gmaps_y, int gmaps_zoom, char *filename)
{ 
  static int            initialized = 0;
  static unsigned char  buffer[TILE_WIDTH * TILE_HEIGHT * 4];
  int                   i, last_i, first;
  double                lat1, lon1, lat2, lon2, x, y, last_x = 0, last_y = 0;
  Image                *image;
  ImageInfo            *info;
  PixelWand            *color;
  ExceptionInfo         exception;
  DrawContext           wand;

  for (i=0; i<TILE_WIDTH * TILE_HEIGHT * 4; i++) {
    buffer[i] = 0;
  }

  if(!initialized) {
    InitializeMagick("DGC");
    initialized = 1;
  }
  GetExceptionInfo(&exception);
  image = ConstituteImage(TILE_WIDTH, TILE_HEIGHT, "RGBA",
			  CharPixel, buffer, &exception);
  if(exception.severity != UndefinedException)
    CatchException(&exception);
  if(image == (Image *)NULL)
    return;

  info = CloneImageInfo((ImageInfo *)NULL);
  color = NewPixelWand();
  wand  = DrawAllocateWand( (DrawInfo*) NULL, image );

  PixelSetColor( color, "red" );
  DrawSetStrokeColor( wand, color );
  PixelSetColor( color, "none" );
  DrawSetFillColor( wand, color );
  DrawSetStrokeWidth( wand, 3 );
  
  dgc_gmaps_tile_bounds(gmaps_x, gmaps_y, gmaps_zoom, 
			&lat1, &lon1, &lat2, &lon2);
  
  last_i = 0; first = 1;
  for(i = 0; i < (int)pose.size(); i++) {
    x = (pose[i].lon - lon1) / (lon2 - lon1) * (double)TILE_WIDTH;
    y = (pose[i].lat - lat1) / (lat2 - lat1) * (double)TILE_HEIGHT;
    if(first) {
      last_x = x;
      last_y = y;
      last_i = i;
      first = 0;
    } else if (hypot(pose[i].x - pose[last_i].x,
		     pose[i].y - pose[last_i].y) > 1.0) {
      DrawLine ( wand, x, TILE_HEIGHT-y, last_x, TILE_HEIGHT-last_y ); 
      last_x = x;
      last_y = y;
      last_i = i;
    }
  }

  DrawRender( wand );

  strcpy(image->filename, filename);
  WriteImage(info, image);

  DestroyDrawingWand( wand );
  DestroyPixelWand( color );
  DestroyImage(image);
  DestroyImageInfo(info);
  DestroyExceptionInfo(&exception);      
}

inline void 
render_image(double lat, double lon, int gmaps_zoom)
{
  static int last_gmaps_x = -1, last_gmaps_y = -1, last_gmaps_zoom = -1;
  int gmaps_x, gmaps_y;
  char filename[MAX_NAME_LENGTH];

  dgc_ll_to_gmaps_tile(lat, lon, gmaps_zoom, &gmaps_x, &gmaps_y);
  if(gmaps_x == last_gmaps_x && gmaps_y == last_gmaps_y &&
     gmaps_zoom == last_gmaps_zoom)
    return;

  snprintf(filename, MAX_NAME_LENGTH, 
	   "%s/overlay_%d_%d_%d.png", overlaydir,
	   gmaps_zoom, gmaps_x, gmaps_y);
  if(!dgc_file_exists(filename)) {
    generate_png(gmaps_x, gmaps_y, gmaps_zoom,filename);
  }						
  
  last_gmaps_x = gmaps_x;
  last_gmaps_y = gmaps_y;
  last_gmaps_zoom = gmaps_zoom;
}


void 
read_path(char *filename)
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

int
create_google_webdir( char *logname, char *dirname, int force )
{
  int     i, gmaps_zoom;
  char    filename[MAX_NAME_LENGTH];
  char    nopose_filename[MAX_NAME_LENGTH];

  strncpy(overlaydir, dirname, MAX_NAME_LENGTH );

  snprintf(filename, MAX_NAME_LENGTH, "%s/overlay.html", overlaydir);
  snprintf(nopose_filename, MAX_NAME_LENGTH, "%s/nopose.txt", overlaydir);

  if(!force && dgc_file_exists(overlaydir) && 
     ( dgc_file_exists(filename) || 
       dgc_file_exists(nopose_filename) ) ) {
    return 1;
  }
  mkdir(overlaydir, 0755);

  read_path(logname);
  if (pose.size()==0) {
    write_semaphore( nopose_filename );
    return(-1);
  }

  write_js(filename);

  for(gmaps_zoom = MIN_GMAPS_ZOOM; gmaps_zoom <= MAX_GMAPS_ZOOM; gmaps_zoom++) 
    for(i = 0; i < (int)pose.size(); i++) 
      render_image(pose[i].lat, pose[i].lon, gmaps_zoom);

  pose.clear();

  return 0;
}
