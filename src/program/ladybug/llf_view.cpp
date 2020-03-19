#include <roadrunner.h>
#include <gui3D.h>
#include <videoout.h>
#include <ipc_std_interface.h>
#include <ladybug_video.h>
#include <ladybug_interface.h>
#include <param_interface.h>
#include <blf.h>
#include <blf_id.h>

#include "qt.h"

using namespace dgc;
using namespace vlr;

#define MAX_NUM_SCANS            10000
#define OPENGL_FOV                 60
#define MAX_STRING_LENGTH          256

extern QApplication   *win  ;
extern QTGui          *qgui;

char                   *ladybug_config_file;
LadybugVideo           *vs;
LadybugPacket          *pkt = NULL;;
int                     pkt_update = FALSE;
int                     pkt_index = 0;

blf_t                   *blf = NULL;
blf_index_t             *blf_index = NULL;

dgc_videoout_p          vo = NULL;
int                     record_video = false;
int                     correct_falloff = true;

void   qgui3D_initialize(int argc, char **argv, int window_x, int window_y,
			 int window_width, int window_height, double fps);

void   qgui3D_mainloop( void );

void   qgui3D_set_num_frames( int num );


char *
createTimeString( double time )
{
  static char str[256];
  long tv_sec = (long) time;
  struct tm *actual_date;
  actual_date = localtime( &tv_sec );
  snprintf( str, 256, "%02d:%02d:%02d",
	    actual_date->tm_hour,
	    actual_date->tm_min,
	    actual_date->tm_sec );
  return(str);
}

char *
createDateString( double time )
{
  static char str[256];
  long tv_sec = (long) time;
  struct tm *actual_date;
  actual_date = localtime( &tv_sec );
  snprintf( str, 256, "%04d-%02d-%02d",
	    1900+actual_date->tm_year,
	    actual_date->tm_mon+1,
	    actual_date->tm_mday );
  return(str);
}

void
llf_read_pkt( int index )
{
  int                           ret;
  unsigned short                pkt_id;

  if(index>=0 && index < blf_index->num_blocks) {
    blf->seek(blf_index->block[index].offset, SEEK_SET);
    ret = blf->read_data(&pkt_id, &(pkt->timestamp),
			 &(pkt->data), &(pkt->len), &(pkt->max_len));
    if( (1 || pkt_id == BLF_LADYBUG2_ID || 
	 pkt_id == BLF_LADYBUG3_ID ||
	 pkt_id == BLF_UNKNOWN_ID ) && pkt->len > 0) {
      pkt->version = LADYBUG_VERSION_2;
      pkt_update = TRUE;
      pkt_index = index;
    }
  }
}


void 
shutdown_handler(int x)
{
  if(x == SIGINT) {
    fprintf(stderr, "Shutting down...\n");
    exit(0);
  }
}

void 
keyboard(unsigned char key, __attribute__ ((unused)) int x,
	 __attribute__ ((unused)) int y )
{
  switch(key) {
  case 'f': case 'F':
    correct_falloff = 1 - correct_falloff;
    fprintf(stderr, "correct_falloff=%d\n", correct_falloff);
    break;
  case 'r': case 'R':
    record_video = 1 - record_video;
#ifdef HAVE_VIDEOOUT
    if(record_video) {
      dgc_info("record video with size %d x %d\n",
	       gui3D.window_width, gui3D.window_height);
      vo  = dgc_videoout_init_mt(dgc_unique_filename("ladybug.avi"),
				 40 * gui3D.window_width *
				 gui3D.window_height, 
				 gui3D.window_width, gui3D.window_height,
				 15, CODEC_ID_MPEG4, 0, PIX_FMT_YUV420P);
    } else {
      dgc_videoout_release_mt(&vo);
    }
#endif
    break;  
  case 27: case 'q': case 'Q':
    kill(getpid(), SIGTERM);
#ifdef HAVE_VIDEOOUT
    if(record_video)
      dgc_videoout_release_mt(&vo);
#endif
    break;
  default:
    break;
  }
  gui3D_forceRedraw();
}

void 
display(void) 
{
  char              str[MAX_STRING_LENGTH];
  static int        firsttime = 1;

  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (pkt_update) { 
    if (firsttime) {
      vs->initializeCg();
      firsttime = 0;
    }

    vs->uncompressData(pkt->data);
    for(int i = 0; i < 6; i++) {
      if(vs->cameraInUse(i)) {
	vs->getImageDistorted(i);
	//[cp]
  dgc_image_write( &(vs->distortedImage[2]), "out.png" );
	//
	vs->demosaicBilinear(i);
      }
    } 
    pkt_update = FALSE;
  }
  
  gui3D_switch_to_3D_mode();
  for(int i = 0; i < 6; i++) {
    if(!vs->cameraInUse(i))
      continue;
    vs->drawGL(i,correct_falloff);
  }

  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  snprintf( str, MAX_STRING_LENGTH, "%s %s [frame: %06d]", 
	    createDateString( pkt->timestamp ),
	    createTimeString( pkt->timestamp ),
	    pkt_index );
  renderBitmapString(gui3D.window_width-310, 15, GLUT_BITMAP_HELVETICA_18, str);

#ifdef HAVE_VIDEOOUT
  /* save to video */
  if(record_video) {  
    dgc_videoout_add_opengl_frame_mt(vo);
  }
#endif    


  gui3D_forceRedraw();
}


void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"ladybug", "config_file", DGC_PARAM_FILENAME, &ladybug_config_file, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int 
main(int argc, char **argv)
{
  int h, min, sec, secs;

  if (argc!=2) {
    fprintf( stderr, "usage: %s <LLF-FILE>\n", argv[0] );
    exit(0);
  }

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  blf = new blf_t;
  if(blf->open(argv[argc - 1], "r") != BLF_OK)
    dgc_fatal_error("Could not open BLF file %s for reading.", argv[argc - 1]);

  blf_index = blf_index_load(argv[argc - 1]);
  if(blf_index == NULL) 
    dgc_fatal_error("Ladybug file must have index file.");

  secs = blf_index->block[blf_index->num_blocks-1].timestamp-
      blf_index->block[0].timestamp;
  h = secs / 3600;
  min = (secs - h * 3600) / 60;
  sec = secs - h * 3600 - min * 60;
  dgc_info( "log file duration: %02dh:%02dmin:%02dsec\n", 
	    h, min, sec );

  pkt = new LadybugPacket;

  vs = new LadybugVideo(ladybug_config_file);

  signal(SIGINT, &shutdown_handler);
  
  fprintf( stderr, "# INFO: initialize QT GUI interface\n" );
  qgui3D_initialize(argc, argv, 10, 10, 800, 600,25);
  gui3D_setCameraParams(0.01, 0.3, 0.001, 0.009, OPENGL_FOV, 0.2, 4000); 
  gui3D_setInitialCameraPos(180, 0, 0.03, 0, 0, 0 );

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);

  qgui3D_set_num_frames(blf_index->num_blocks);
  llf_read_pkt( 0 );

  qgui3D_mainloop();

  return 0;
}
