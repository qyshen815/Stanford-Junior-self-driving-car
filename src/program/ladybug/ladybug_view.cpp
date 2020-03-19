#include <roadrunner.h>
#include <gui3D.h>
#include <imagery.h>
#include <ipc_std_interface.h>
#include <passat_constants.h>
#include <videoout.h>
#include <ladybug_video.h>
#include <param_interface.h>
#include <ladybug_shm_interface.h>
#include <velodyne_shm_interface.h>
#include <perception_interface.h>

#define MAX_NUM_SCANS            10000

using namespace vlr;
using namespace dgc;
using namespace vlr;

LadybugVideo        *vs;
LadybugPacket       *pkt = NULL;
int                  pkt_update = FALSE;
pthread_mutex_t      ladybug_mutex = PTHREAD_MUTEX_INITIALIZER;
dgc_transform_t      ladybug_offset;
char                *ladybug_config_file;
VelodyneInterface   *velo_interface = NULL;
LadybugInterface    *lbug_interface = NULL;

pthread_mutex_t      velodyne_mutex = PTHREAD_MUTEX_INITIALIZER;
dgc_velodyne_scan_p  scans = NULL;
int                  num_scans = 0;

pthread_mutex_t      obstacles_mutex = PTHREAD_MUTEX_INITIALIZER;
PerceptionObstacles      obstacles;

typedef struct {
  double    r;
  double    g;
  double    b;
} dgc_rgb_t, *dgc_rgb_p;

#define        OPENGL_FOV                    60

dgc_videoout_p vo = NULL;
int record_video = false;

bool show_ladybug   = true;
bool show_imagery   = true;
bool show_velodyne  = false;
bool use_velodyne   = true;
bool show_obstacles = false;

int correct_falloff = true;

pthread_t thread1, thread2, thread3;

char *imagery_root;

double robot_x = 0, robot_y = 0, robot_z = 0;
double robot_yaw = 0, robot_pitch = 0, robot_roll = 0;
char utmzone[10];

void 
shutdown_handler(int x)
{
  if(x == SIGINT) {
    fprintf(stderr, "Shutting down...");
    pthread_cancel(thread1);
    pthread_cancel(thread2);
    pthread_cancel(thread3);
    fprintf(stderr, "done\n");
    exit(0);
  }
}

void 
keyboard(unsigned char key, __attribute__ ((unused)) int x,
	 __attribute__ ((unused)) int y )
{
  switch(key) {
  case '1':
    show_ladybug = !show_ladybug;
    break;
  case '2':
    show_velodyne = !show_velodyne;
    break;
  case '3':
    show_obstacles = !show_obstacles;
    break;
  case 'f': case 'F':
    correct_falloff = 1 - correct_falloff;
    fprintf(stderr, "correct_falloff=%d\n", correct_falloff);
    break;
  case 'l': case 'L':
    show_ladybug =  1 - show_ladybug;
    break; 
  case 'o': case 'O': 
    show_obstacles = !show_obstacles;
    break;
  case 'v': case 'V': 
    show_velodyne = !show_velodyne;
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
  case 'i':
    show_imagery = 1 - show_imagery;
    break;
  case 'I':
    dgc_imagery_cycle_imagery_type();
    if (dgc_imagery_current_imagery_type()==DGC_IMAGERY_TYPE_NONE) {
      dgc_imagery_cycle_imagery_type();
    }
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
draw_cube( float x, float y, float zm, float zp, float w, 
	   dgc_rgb_t rgb1, dgc_rgb_t rgb2, float occ )
{
  float w2 = w/2.0;
  float xp = x+w2;
  float xm = x-w2;
  float yp = y+w2;
  float ym = y-w2;

  // top
  glBegin(GL_QUADS);
  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xm, yp, zp );
  glVertex3f( xp, yp, zp );
  glVertex3f( xp, ym, zp );
  glVertex3f( xm, ym, zp );
  glEnd();
  

  // Draw the side faces
  glBegin(GL_TRIANGLE_STRIP);

  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xm, yp, zp );

  glColor4f( rgb1.r, rgb1.g, rgb1.b, occ );
  glVertex3f( xm, yp, zm );

  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xp, yp, zp );

  glColor4f( rgb1.r, rgb1.g, rgb1.b, occ );
  glVertex3f( xp, yp, zm );

  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xp, ym, zp );

  glColor4f( rgb1.r, rgb1.g, rgb1.b, occ );
  glVertex3f( xp, ym, zm );
  
  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xm, ym, zp );

  glColor4f( rgb1.r, rgb1.g, rgb1.b, occ );
  glVertex3f( xm, ym, zm );

  glColor4f( rgb2.r, rgb2.g, rgb2.b, occ );
  glVertex3f( xm, yp, zp );

  glColor4f( rgb1.r, rgb1.g, rgb1.b, occ );
  glVertex3f( xm, yp, zm );

  glEnd();

}

void
draw_obstacles( dgc_pose_t pose )
{
  int               i;
  dgc_rgb_t         rgb1={0.0,0.0,0.0}, rgb2={0.0,0.0,0.0};
  double            p_x, p_y, p_z;
  dgc_transform_t   t, tinv;

  if (show_obstacles && obstacles.num_points>0) {

    dgc_transform_copy(t, ladybug_offset);
    dgc_transform_rotate_x(t, pose.roll);
    dgc_transform_rotate_y(t, pose.pitch);
    dgc_transform_rotate_z(t, pose.yaw);
    dgc_transform_inverse(t, tinv);
      
    glPushMatrix();
    glColor3f( 1.0, 0.0, 0.0 );
    
    glEnable(GL_BLEND);

    glTranslatef( 0.0, 0.0, -DGC_PASSAT_HEIGHT );

    pthread_mutex_lock( &obstacles_mutex );

    rgb1.r = 0.0;
    rgb1.g = 0.0;
    rgb1.b = 0.0;
    for(i = 0; i < obstacles.num_points; i++) {
      if (obstacles.point[i].type!=0) {
	rgb2.r = 1.0;
	rgb2.g = 1.0;
	rgb2.b = 0.0;
      } else {
	rgb2.r = 1.0;
	rgb2.g = 0.0;
	rgb2.b = 0.0;
      }
      p_x = obstacles.point[i].x-pose.x;
      p_y = obstacles.point[i].y-pose.y;
      p_z = obstacles.point[i].z_min-pose.z;
      dgc_transform_point( &p_x, &p_y, &p_z, tinv );
      draw_cube( p_x, p_y, 
		 p_z+DGC_PASSAT_HEIGHT, 
		 p_z+DGC_PASSAT_HEIGHT+
		 (obstacles.point[i].z_max-obstacles.point[i].z_min),
		 0.245, rgb1, rgb2, 0.5 );
    }

    pthread_mutex_unlock( &obstacles_mutex );

    glPopMatrix();
    
    glDisable(GL_BLEND);
   
  }
}
     

void 
timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void 
display(void) 
{
  double            current_time = dgc_get_time();
  static double     last_time = 0;
  static int        firsttime = 1;
  dgc_transform_t   t, tinv;
  int               i, j;
  double            p_x, p_y, p_z;
  double            u;
  dgc_pose_t        ladybug_pose;

  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (show_imagery) {
    if (current_time-last_time>0.05) {
      dgc_imagery_update();
      last_time = current_time;
    }
    glPushMatrix();
    {
      glEnable(GL_BLEND);
      glTranslatef(0, 0, -DGC_PASSAT_HEIGHT-0.05);
      glBlendFunc(GL_DST_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  
      
      dgc_imagery_draw_3D(imagery_root,
			  gui3D.camera_pose.distance,
			  gui3D.camera_pose.x_offset, 
			  gui3D.camera_pose.y_offset, 
			  robot_x, robot_y,
			  utmzone, 1, 1.0, 1);
      
      glDisable(GL_BLEND);
    }
    glPopMatrix();
  }

  pthread_mutex_lock(&ladybug_mutex);
  ladybug_pose.x     = pkt->smooth_x;
  ladybug_pose.y     = pkt->smooth_y;
  ladybug_pose.z     = pkt->smooth_z;
  ladybug_pose.yaw   = pkt->yaw;
  ladybug_pose.pitch = pkt->pitch;
  ladybug_pose.roll  = pkt->roll;
  pthread_mutex_unlock(&ladybug_mutex);

  if (show_ladybug) {
    pthread_mutex_lock(&ladybug_mutex);
    if (pkt_update) { 
      if (firsttime) {
	vs->initializeCg();
	firsttime = FALSE;
      }
      vs->uncompressData(pkt->data);
      for(int i = 0; i < 6; i++) {
	if(vs->cameraInUse(i)) {
	  vs->getImageDistorted(i);
	  vs->demosaicBilinear(i);
	}
      }
      pkt_update = FALSE;
    }
    
    gui3D_switch_to_3D_mode();
    if (!firsttime) {
      for(int i = 0; i < 6; i++) {
	if(!vs->cameraInUse(i))
	  continue;
	vs->drawGL(i,correct_falloff);
      }
    }
    pthread_mutex_unlock(&ladybug_mutex);
  }


  if (use_velodyne && show_velodyne && num_scans>0) {
    pthread_mutex_lock(&velodyne_mutex);
    glPushMatrix();
    {

      dgc_transform_copy(t, ladybug_offset);
      dgc_transform_rotate_x(t, ladybug_pose.roll);
      dgc_transform_rotate_y(t, ladybug_pose.pitch);
      dgc_transform_rotate_z(t, ladybug_pose.yaw);
      dgc_transform_inverse(t, tinv);
      
      {
	glEnable(GL_BLEND);
	glPointSize(7.0);
	glBegin(GL_POINTS);
	for(i = 0; i < num_scans; i++) {
	  for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
	    if(scans[i].p[j].range < 0.01) 
	      continue;
	    
	    u = 0.5 + (scans[i].p[j].intensity - 40.0)/80.0;
	    if(u < 0) u = 0;
	    if(u > 1) u = 1;
	    glColor4f(u, u, u, 0.5);
	    p_x = scans[i].p[j].x * 0.01;
	    p_y = scans[i].p[j].y * 0.01;
	    p_z = scans[i].p[j].z * 0.01;
	    dgc_transform_point( &p_x, &p_y, &p_z, tinv );
	    glVertex3f(p_x, p_y, p_z);
	  }
	}
	glEnd();
	glDisable(GL_BLEND);
      }
    }
    glPopMatrix();
    pthread_mutex_unlock(&velodyne_mutex);
  }

  if (show_obstacles && obstacles.num_points>0) {
    draw_obstacles( ladybug_pose );
  }
    
#ifdef HAVE_VIDEOOUT
  /* save to video */
  if(record_video) {  
    dgc_videoout_add_opengl_frame_mt(vo);
  }
#endif    

  gui3D_forceRedraw();
}


void *
graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 800, 600, 15.0);
  gui3D_setCameraParams(0.01, 0.3, 0.001, 0.009, OPENGL_FOV, 0.2, 4000); 
  gui3D_setInitialCameraPos(180, 0, 0.03, 0, 0, 0 );

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  
  gui3D_mainloop();
  return NULL;
}

void *
ladybug_thread(__attribute__ ((unused)) void *ptr)
{
  pkt = new LadybugPacket;

  while (1) {
    if (lbug_interface->DataWaiting()) {
      pthread_mutex_lock(&ladybug_mutex);
      lbug_interface->ReadCurrentPacket(pkt);
      pthread_mutex_unlock(&ladybug_mutex);
      if(pkt->len > 0) 
	pkt_update = TRUE;
    } 
    else 
      usleep(100);
  }
  return NULL;
}

void *
velodyne_thread(__attribute__ ((unused)) void *ptr)
{
  while (1) {
    if (velo_interface->ScanDataWaiting()) {
      pthread_mutex_lock(&velodyne_mutex);
      num_scans = 
	velo_interface->ReadCurrentScans(scans, MAX_NUM_SCANS);
      pthread_mutex_unlock(&velodyne_mutex);
    } else {
      usleep(10000);
    }
  }
  return NULL;
}

void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"ladybug", "config_file", DGC_PARAM_FILENAME, &ladybug_config_file, 0, NULL},
    {"transform", "ladybug", DGC_PARAM_TRANSFORM, &ladybug_offset, 1, NULL},
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int 
main(int argc, char **argv)
{
  param_struct_t param;

  /* initialize IPC */
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  lbug_interface = new LadybugShmInterface;
  if(lbug_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to ladybug interface.\n");

  //  init_ground_transforms();
  /* start video source */
  vs = new LadybugVideo(ladybug_config_file);

  velo_interface = new VelodyneShmInterface;
  if(velo_interface->CreateClient() < 0) {
    fprintf(stderr, 
	    "Warning: could not connect to velodyne scan shm server.\n");
    fprintf(stderr, "Warning: disabling velodyne support.\n");
    use_velodyne = false;
  } 
  else {							
    scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS * 
					sizeof(dgc_velodyne_scan_t));
    dgc_test_alloc(scans);
  }

  signal(SIGINT, &shutdown_handler);
  
  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread1, NULL, graphics_thread, &param);

  /* start the velodyne thread */
  if(use_velodyne) 
    pthread_create(&thread2, NULL, velodyne_thread, NULL);

  /* start the ladybug thread */
  pthread_create(&thread3, NULL, ladybug_thread, NULL);

  ipc->Subscribe(PerceptionObstaclesID, &obstacles, DGC_SUBSCRIBE_LATEST,
		 &obstacles_mutex);
  ipc->Dispatch();
  return 0;
}
