#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <ladybug_shm_interface.h>
#include <applanix_interface.h>

#include "ladybug_dc1394.h"

using namespace dgc;

#define MAX_NAME_LENGTH         256

// applanix
ApplanixPose                     pose;
pthread_mutex_t                  applanix_mutex = PTHREAD_MUTEX_INITIALIZER;

// ladybug
LadybugInterface                 *lbug_interface = NULL;
LadybugPacket                    *lbug_pkt = NULL;

unsigned char jpegHeader[] = {
  0xFF,0xD8,0xFF,0xE0,0x00,0x10,0x4A,0x46, /* 000 */
  0x49,0x46,0x00,0x01,0x01,0x00,0x00,0x01, /* 008 */
  0x00,0x01,0x00,0x00,0xff,0xdb,0x00,0x43, /* 010 */
};

  char *
numf2( float v, char *unit )
{
  static char nstr[20];
  int  val = (int)fabs(v * 100);

  snprintf( nstr, 20, "%s%02.2f %s", (val < 1000) ? "0" : "", 
      val / 100.0, unit );
  return(nstr);
}

  int
check_jpeg_header( unsigned char *buf )
{
  int i;

  for (i=0; i<10; i++) 
    if (buf[i] != jpegHeader[i]) 
      return 0;
  return 1;
}

  unsigned int
ladybug_check_frame( unsigned char *ptr )
{
  int           ok  = 0;
  int           cam, k;
  unsigned int  jpgadr, jpgsize, adr, len = 0;

  for (cam = 0; cam < 6; cam++) {
    for (k = 0; k < 4; k++) {
      adr = 0x340 + (5 - cam) * 32 + (3 - k) * 8;
      jpgadr =
        (((unsigned int)*(ptr + adr)) << 24)+
        (((unsigned int)*(ptr + adr + 1)) << 16)+
        (((unsigned int)*(ptr + adr + 2)) << 8)+
        (((unsigned int)*(ptr + adr + 3)));
      adr += 4;
      jpgsize =
        (((unsigned int)*(ptr + adr)) << 24)+
        (((unsigned int)*(ptr + adr + 1)) << 16)+
        (((unsigned int)*(ptr + adr + 2)) << 8)+
        (((unsigned int)*(ptr + adr + 3)));

      if (jpgadr + jpgsize > len) 
        len = jpgadr + jpgsize;
      if ((jpgsize != 0 && check_jpeg_header( jpgadr + ptr ))) 
        ok++;
      else
        if(jpgsize == 0)
          printf("JPEG SIZE 0! - CAM %d frame %d\n", cam, k);
    }
  }
  if (ok == 24) 
    return len;
  return len;
}

  void 
shutdown( __attribute__ ((unused)) int signal ) 
{
  fprintf(stderr,  "shutting down...");
  ladybug_close_camera();
  fprintf(stderr,"done\n");
  exit(0);
}

  void *
dc1394_thread( void *ptr __attribute__ ((unused)) ) 
{
  time_t            c_time;
  struct tm        *local_time;
  char              date_str[MAX_NAME_LENGTH];
  int               num_images;
  int               num_errors;
  off64_t           written;
  double            fps, o_mb, current_s, last_s, delta_s;

  num_images = 0;
  num_errors = 0;
  written    = 0;
  last_s     = dgc_get_time();

  dgc_info( "start capture loop ..." );
  while (1) {
    if (ladybug_retrieve_frame( lbug_pkt )) {
      pthread_mutex_lock(&applanix_mutex);
      lbug_pkt->smooth_x  = pose.smooth_x;
      lbug_pkt->smooth_y  = pose.smooth_y;
      lbug_pkt->smooth_z  = pose.smooth_z;
      lbug_pkt->yaw       = pose.yaw;
      lbug_pkt->pitch     = pose.pitch;
      lbug_pkt->roll      = pose.roll;
      pthread_mutex_unlock(&applanix_mutex);
      lbug_interface->WritePacket(lbug_pkt);
      written += lbug_pkt->len;
      num_images++;
    } else {
      num_errors++;
    }

    current_s = dgc_get_time();
    delta_s   = current_s - last_s;

    if (delta_s > 0.2) {
      fps    = dgc_avg_times(delta_s, num_images, 0);
      o_mb   = written / (delta_s * 1024.0 * 1024.0);
      c_time = (time_t) current_s;
      local_time = localtime(&c_time);
      snprintf( date_str, MAX_NAME_LENGTH, 
          "%02d-%02d-%02d:%02d-%02d-%02d", 
          local_time->tm_mon + 1, 
          local_time->tm_mday, 
          local_time->tm_year-100, 
          local_time->tm_hour, 
          local_time->tm_min, 
          local_time->tm_sec );

      fprintf(stderr, "\r# LADYBUG: [");
      dgc_fprintf_green( stderr, "%s", date_str );
      fprintf( stderr, "][" );
      dgc_fprintf_red( stderr, "FREQ: %s", numf2( fps, "fps" ) );
      fprintf( stderr, "][" );
      dgc_fprintf_blue( stderr, "SHM: %s", numf2( o_mb, "MB/s" ) );
      fprintf( stderr, "][" );
      dgc_fprintf_red( stderr, "#ERROR: %d", num_errors );
      fprintf( stderr, "]   " );
      num_images = 0;
      written    = 0;
      last_s     = current_s;
    }
  }
  return NULL;
}

void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"ladybug", "auto", DGC_PARAM_ONOFF, &auto_settings, 0, NULL},
    {"ladybug", "auto_exposure_value", DGC_PARAM_INT, &auto_exposure_value, 0, NULL},
    {"ladybug", "gain_value", DGC_PARAM_INT, &gain_value, 1, ParamCB(ladybug_set_exposure)},
    {"ladybug", "shutter_value", DGC_PARAM_INT, &shutter_value, 1, ParamCB(ladybug_set_exposure)},
    {"ladybug", "sync_velodyne", DGC_PARAM_ONOFF, &sync_velodyne, 1, ParamCB(ladybug_set_trigger)},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int 
main(int argc, char *argv[]) 
{
  IpcInterface *ipc = NULL;
  pthread_t capture_thread;

  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  signal( SIGTERM, &shutdown );
  signal( SIGINT, &shutdown );

  ladybug_init_firewire(0);
  dgc_info("Camera initialized");

  lbug_pkt = new LadybugPacket;

  lbug_interface = new LadybugShmInterface;
  if(lbug_interface->CreateServer() < 0) 
    dgc_fatal_error("Could not open ladybug interface.");

  /* subscribe to applanix messages */
  pose.smooth_x  = 0;
  pose.smooth_y  = 0;
  pose.smooth_z  = 0;
  pose.roll      = 0;
  pose.pitch     = 0;
  pose.yaw       = 0;
  ipc->Subscribe(ApplanixPoseID, &pose, DGC_SUBSCRIBE_ALL, &applanix_mutex);

  pthread_create( &capture_thread, NULL, dc1394_thread, NULL );
  usleep(300000);
  ipc->Dispatch();
  return 0;
}
