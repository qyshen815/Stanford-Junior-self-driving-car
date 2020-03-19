#include <iostream>
#include <string>
#include <vector>

#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <camera_interface.h>
#include <camera_shm_interface.h>
#include <dc1394camera.h>

/*** GLOBAL VARIABLES (non-boolean) ***/
static double mbytes_               = 0;
static double mbytes_total_         = 0;
static int images_                  = 0;
static int images_total_            = 0;

using namespace dgc;
using namespace vlr;

DC1394Camera* camera=NULL;

void 
shutdown( __attribute__ ((unused)) int signal ) 
{
  fprintf(stderr,  "shutting down...");
  camera->close();
  fprintf(stderr,"done\n");
  exit(0);
}

int main(int argc __attribute__ ((unused)), char **argv)
{
  double start_time, cur_time, last_time, delta_s, n_bytes;
  CameraInterface *camera_interface=NULL;
  
  IpcInterface *ipc = NULL;
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  camera_interface = new CameraShmInterface;
  if (camera_interface->CreateServer(0) < 0) {
    dgc_die("Could not open camera interface");
  }
  
  camera = new DC1394Camera;

  if (!camera->open(0)) {
    dgc_die("Could not initialize camera.");
  }
  
  signal( SIGTERM, &shutdown );
  signal( SIGINT, &shutdown );

  last_time = start_time = dgc_get_time();
  
  camera->startCapture();

  while (1) {

    cur_time = dgc_get_time();
    delta_s = cur_time - last_time;
    
    n_bytes = camera->writeImage(camera_interface); 
    if( n_bytes==0 ) {
      continue;
    }
    
    images_++;
    images_total_++;
    
    mbytes_ += (n_bytes) / (1024.0*1024.0);
    mbytes_total_ += (sizeof(CameraImage) / (1024*1024));
    
    if (delta_s > 0.2) {
      fprintf(stderr, 
	      "\r                                                \r"
	      "# CAMERA: ... %03.2f MB/s (%.2f imgs/s)", 
	      dgc_avg_times(delta_s, mbytes_, 0), 
	      dgc_avg_times(delta_s, images_, 1));
      mbytes_ = 0;
      images_ = 0;
      last_time = cur_time;
    }
    
    usleep(10000);
  }
  
  return 0;
}

