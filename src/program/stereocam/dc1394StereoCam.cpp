#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <global.h>
#include <ipc_std_interface.h>
#include <camera_interface.h>
#include <camera_shm_interface.h>
#include <dc1394stereo.h>

/*** GLOBAL VARIABLES (non-boolean) ***/
static double mbytes_               = 0;
static double mbytes_total_         = 0;
static int images_                  = 0;
static int images_total_            = 0;

using namespace dgc;
using namespace vlr;

vlr::DC1394Stereo* camera=NULL;

void 
shutdown( __attribute__ ((unused)) int signal ) 
{
  fprintf(stderr,  "shutting down...");
  camera->close();
  fprintf(stderr,"done\n");
  exit(0);
}

int main(int argc, char **argv) {
  double start_time, cur_time, last_time, delta_s, n_bytes;
  
  if(argc<3) {
    std::cout << "Usage: " << argv[0] << " left_guid right_guid [color mode] [format]\n";
    std::cout << "Available cameras:\n";
    DC1394Stereo cam;
    cam.listBus();
    exit(0);
  }

  std::istringstream liss, riss;
  uint64_t left_guid, right_guid;
  liss.str(argv[1]); liss >> left_guid;
  riss.str(argv[2]); riss >> right_guid;
  IpcInterface* ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0) {
    dgc_fatal_error("Could not connect to IPC network.");
  }
  
  CameraInterface* camera_interface = new CameraShmInterface;
  if (camera_interface->CreateServer(0) < 0) {
    dgc_die("Could not open camera interface");
  }
  
  camera = new DC1394Stereo;

  try {
    camera->open(left_guid, right_guid);
  }
  catch(Exception& e) {
    std::cout << e.what() << std::endl;
    exit(-10);
  }
  
  signal( SIGTERM, &shutdown );
  signal( SIGINT, &shutdown );

  last_time = start_time = vlr::Time::current();
  
  camera->startCapture();

  while (1) {
    cur_time = vlr::Time::current();
    delta_s = cur_time - last_time;
    
    n_bytes = camera->write(*camera_interface);
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

