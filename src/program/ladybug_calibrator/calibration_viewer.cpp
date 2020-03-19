#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include "ladybug_model.h"
#include <transform.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <velo_support.h>
#include <ladybug_playback.h>

#include <highgui.h>
#include <cv.h>
#include <Eigen/Core>


#include <boost/filesystem.hpp>

using namespace dgc;
using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

//! The camera number we've calibrated the velodyne-ladybug transform to.
int g_base_cam;
//! The camera number to visualize.
int g_view_cam;
double g_brightness = 1.0;

char *g_intrinsics_path;
char *g_extrinsics_path;
double g_ladybug_sync_offset;
double g_ladybug_synchronizer_angle;

bool g_show_intensities = false;
dgc_velodyne_config_p g_velodyne_config = NULL;
char *g_cal_filename = NULL;
dgc_transform_t g_velo_offset;
dgc_velodyne_index g_velodyne_index;
dgc_velodyne_file_p g_velodyne_file;
bool g_show_points = false;
bool g_pause = false;

IplImage *g_vis = NULL;

int displayOverlay(char* window,
		   IplImage* img,
		   const MatrixXf& pointcloud,
		   const VectorXf& intensities,
		   const LadybugModel& lb3)
{
  // -- Draw the points in the image.
  if(g_show_points && g_view_cam != 5) { 
    for(int j=0; j<pointcloud.rows(); ++j) {
      int u = -1;
      int v = -1;
      lb3.intrinsics_.cameras_[g_view_cam].xyzToUVWarp(pointcloud(j,0), pointcloud(j,1), pointcloud(j,2), &u, &v);
      if(u == -1 && v == -1)
	continue;
      cvCircle(img, cvPoint(u, v), 1, colorMap(pointcloud.row(j), intensities(j), g_show_intensities), 0);	
    }
  }

  if(!g_vis)
    g_vis = cvCloneImage(img);
  cvConvertScale(img, g_vis, g_brightness);
  cvShowImage(window, g_vis);
  
  int increment = 1;
  if(g_pause)
    increment = 0;
  
  char key = cvWaitKey(10);
  switch(key) {
  case '0':
    g_view_cam = 0;
    break;
  case '1':
    g_view_cam = 1;
    break;
  case '2':
    g_view_cam = 2;
    break;
  case '3':
    g_view_cam = 3;
    break;
  case '4':
    g_view_cam = 4;
    break;
  case '5':
    g_view_cam = 5;
    break;
  case '+':
    g_ladybug_sync_offset += 0.1;
    cout << "Sync offset: " << g_ladybug_sync_offset << endl;
    break;
  case '-':
    g_ladybug_sync_offset -= 0.1;
    cout << "Sync offset: " << g_ladybug_sync_offset << endl;
    break;
  case 'v':
    g_show_points = !g_show_points;
    break;
  case 'i':
    g_show_intensities = !g_show_intensities;
    break;
  case 'q':
    exit(0);
    break;
  case 'P':
    g_pause = !g_pause;
    break;
  case 'n':
    if(g_pause)
      increment = 1;
    break;
  case 'p':
    if(g_pause)
      increment = -1;
    break;
  case 'b':
    g_brightness += 0.1;
    break;
  case 'd':
    g_brightness -= 0.1;
    break;
  default:
    break;
  }

  return increment;
}


void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &g_velo_offset,    0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "extrinsics_path", DGC_PARAM_FILENAME, &g_extrinsics_path, 0, NULL},
    {"ladybug", "base_cam", DGC_PARAM_INT, &g_base_cam, 0, NULL},
    {"ladybug", "default_sync_offset", DGC_PARAM_DOUBLE, &g_ladybug_sync_offset, 0, NULL},
    {"ladybug", "synchronizer_angle", DGC_PARAM_DOUBLE, &g_ladybug_synchronizer_angle, 0, NULL},
  };

  pint->InstallParams(argc, argv, params, sizeof(params)/sizeof(params[0]));
}


int main(int argc, char** argv) {
  if(argc != 3 && argc != 4) { 
    cout << "Usage: " << argv[0] << " VLF LLF [IMAGE_DIR]" << endl;
    cout << "  If IMAGE_DIR is provided, it will be created and filled with images for later processing into video." << endl;
    return 1;
  }

  // -- Set up image dump.
  string image_dir;
  bool dump_images = false;
  if(argc == 4) {
    dump_images = true;
    image_dir = argv[3];
    if(bfs::exists(image_dir)) {
      cout << image_dir << " already exists, will not overwrite." << endl;
      return -1;
    }
    bfs::create_directory(image_dir);
  }
  
  // -- Connect to IPC and get the velo offset and cal filename.
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  if(ipc->ConnectLocked("calibration_viewer") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete ipc;
  delete pint;
  g_view_cam = g_base_cam;

  // -- Read in the velodyne calibration file to g_velodyne_config.
  dgc_velodyne_get_config(&g_velodyne_config);
  if(dgc_velodyne_read_calibration(g_cal_filename, g_velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  // Set g_velodyne_config->offset to be g_velo_offset.
  dgc_velodyne_integrate_offset(g_velo_offset, g_velodyne_config);
  
  // -- Load logfiles.
  cout << "Loading Velodyne Log File " << argv[1] << endl;
  g_velodyne_file = dgc_velodyne_open_file(argv[1]);
  if(g_velodyne_file == NULL) {
    cout << "Could not open " << argv[1] << " for reading." << endl;
    return 1;
  }

  char index_filename[300];
  strcpy(index_filename, argv[1]);
  strcat(index_filename, ".index.gz");
  g_velodyne_index.load(index_filename);
  
  cout << "Loading Ladybug Log File " << argv[2] << endl;
  LadybugPlayback lbp;
  lbp.openLLF(argv[2]);

  // -- Load the ladybug model.
  cout << "Loading Ladybug intrinsics at " << g_intrinsics_path << endl;
  cout << "Base cam is " << g_base_cam << ", extrinsics at " << g_extrinsics_path << endl;
  //LadybugModel lb3(g_base_cam, g_extrinsics_path, g_intrinsics_path);
  LadybugModel lb3(g_extrinsics_path, g_intrinsics_path);
  for(int i = 0; i < 6; ++i)
    cout << lb3.intrinsics_.cameras_[i].status() << endl;
    
  // -- Get the first spin / ladybug image pair.
  int spin_num = findFirstMatchingPair(lbp, g_velodyne_index);
  cout << "First spin with matching ladybug data is spin " << spin_num << endl;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	    &applanix_lat, &applanix_lon, &applanix_alt);

  // -- Skip the first few frames, as they tend to be bad for our logs.
  int skip = 50;
  spin_num += skip;
  double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; // Allows for un-sychronized velo & ladybug.
  lbp.readTimestampPacket(vlf_timestamp + g_ladybug_sync_offset);
  
  while(true) {
    
    // -- Display the results.
    IplImage* warp = dgcToIpl(*lbp.cameraImage(g_view_cam));
    if(!warp)
      break;
    
    MatrixXf cloud;
    VectorXf intensity;
    getSpinInVeloCoords(spin, g_ladybug_synchronizer_angle, g_velo_offset, &cloud, &intensity);
    int increment = displayOverlay("Calibration", warp, cloud * lb3.extrinsics_.veloToCamera_[g_view_cam], intensity, lb3);
    if(dump_images) {
      ostringstream path;
      path << image_dir << "/" << setprecision(16) << lbp.getTimestamp() << "_cam" << g_view_cam << ".png";
      cvSaveImage(path.str().c_str(), warp);
    }
    cvReleaseImage(&warp);
    
    // -- Get the next ladybug image.
    spin_num += increment;
    if(spin_num >= g_velodyne_index.num_spins)
      break;
    double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; // Allows for un-sychronized velo & ladybug.
    lbp.readTimestampPacket(vlf_timestamp + g_ladybug_sync_offset);
    
    // -- Load the next spin.
    spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
  }

  return 0;
}
