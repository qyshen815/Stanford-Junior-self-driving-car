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


using namespace dgc;
using namespace std;
using namespace Eigen;


// At some point this will switch this over to be any camera you want.
// For now we are only supporting a single camera.
#define CAMERA_NUM g_base_cam
//#define CAMERA_NUM (getenv("CAMERA_NUM") ? atoi(getenv("CAMERA_NUM")) : 0)

char *g_intrinsics_path;
char *g_base_cam_extrinsics_path;
int g_base_cam;
      
//! How many velodyne spins to skip ahead to match the camera data.
bool g_show_intensities = false;
int g_velo_sync_offset = 0;
bool g_show_points = true;

dgc_velodyne_config_p g_velodyne_config = NULL;
char *g_cal_filename = NULL;
dgc_transform_t g_velo_offset;
dgc_velodyne_index g_velodyne_index;
dgc_velodyne_file_p g_velodyne_file;


int projectScan(const dgc_velodyne_scan_t& scan,
		 dgc_transform_t velo_offset,
		 int camera_num,
		 const LadybugModel& lbmodel,
		 IplImage* vis)
{
  dgc_transform_t original;
  dgc_transform_t inv;
  dgc_transform_rpy(original,
		    velo_offset,
		    scan.robot.roll,
		    scan.robot.pitch,
		    scan.robot.yaw);
  dgc_transform_inverse(original, inv);

  int num_valid = 0;
  for(int i = 0; i < VELO_BEAMS_IN_SCAN; ++i) {
    if(scan.p[i].range < 0.01)
      continue;

    // -- Put point into velo frame.
    double x = scan.p[i].x / 100.; //Convert from cm to meters.
    double y = scan.p[i].y / 100.;
    double z = scan.p[i].z / 100.;
    dgc_transform_point(&x, &y, &z, inv);
    
    // -- Project velo frame point into image.
    int u, v;
    bool in_img = lbmodel.projectVeloToWarpedImage(camera_num, x, y, z, &u, &v);
    if(in_img) { 
      ++num_valid;
    }
    
    // -- Draw.
    VectorXf pt(4);
    pt(0) = x;
    pt(1) = y;
    pt(2) = z;
    pt(3) = 1;
    cvCircle(vis, cvPoint(u, v), 3, depthMap(pt), -1);
  }

  return num_valid;
}

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &g_velo_offset,    0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "base_cam_extrinsics_path", DGC_PARAM_FILENAME, &g_base_cam_extrinsics_path, 0, NULL},
    {"ladybug", "base_cam", DGC_PARAM_INT, &g_base_cam, 0, NULL},
  };

  pint->InstallParams(argc, argv, params, sizeof(params)/sizeof(params[0]));
}


int main(int argc, char** argv) {
  if(argc != 3) { 
    cout << "Usage: " << argv[0] << " VLF LLF" << endl;
    return 1;
  }
  
  // -- Connect to IPC and get the velo offset and cal filename.
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  if(ipc->ConnectLocked("calibration_viewer") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete ipc;
  delete pint;

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
  cout << "Base cam is " << g_base_cam << ", extrinsics at " << g_base_cam_extrinsics_path << endl;
  LadybugModel lbmodel(g_base_cam, g_base_cam_extrinsics_path, g_intrinsics_path);
  cout << lbmodel.intrinsics_.cameras_[CAMERA_NUM].status() << endl;
  assert(CAMERA_NUM == g_base_cam); // Propagating transforms doesn't work yet.
  MatrixXf transform = lbmodel.extrinsics_.veloToCamera_[CAMERA_NUM];
  
  // -- Get the first spin / ladybug image pair.
  int spin_num = findFirstMatchingPair(lbp, g_velodyne_index);
  cout << "First spin with matching ladybug data is spin " << spin_num << endl;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	    &applanix_lat, &applanix_lon, &applanix_alt);
  int scan_num = 0;



  while(true) { 
    // -- Display the results.
    IplImage* warp = dgcToIpl(*lbp.cameraImage(CAMERA_NUM));
    IplImage* vis = cvCloneImage(warp);    
    
    // -- Project scan into image.
    int num_valid = projectScan(spin.scans[scan_num], g_velo_offset, CAMERA_NUM, lbmodel, vis);
    while(num_valid == 0 && scan_num < spin.num_scans) { 
      ++scan_num;
      num_valid = projectScan(spin.scans[scan_num], g_velo_offset, CAMERA_NUM, lbmodel, vis);
    }
   
    cvShowImage("Projection", vis);

    // -- Menu: change scan num or spin num.
    char key = cvWaitKey();
    switch(key) {
    case 'n':
      scan_num += 1;
      break;
    case 'N':
      spin_num += 10;
      break;
    case 'p':
      scan_num -= 1;
      break;
    case 'P':
      spin_num -= 10;
      break;
    case 'q':
      exit(0);
      break;
    default:
      break;
    }

    // -- Clean up.
    cvReleaseImage(&warp);
    cvReleaseImage(&vis);
    
    // -- Get the next ladybug image.
    double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; // Allows for un-sychronized velo & ladybug.
    lbp.readTimestampPacket(vlf_timestamp);
    
    // -- Load the next spin.
    spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
  }
}
