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
double g_ladybug_sync_offset;
double g_ladybug_synchronizer_angle;

bool g_show_intensities = false;
dgc_velodyne_config_p g_velodyne_config = NULL;
char *g_cal_filename = NULL;
dgc_transform_t g_velo_offset;
dgc_velodyne_index g_velodyne_index;
dgc_velodyne_file_p g_velodyne_file;
bool g_show_points = true;


double g_focal_length_modifier = 0.0;
double g_focal_increment = 10;
double g_control_multiplier = 1;
double g_base_translation = 0.1;
double g_base_rotation = 0.025;
double g_rotation_amount = g_base_rotation * g_control_multiplier;
double g_translation_amount = g_base_translation * g_control_multiplier;
MatrixXf g_transform;


void rotate(double roll, double pitch, double yaw) {
  MatrixXf Rx = MatrixXf::Identity(4,4);
  Rx(1,1) = cos(roll);
  Rx(2,2) = cos(roll);
  Rx(2,1) = -sin(roll);
  Rx(1,2) = sin(roll);

  MatrixXf Ry = MatrixXf::Identity(4,4);
  Ry(0,0) = cos(pitch);
  Ry(2,2) = cos(pitch);
  Ry(0,2) = -sin(pitch);
  Ry(2,0) = sin(pitch);

  MatrixXf Rz = MatrixXf::Identity(4,4);
  Rz(0,0) = cos(yaw);
  Rz(1,1) = cos(yaw);
  Rz(1,0) = -sin(yaw);
  Rz(0,1) = sin(yaw);

  g_transform = g_transform * Rx * Ry * Rz;

//  cout << "-----" << g_transform << endl;
}

void translate(double x, double y, double z) {
  g_transform(3, 0) += x;
  g_transform(3, 1) += y;
  g_transform(3, 2) += z;
  //cout << "-----" << g_transform << endl;
}

void incrementFocalLength(double val) {
  g_focal_length_modifier += val;
  cout << "New focal length modifier: " << g_focal_length_modifier << endl;
}


int displayOverlay(char* window, IplImage* img, const MatrixXf& pointcloud, const VectorXf& intensities, const LadybugModel& lb3) {
  IplImage* vis = cvCloneImage(img);
  
  // -- Draw the points in the image.
  if(g_show_points) { 
    for(int j=0; j<pointcloud.rows(); ++j) {
      int u = -1;
      int v = -1;
      lb3.intrinsics_.cameras_[CAMERA_NUM].xyzToUVRect(pointcloud(j,0), pointcloud(j,1), pointcloud(j,2), &u, &v, g_focal_length_modifier);
      if(u == -1 && v == -1)
	continue;
      cvCircle(vis, cvPoint(u, v), 0, colorMap(pointcloud.row(j), intensities(j), g_show_intensities), 0);	
    }
  }

  cvShowImage(window, vis);
  int increment = 0;
  char key = cvWaitKey();
  switch(key) {
  case 'r':
    eigen_extensions::save(g_transform, g_base_cam_extrinsics_path);
    cout << "Saved extrinsics for camera " << CAMERA_NUM << " to " << LadybugExtrinsics::getTransformFilename(CAMERA_NUM) << endl;
    break;

    
    // -- Gain control.
  case '*':
    g_control_multiplier *= 2;
    g_rotation_amount = g_control_multiplier * g_base_rotation;
    g_translation_amount = g_control_multiplier * g_base_translation;
    break;
  case '/':
    g_control_multiplier /= 2;
    g_translation_amount = g_control_multiplier * g_base_translation;
    g_rotation_amount = g_control_multiplier * g_base_rotation;
    break;

    // -- Translation keys.
  case 's':
    translate(0, 0, g_translation_amount);
    break;
  case 'w':
    translate(0, 0, -g_translation_amount);
    break;
  case 'a':
    translate(g_translation_amount, 0, 0);
    break;
  case 'd':
    translate(-g_translation_amount, 0, 0);
    break;
  case 'e':
    translate(0, g_translation_amount, 0);
    break;
  case 'q':
    translate(0, -g_translation_amount, 0);
    break;
    
    // -- Rotation keys.
  case 'D':
    rotate(0, 0, g_rotation_amount);
    break;
  case 'A':
    rotate(0, 0, -g_rotation_amount);
    break;
  case 'W':
    rotate(g_rotation_amount, 0, 0);
    break;
  case 'S':
    rotate(-g_rotation_amount, 0, 0);
    break;
  case 'Q':
    rotate(0, g_rotation_amount, 0);
    break;
  case 'E':
    rotate(0, -g_rotation_amount, 0);
    break;

    // -- Other
  case 'f':
    incrementFocalLength(g_control_multiplier * g_focal_increment);
    break;
  case 'F':
    incrementFocalLength(-g_control_multiplier * g_focal_increment);
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
  case 'n':
    increment = 1;
    break;
  case 'N':
    increment = 10;
    break;
  case 'p':
    increment = -1;
    break;
  case 'P':
    increment = -10;
    break;
  case 'i':
    g_show_intensities = !g_show_intensities;
    break;
  default:
    break;
  }
  cvReleaseImage(&vis);

  return increment;
}


void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &g_velo_offset,    0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "base_cam_extrinsics_path", DGC_PARAM_FILENAME, &g_base_cam_extrinsics_path, 0, NULL},
    {"ladybug", "base_cam", DGC_PARAM_INT, &g_base_cam, 0, NULL},
    {"ladybug", "default_sync_offset", DGC_PARAM_DOUBLE, &g_ladybug_sync_offset, 0, NULL},
    {"ladybug", "synchronizer_angle", DGC_PARAM_DOUBLE, &g_ladybug_synchronizer_angle, 0, NULL},
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
  LadybugModel lb3(g_base_cam, g_base_cam_extrinsics_path, g_intrinsics_path);
  cout << lb3.intrinsics_.cameras_[CAMERA_NUM].status() << endl;
  assert(CAMERA_NUM == g_base_cam); // Propagating transforms doesn't work yet.
  g_transform = lb3.extrinsics_.veloToCamera_[CAMERA_NUM];
  cout << "Initial transform is " << endl << g_transform << endl;
  
  // -- Get the first spin / ladybug image pair.
  int spin_num = findFirstMatchingPair(lbp, g_velodyne_index);
  cout << "First spin with matching ladybug data is spin " << spin_num << endl;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	    &applanix_lat, &applanix_lon, &applanix_alt);
  
  while(true) { 

//     cout << "ladybug timestamp - displayed spin timestamp: "
// 	 << lbp.getTimestamp() - g_velodyne_index.spin[spin_num].pose[0].timestamp << endl;
    
    // -- Display the results.
    IplImage* warp = dgcToIpl(*lbp.cameraImage(CAMERA_NUM));
    IplImage* rect = lb3.intrinsics_.cameras_[CAMERA_NUM].rectify(warp);
    
    MatrixXf cloud;
    VectorXf intensity;
    getSpinInVeloCoords(spin, g_ladybug_synchronizer_angle, g_velo_offset, &cloud, &intensity);
    int increment = displayOverlay("Calibration", rect, cloud * g_transform, intensity, lb3);
  
    cvReleaseImage(&warp);
    cvReleaseImage(&rect);
    
    // -- Get the next ladybug image.
    spin_num += increment;
    if(spin_num >= g_velodyne_index.num_spins)
      spin_num = g_velodyne_index.num_spins - 1;
    else if(spin_num < 0)
      spin_num = 0;
    
    double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; // Allows for un-sychronized velo & ladybug.
    lbp.readTimestampPacket(vlf_timestamp + g_ladybug_sync_offset);
    
    // -- Load the next spin.
    spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
  }
}
