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


#include <eigen_extensions/eigen_extensions.h>
#include <boost/filesystem.hpp>

using namespace dgc;
using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

int g_base_cam;
int g_view_cam;
char *g_intrinsics_path;
char *g_extrinsics_path;
double g_ladybug_sync_offset;
double g_ladybug_synchronizer_angle;

dgc_velodyne_config_p g_velodyne_config = NULL;
char *g_cal_filename = NULL;
dgc_transform_t g_velo_offset;
dgc_velodyne_index g_velodyne_index;
dgc_velodyne_file_p g_velodyne_file;

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

void formatData(const LadybugModel& lb3, const MatrixXf& cloud_cam, const MatrixXf& cloud_smooth,
		const VectorXf& intensity, IplImage* warp,
		const string& output_dir, double timestamp)
{
  // -- Save the image.
  ostringstream oss;
  oss << output_dir << "/" << setprecision(16) << timestamp;
  cvSaveImage((oss.str() + ".png").c_str(), warp);
  
  // -- Create the output file.
  string filename = oss.str() + ".dat";
  ofstream file(filename.c_str());

  // -- Get number of points in the image.
  int num_valid = 0;
  for(int i = 0; i < cloud_cam.rows(); ++i) {
    int u = -1;
    int v = -1;
    lb3.intrinsics_.cameras_[g_view_cam].xyzToUVWarp(cloud_cam(i, 0), cloud_cam(i, 1), cloud_cam(i, 2), &u, &v);
    if(u == -1 && v == -1)
      continue;
    ++num_valid;
  }
  file.write((char*)&num_valid, sizeof(int));

  // -- Write out data for each point.
  for(int i = 0; i < cloud_cam.rows(); ++i) {
    int u = -1;
    int v = -1;
    lb3.intrinsics_.cameras_[g_view_cam].xyzToUVWarp(cloud_cam(i, 0), cloud_cam(i, 1), cloud_cam(i, 2), &u, &v);
    if(u == -1 && v == -1)
      continue;
    file.write((const char*)&cloud_smooth.coeff(i, 0), sizeof(float));
    file.write((const char*)&cloud_smooth.coeff(i, 1), sizeof(float));
    file.write((const char*)&cloud_smooth.coeff(i, 2), sizeof(float));
    file.write((const char*)&cloud_cam.coeff(i, 0), sizeof(float));
    file.write((const char*)&cloud_cam.coeff(i, 1), sizeof(float));
    file.write((const char*)&cloud_cam.coeff(i, 2), sizeof(float));
    file.write((const char*)&intensity.coeff(i), sizeof(float));
    file.write((const char*)&u, sizeof(int));
    file.write((const char*)&v, sizeof(int));
  }

  file.close();
  cout << "Wrote to " << filename << endl;
}

int main(int argc, char** argv) {
  if(argc != 3 && argc != 4) { 
    cout << "Usage: " << argv[0] << " VLF LLF OUTPUT_DIR" << endl;
    cout << "  OUTPUT_DIR must not yet exist.  It will be created." << endl;
    return 1;
  }

  // -- Set up image dump.
  string output_dir = argv[3];
  if(bfs::exists(output_dir)) {
    cout << output_dir << " already exists, will not overwrite." << endl;
    return -1;
  }
  bfs::create_directory(output_dir);
    
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
    
    // -- Get the next image.
    IplImage* warp = dgcToIpl(*lbp.cameraImage(g_view_cam));
    if(!warp)
      break;

    MatrixXf cloud_velo, cloud_smooth;
    VectorXf intensity;
    getSpinInVeloCoords(spin, g_ladybug_synchronizer_angle, g_velo_offset, &cloud_velo, &intensity);
    getSpinInSmoothCoords(spin, &cloud_smooth, &intensity);

    // -- Project points into the image and save the data.
    formatData(lb3, cloud_velo * lb3.extrinsics_.veloToCamera_[g_view_cam], cloud_smooth, intensity, warp, output_dir, lbp.getTimestamp());
    cvReleaseImage(&warp);
    
    // -- Get the next ladybug image.
    ++spin_num;
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
