#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <global.h>

#include "ladybug_model.h"
#include <transform.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <velo_support.h>
#include <ladybug_playback.h>
#include "cropped_video_writer.h"

#include <highgui.h>
#include <cv.h>
#include <Eigen/Core>


#include <ros/ros.h>
#include <pcl_ros/publisher.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace dgc;
using namespace std;
using namespace Eigen;

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

  // Initialize and connect to ROS
  ros::init(argc, argv, "color_velodyne");
  ros::NodeHandle ros_node;

  pcl_ros::Publisher<pcl::PointXYZRGB> pub;
  pub.advertise(ros_node, "color_velodyne", 1);

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
  cout << "Extrinsics at " << g_extrinsics_path << endl;
  LadybugModel *lb3 = new LadybugModel(g_extrinsics_path, g_intrinsics_path);
  // -1 means we ignore to top camera
  unsigned int num_cams = lb3->intrinsics_.cameras_.size() - 1;

  // -- Get the first spin / ladybug image pair.
  int spin_num = findFirstMatchingPair(lbp, g_velodyne_index);
  cout << "First spin with matching ladybug data is spin " << spin_num << endl;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	    &applanix_lat, &applanix_lon, &applanix_alt);

  double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; // Allows for un-sychronized velo & ladybug.
  lbp.readTimestampPacket(vlf_timestamp);

  int loop = 0;
  double timer = 0;
  timer = dgc_get_time();
  while(loop < 500) {
    loop++;
    
    pcl::PointCloud<pcl::PointXYZRGB> *pclcloud = new pcl::PointCloud<pcl::PointXYZRGB>();
    pclcloud->header.frame_id = "velodyne";
    IplImage* warp[num_cams];
    for( unsigned int cam = 0; cam < num_cams; cam++ ) {
      warp[cam] = dgcToIpl(*lbp.cameraImage(cam)); 
      if(!warp[cam]) {
        fprintf(stderr, "Bad camera data!\n");
        break;
      }
    }
    printf("DGC->IPL Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
    Eigen::MatrixXf cloud;
    Eigen::VectorXf intensity;
    getSpinInVeloCoords(spin, g_ladybug_synchronizer_angle, g_velo_offset, &cloud, &intensity);
    printf("Velo conversion Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
    // Color the points
    for(int i = 0; i < cloud.rows(); i++) {
      pcl::PointXYZRGB point;
      point.x = cloud(i,0);
      point.y = cloud(i,1);
      point.z = cloud(i,2);
      uint32_t rgb = 0x00000000;
      unsigned int matched_cam;
      int u, v;
      if(lb3->projectVeloToWarpedImage(point.x, point.y, point.z, &u, &v, &matched_cam))
      {
        unsigned char *colors = (unsigned char*)cvPtr2D(warp[matched_cam], v, u);
        rgb |= (uint32_t)(*(colors+2)) << 16;
        rgb |= (uint32_t)(*(colors+1)) << 8;
        rgb |= (uint32_t)(*(colors));
      }
      // Because it's stored as a float in ROS.  Don't ask me why
      point.rgb = *reinterpret_cast<float*>(&rgb);
      pclcloud->points.push_back(point);
    }
    printf("Coloring Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
    for(unsigned cam = 0; cam < num_cams; cam++)
      cvReleaseImage(&warp[cam]);
    
    //pcl::io::savePCDFileBinary("test_pcd.pcd", *pclcloud);
    pub.publish(*pclcloud);
    delete pclcloud;

    printf("Publish Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
    // -- Get the next ladybug image.
    spin_num++;
    if(spin_num >= g_velodyne_index.num_spins)
      break;
    double vlf_timestamp = g_velodyne_index.spin[spin_num].pose[0].timestamp; 
    lbp.readTimestampPacket(vlf_timestamp);
    printf("New Ladybug Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
    
    // -- Load the next spin.
    spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
    printf("New Velo Time:   %f\n", dgc_get_time() - timer);
    timer = dgc_get_time(); 
  }

  return 0;
}
