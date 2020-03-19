#include <iomanip>
#include <highgui.h>
#include <cv.h>
#include "ladybug_model.h"
#include <Eigen/Core>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <math.h>
#include <assert.h>
#include <epnp.h>
#include <sstream>
#include <transform.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <velo_support.h>
#include <ladybug_playback.h>

using namespace dgc;
using namespace std;
using namespace Eigen;


//! Which camera we're calibrating.
int g_calibration_camera;
int g_base_cam;

char *g_intrinsics_path;
char *g_extrinsics_path;


int g_camera_mouse[3] = {0, 0, 0};
int g_depth_mouse[3] = {0, 0, 0};
bool g_show_intensities = false;

char *g_cal_filename = NULL;
dgc_velodyne_config_p g_velodyne_config = NULL;
//! How many velodyne spins to skip ahead to match the camera data.

int g_velo_sync_offset = 0;
double g_ladybug_synchronizer_angle;
dgc_transform_t g_velo_offset;
dgc_velodyne_index g_velodyne_index;
dgc_velodyne_file_p g_velodyne_file;
MatrixXf g_initial_transform;

int findClosest(IplImage* index, int query_u, int query_v, int* closest_u, int* closest_v) {
  assert(index->depth = IPL_DEPTH_32S);
  int max_radius = 10;

  for(int radius = 0; radius<max_radius; ++radius) {

    float min_dist = FLT_MAX;
    int best_u = -1;
    int best_v = -1;
    int best_idx = -1;

    for(int u = query_u - radius; u <= query_u + radius; ++u) {
      if(u < 0 || u >= index->width)
        continue;
      for(int v = query_v - radius; v <= query_v + radius; ++v) {
        if(v < 0 || v >= index->height)
          continue;
        int idx = ((int*)(index->imageData + index->widthStep * v))[u];
        if(idx != -1) {
          float dist = pow((float)(u - query_u), 2) + pow((float)(v - query_v), 2);
          if(dist < min_dist) {
            min_dist = dist;
            best_idx = idx;
            best_u = u;
            best_v = v;
          }
        }
      }
    }

    if(min_dist != FLT_MAX) { 
      *closest_u = best_u;
      *closest_v = best_v;
      return best_idx;
    }
  }

  // -- If no neighboring point is found.
  *closest_u = -1;
  *closest_v = -1;
  return -1;
}

void mouseHandler(int event, int x, int y, __attribute__((unused)) int flags, void* param) {
  switch(event){
    case CV_EVENT_MOUSEMOVE:
      ((int*)param)[0] = x;
      ((int*)param)[1] = y;
      break;

    case CV_EVENT_LBUTTONDOWN:
      ((int*)param)[0] = x;
      ((int*)param)[1] = y;
      ((int*)param)[2] = 1;
      break;

    case CV_EVENT_LBUTTONUP:
      ((int*)param)[0] = x;
      ((int*)param)[1] = y;
      ((int*)param)[2] = 0;
      break;
  }
}


void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &g_velo_offset,    0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "extrinsics_path", DGC_PARAM_FILENAME, &g_extrinsics_path, 0, NULL},
    {"ladybug", "base_cam", DGC_PARAM_INT, &g_base_cam, 0, NULL},
    {"ladybug", "synchronizer_angle", DGC_PARAM_DOUBLE, &g_ladybug_synchronizer_angle, 0, NULL},
  };

  pint->InstallParams(argc, argv, params, sizeof(params)/sizeof(params[0]));
}

//! @param img The image for camera g_calibration_camera
//! @param pointcloud In velodyne coordinates.
void collectPointCorrespondences(IplImage* img,
				 const MatrixXf& pointcloud,
				 const VectorXf& intensities,
				 const LadybugModel& lb3,
				 vector<VectorXf>* img_pts,
				 vector<VectorXf>* xyz_pts)
{
  assert(img->width == lb3.intrinsics_.cameras_[g_calibration_camera].rect_cols_);
  assert(img->height == lb3.intrinsics_.cameras_[g_calibration_camera].rect_rows_);
  
  // -- Set up the depth image and the index.
  IplImage* depth = cvCreateImage(cvSize(img->width, img->height), img->depth, 3);
  IplImage* index = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_32S, 1);
  cvSet(depth, cvScalar(0,0,0));
  cvSet(index, cvScalar(-1));

  // -- Draw the points in the image and fill the index.
  for(int j=0; j<pointcloud.rows(); ++j) {
    int u = -1;
    int v = -1;
    lb3.projectVeloToRectifiedImage(g_calibration_camera, pointcloud(j, 0), pointcloud(j, 1), pointcloud(j, 2), &u, &v);
    if(u == -1 && v == -1)
      continue;

    CvScalar color = colorMap(pointcloud.row(j), intensities(j), g_show_intensities);
    cvCircle(depth, cvPoint(u, v), 0, color, 0);

    ((int*)(index->imageData + index->widthStep * v))[u] = j;
  }

  IplImage* depth_vis = cvCloneImage(depth);
  IplImage* img_vis = cvCloneImage(img);

  bool done = false;
  while(!done) { 
    cvCopy(depth, depth_vis);
    cvCopy(img, img_vis);
    cvShowImage("Depth", depth_vis);
    cvShowImage("Camera", img_vis);

    int idx = -1;
    int u = -1;
    int v = -1;
    char key = cvWaitKey(10);
    while(key != 'a') {
      if(g_depth_mouse[2] == 1) {
        // -- Draw the circle.
        int query_u = g_depth_mouse[0];
        int query_v = g_depth_mouse[1];
        cvCopy(depth, depth_vis);
        int nearest_u = -1;
        int nearest_v = -1;
        idx = findClosest(index, query_u, query_v, &nearest_u, &nearest_v);
        if(idx != -1) {
          cvCircle(depth_vis, cvPoint(nearest_u, nearest_v), 2, colorMap(pointcloud.row(idx), intensities(idx), g_show_intensities), 2);
          cvShowImage("Depth", depth_vis);
        }
      }
      if(g_camera_mouse[2] == 1) {
        u = g_camera_mouse[0];
        v = g_camera_mouse[1];
        cvCopy(img, img_vis);
        cvCircle(img_vis, cvPoint(u, v), 2, cvScalar(255, 0, 0), 2);
        cvShowImage("Camera", img_vis);
      }
      key = cvWaitKey(10);
      if(key == 'd') {
        done = true;
        break;
      }
    }

    if(idx != -1 && u != -1 && v != -1) {
      cout << "Adding point " << u << " " << v << ", xyz = " << pointcloud.block(idx, 0, 1, 3) << endl;
      VectorXf img_pt(2);
      img_pt(0) = u;
      img_pt(1) = v;
      img_pts->push_back(img_pt);

      VectorXf xyz_pt(3);
      xyz_pt(0) = pointcloud(idx, 0);
      xyz_pt(1) = pointcloud(idx, 1);
      xyz_pt(2) = pointcloud(idx, 2);
      xyz_pts->push_back(xyz_pt);
    }
    key = cvWaitKey(10);
  }

  // -- Cleanup.
  cvReleaseImage(&index);
  cvReleaseImage(&depth);
  cvReleaseImage(&depth_vis);
  cvReleaseImage(&img_vis);
}

void enforceLogBounds(int* num)
{ 
  if(*num < 0)
    *num = 0;
  else if(*num >= g_velodyne_index.num_spins)
    *num = g_velodyne_index.num_spins - 1;
}

void runCalibrationInterface(LadybugPlayback& lbp,
			     const LadybugModel& lb3,
			     vector<VectorXf>* img_pts,
			     vector<VectorXf>* xyz_pts)
{

  int spin_num = findFirstMatchingPair(lbp, g_velodyne_index);
  cout << "First spin with matching ladybug data is spin " << spin_num << endl;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
      &applanix_lat, &applanix_lon, &applanix_alt);


  while(true) {
    // -- Get the rectified image.
    IplImage* warp = dgcToIpl(*lbp.cameraImage(g_calibration_camera));
    IplImage* rect = lb3.intrinsics_.cameras_[g_calibration_camera].rectify(warp);
    cvShowImage("Camera", rect);

    MatrixXf cloud;
    VectorXf intensity;
    getSpinInVeloCoords(spin, g_ladybug_synchronizer_angle, g_velo_offset, &cloud, &intensity);
    //spinDgcToEigen(spin, &cloud, &intensity);

    // -- Input.
    char key = cvWaitKey();
    switch(key) {
    case 'i':
      g_show_intensities = !g_show_intensities;
      break;
    case 'q':
      return;
      break;
    case 'n':
      ++spin_num;
      break;
    case 'N':
      spin_num += 10;
      break;
    case 'p':
      --spin_num;
      break;
    case 'P':
      spin_num -= 10;
      break;
    case 'c':
      collectPointCorrespondences(rect, cloud, intensity, lb3, img_pts, xyz_pts);
      break;
    case '+':
      ++g_velo_sync_offset;
      cout << "Sync offset: displaying velo as " << g_velo_sync_offset << " frames ahead of the ladybug." << endl;
      break;
    case '-':
      --g_velo_sync_offset;
      cout << "Sync offset: displaying velo as " << g_velo_sync_offset << " frames ahead of the ladybug." << endl;
      break;
    default:
      break;
    }

    // -- Get the next ladybug image.
    enforceLogBounds(&spin_num);
    int offset_spin_num = spin_num - g_velo_sync_offset;
    enforceLogBounds(&offset_spin_num);
    double vlf_timestamp = g_velodyne_index.spin[offset_spin_num].pose[0].timestamp;
    lbp.readTimestampPacket(vlf_timestamp);
    
    // -- Load the next spin.
    spin.load(g_velodyne_file, g_velodyne_config, &g_velodyne_index, spin_num,
        &applanix_lat, &applanix_lon, &applanix_alt);
    cout << "On spin " << spin_num << endl;

    // -- Check the timestamps.
    double thresh = 0.11;
    double spin_start = g_velodyne_index.spin[spin_num].pose[0].timestamp;
    double spin_end = g_velodyne_index.spin[spin_num].pose[g_velodyne_index.spin[spin_num].num_poses-1].timestamp;
    if(fabs(spin_start - lbp.getTimestamp()) > thresh || fabs(spin_end - lbp.getTimestamp()) > thresh) {
      cout << "WARNING: velo and ladybug timestamps differ by more than " << thresh << endl;
      cout << "Relative velo timestamps: " << spin_start - lbp.getTimestamp() << " to " << spin_end - lbp.getTimestamp() << endl;
    }

    // -- Clean up.
    cvReleaseImage(&warp);
    cvReleaseImage(&rect);
  }   
}


MatrixXf solvePNP(const LadybugModel& lb3, const vector<VectorXf>& img_pts, const vector<VectorXf>& xyz_pts) { 
  assert(img_pts.size() == xyz_pts.size());
  for(size_t i = 0; i < img_pts.size(); ++i)
    assert(img_pts[i].rows() == 2 && xyz_pts[i].rows() == 3);
  if(img_pts.size() < 4) {
    cout << img_pts.size() << " correspondences is not enough." << endl;
    return MatrixXf();
  }

  epnp pnp;
  pnp.set_maximum_number_of_correspondences(img_pts.size());
  const LadybugCameraIntrinsics& cam = lb3.intrinsics_.cameras_[g_calibration_camera];
  cout << cam.rect_center_u_ << " " <<  cam.rect_center_v_ << " " <<  cam.focal_length_ << endl;
  pnp.set_internal_parameters(cam.rect_center_u_, cam.rect_center_v_, cam.focal_length_, cam.focal_length_);

  for(size_t i=0; i<img_pts.size(); ++i) {
    cout << "Correspondence " << i << ": " << img_pts[i].transpose() << ", " << xyz_pts[i].transpose() << endl;
    pnp.add_correspondence(xyz_pts[i](0), xyz_pts[i](1), xyz_pts[i](2), img_pts[i](0), img_pts[i](1));
  }

  double R[3][3];
  double T[3];
  timeval start, end;
  gettimeofday(&start, NULL);
  pnp.compute_pose(R, T);
  gettimeofday(&end, NULL);
  pnp.print_pose(R, T);
  cout << "EPnP took " << (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.  << " milliseconds." << endl;

  MatrixXf transform = MatrixXf::Identity(4,4);
  for(int i=0; i<3; ++i)
    for(int j=0; j<3; ++j)
      transform(i, j) = R[i][j];
  for(int i=0; i<3; ++i)
    transform(i, 3) = T[i];

  MatrixXf transform_final = transform.transpose();
  return transform_final;
}

int main(int argc, char** argv) {
  if(argc != 4) {
    cout << "Usage: " << argv[0] << " CAM_NUM VLF LLF" << endl;
    return 1;
  }

  g_calibration_camera = atoi(argv[1]);
  cout << "Calibrating camera " << g_calibration_camera << endl;
  
  // -- Connect to IPC and get the velo offset and cal filename.
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  if(ipc->ConnectLocked("hand_calibrator") < 0)
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
  char offset_str[500];
  char config_str[500];
  dgc_transform_print(g_velo_offset, offset_str);
  dgc_transform_print(g_velodyne_config->offset, config_str);
  cout << "g_velo_offset: " << endl << offset_str << endl;
  cout << "g_velodyne_config->offset: " << endl << config_str << endl;
  
  // -- Load logfiles.
  cout << "Loading Velodyne Log File " << argv[2] << endl;
  g_velodyne_file = dgc_velodyne_open_file(argv[2]);
  if(g_velodyne_file == NULL) {
    cout << "Could not open " << argv[2] << " for reading." << endl;
    return 1;
  }

  char index_filename[300];
  strcpy(index_filename, argv[2]);
  strcat(index_filename, ".index.gz");
  g_velodyne_index.load(index_filename);

  cout << "Loading Ladybug Log File " << argv[3] << endl;
  LadybugPlayback lbp;
  lbp.openLLF(argv[3]);

  // -- Load the ladybug model.
  cout << "Loading Ladybug intrinsics at " << g_intrinsics_path << endl;
  cout << "Base cam is " << g_base_cam << ", extrinsics at " << g_extrinsics_path << endl;
  LadybugModel lb3(g_extrinsics_path, g_intrinsics_path);
  g_initial_transform = lb3.extrinsics_.veloToCamera_[g_calibration_camera];
  cout << "Initial transform is " << endl << g_initial_transform << endl;

  // -- Set up OpenCV mouse callbacks.
  cvNamedWindow("Camera");
  cvSetMouseCallback("Camera", mouseHandler, g_camera_mouse);

  cvNamedWindow("Depth");
  cvSetMouseCallback("Depth", mouseHandler, g_depth_mouse);


  // -- Get corresponding points.
  vector<VectorXf> img_pts;
  vector<VectorXf> xyz_pts;
  runCalibrationInterface(lbp, lb3, &img_pts, &xyz_pts);


  // -- Compute the transform using EPnP.
  MatrixXf transform = solvePNP(lb3, img_pts, xyz_pts);
  cout << transform << endl;

  // -- Save.
  eigen_extensions::save(transform, LadybugExtrinsics::getTransformFilename(g_calibration_camera));
  cout << "Saved extrinsics for camera " << g_calibration_camera << " to " << LadybugExtrinsics::getTransformFilename(g_calibration_camera) << endl; 

  return 0;
}
