#include "ladybug_model.h"

using namespace std;
using namespace dgc;
using namespace Eigen;

/************************************************************
 * LadybugModel
 ************************************************************/

LadybugModel::LadybugModel(std::string extrinsics, std::string intrinsics) :
  intrinsics_(intrinsics),
  extrinsics_(extrinsics, &intrinsics_)
{
}

LadybugModel::LadybugModel(int base_camera, std::string extrinsics, std::string intrinsics) :
  intrinsics_(intrinsics),
  extrinsics_(extrinsics, base_camera, &intrinsics_)
{
}  

void LadybugModel::veloToCamera(int camera_num, double* x, double* y, double* z) const {
  MatrixXf pt_velo(1, 4);
  pt_velo(0, 0) = *x;
  pt_velo(0, 1) = *y;
  pt_velo(0, 2) = *z;
  pt_velo(0, 3) = 1;
  
  MatrixXf pt_cam = extrinsics_.veloToCamera(camera_num, pt_velo);
  if(fabs(pt_cam(0, 3) - 1) > 1e-6)
    cout << "WARNING: LadybugModel::veloToCamera is seeing un-normalized homogeneous coords.  4th coord is " << setprecision(16) << pt_cam(0, 3) << endl;
  //assert(fabs(pt_cam(0, 3) - 1) < 1e-6);

  *x = pt_cam(0, 0);
  *y = pt_cam(0, 1);
  *z = pt_cam(0, 2);
}
 
bool LadybugModel::projectVeloToRectifiedImage(int camera_num,
						double x, double y, double z,
						int* u, int* v) const
{
  veloToCamera(camera_num, &x, &y, &z);
  intrinsics_.cameras_[camera_num].xyzToUVRect(x, y, z, u, v);
  if(*u == -1 || *v == -1)
    return false;
  else
    return true;
}

bool LadybugModel::projectVeloToRectifiedImage(
            double x, double y, double z,
            int* u, int* v, unsigned int* camera_num) const
{
  for(unsigned int test_cam = 0; test_cam < intrinsics_.cameras_.size(); 
      test_cam++)
  {
    if(projectVeloToRectifiedImage(test_cam, x, y, z, u, v)) {
      *camera_num = test_cam;
      return true;
    }
  }
  return false;
}

bool LadybugModel::projectVeloToWarpedImage(int camera_num, MatrixXd velo_pts, MatrixXi* img_pts) const {
  
  MatrixXi tmp = MatrixXi::Zero(velo_pts.rows(), 2);
  vector<bool> in_img(velo_pts.rows());
  int num_in_img = 0;
  for(int i = 0; i < velo_pts.rows(); ++i) {
    if(velo_pts.cols() == 4) {
      if(fabs(velo_pts(i, 3) - 1) >= 1e-3)
	cout << "WARNING: homogeneous coords with w != 1?  w = " << setprecision(16) << velo_pts(i, 3) << endl;
      assert(fabs(velo_pts(i, 3) - 1) < 1e-3);
    }
    
    in_img[i] = projectVeloToWarpedImage(camera_num, velo_pts(i, 0), velo_pts(i, 1), velo_pts(i, 2),
					 &tmp(i, 0), &tmp(i, 1));
    if(in_img[i])
      ++num_in_img;
  }

  if(num_in_img == 0)
    return false;
  
  *img_pts = MatrixXi::Zero(num_in_img, 2);
  int idx = 0;
  for(int i = 0; i < velo_pts.rows(); ++i) {
    if(in_img[i]) {
      img_pts->row(idx) = tmp.row(i);
      ++idx;
    }
  }

  return true;
}

bool LadybugModel::projectVeloToWarpedImage(int camera_num,
					     double x, double y, double z,
					     int* u, int* v) const
{
  veloToCamera(camera_num, &x, &y, &z);
  intrinsics_.cameras_[camera_num].xyzToUVWarp(x, y, z, u, v);
  if(*u == -1 || *v == -1)
    return false;
  else
    return true;
}

bool LadybugModel::projectVeloToWarpedImage(
            double x, double y, double z,
            int* u, int* v, unsigned int* camera_num) const
{
  for(unsigned int test_cam = 0; test_cam < intrinsics_.cameras_.size();
      test_cam++)
  {
    if(projectVeloToWarpedImage(test_cam, x, y, z, u, v)) {
      *camera_num = test_cam;
      return true;
    }
  }
  return false;
}

/************************************************************
 * LadybugIntrinsics
 ************************************************************/

LadybugIntrinsics::LadybugIntrinsics(string filename) {
  ifstream file;
  file.open(filename.c_str());
  if(file.fail())
    cerr << "Failed to open file " << filename << endl;

  cameras_.reserve(6);
  for(size_t i=0; i<6; ++i)
    cameras_.push_back(LadybugCameraIntrinsics(file));
}

/************************************************************
 * LadybugCameraIntrinsics
 ************************************************************/

template<class T>
void LadybugCameraIntrinsics::naturalToOriginalWarped(T* u, T* v) const {
  T tmp = *u;
  *u = *v;
  *v = warp_cols_ - 1 - tmp; // warp_cols_ is the number of columns in the natural image.
}

template<class T>
void LadybugCameraIntrinsics::originalToNaturalWarped(T* u, T* v) const {
  T tmp = *v;
  *v = *u;
  *u = warp_cols_ - 1 - tmp; // warp_cols_ is the number of rows in the original image.
}

template<class T>
void LadybugCameraIntrinsics::naturalToOriginalRectified(T* u, T* v) const {
  T tmp = *u;
  *u = *v;
  *v = rect_cols_ - 1 - tmp; // rect_cols_ is the number of columns in the natural image.
}

template<class T>
void LadybugCameraIntrinsics::originalToNaturalRectified(T* u, T* v) const {
  T tmp = *v;
  *v = *u;
  *u = rect_cols_ - 1 - tmp; // rect_cols_ is the number of rows in the original image.
}

string LadybugCameraIntrinsics::status() const {
  ostringstream oss;
  oss << "Camera " << camera_id_ << endl;
  oss << "warped rows x cols: " << warp_rows_ << " " << warp_cols_ << endl;
  oss << "rectified rows x cols: " << rect_rows_ << " " << rect_cols_ << endl;
  oss << "Focal length: " << focal_length_ << endl;
  oss << "Rectified image center (x, y): " << rect_center_u_ << " " << rect_center_v_ << endl;
  oss << "rot x: " << rot_x_ << endl;
  oss << "rot y: " << rot_y_ << endl;
  oss << "rot z: " << rot_z_ << endl;
  oss << "trans x: " << trans_x_ << endl;
  oss << "trans y: " << trans_y_ << endl;
  oss << "trans z: " << trans_z_ << endl;
  oss << "rectified image row, col for warped image row 413 and col 130: " << warp_to_rect_[413][130].first << " " << warp_to_rect_[413][130].second << endl;
  oss << "warped image row, col for rectified image row 413 and col 130: " << rect_to_warp_[413][130].first << " " << rect_to_warp_[413][130].second << endl;
  oss << "camera_to_ladybug: " << endl << camera_to_ladybug_ << endl;
  oss << "ladybug_to_camera: " << endl << ladybug_to_camera_ << endl;
  return oss.str();
}  

/**
 * Takes a point in the 3D space defined by (0,0,0) as the focal point,
 * positive x aligned with increasing values on the shorter sensor axis (x = u = right)
 * positive y aligned with increasing values on the longer sensor axis (y = v = down)
 * positive z extending out from the lens
 * This coordinate system is generally determined by the extrinsic calibration between the velodyne and
 * the ladybug.
 *
 * (u, v) is  (0, 0) at the upper left hand corner of the natural image.
 * u and v are set to -1 if the point is not in the FOV of the sensor
 */
void LadybugCameraIntrinsics::xyzToUVWarp(double x, double y, double z, int* u, int* v) const {

  int uTemp, vTemp;

  // First get point in rect image
  xyzToUVRect(x,y,z,&uTemp,&vTemp);  

  if(uTemp == -1) {
    *u = -1;
    *v = -1;
    return;
  }

  // Now convert to warped image
  *u = rect_to_warp_[vTemp][uTemp].second;
  *v = rect_to_warp_[vTemp][uTemp].first;

  // Make sure point is in the warped image
  if( *u < 0 || *u > warp_cols_ - 1 || *v < 0 || *v > warp_rows_ -1 ) {
    *u = -1;
    *v = -1;
  }

  return;
}

void LadybugCameraIntrinsics::xyzToUVRect(double x, double y, double z, int* u, int* v, double focal_length_modifier) const {

  if(z <= 0) {
    *u = -1;
    *v = -1;
    return;
  }

  *u = rint((focal_length_ + focal_length_modifier) * x / z + rect_center_u_); // This projection is into the natural rectified image.
  *v = rint((focal_length_ + focal_length_modifier) * y / z + rect_center_v_);

  // Make sure point is in rectified image
  if( *u < 0 || *u > rect_cols_ - 1 || *v < 0 || *v > rect_rows_ - 1 ) {
    *u = -1;
    *v = -1;
  }

  return;
}

//! This function based on PGR's documentation for ladybugGetCameraUnitExtrinsics.
MatrixXf constructTransform(double rot_x, double rot_y, double rot_z,
			    double trans_x, double trans_y, double trans_z)
{
  double crx = cos(rot_x);
  double cry = cos(rot_y);
  double crz = cos(rot_z);
  double srx = sin(rot_x);
  double sry = sin(rot_y);
  double srz = sin(rot_z);


  MatrixXf transform = MatrixXf::Identity(4, 4);

  transform(0, 0) = crz * cry;
  transform(0, 1) = crz * sry * srx - srz * crx;
  transform(0, 2) = crz * sry * crx + srz * srx;
  transform(0, 3) = trans_x;

  transform(1, 0) = srz * cry;
  transform(1, 1) = srz * sry * srx + crz * crx;
  transform(1, 2) = srz * sry * crx - crz * srx;
  transform(1, 3) = trans_y;

  transform(2, 0) = -sry;
  transform(2, 1) = cry * srx;
  transform(2, 2) = cry * crx;
  transform(2, 3) = trans_z;

  transform = transform.transpose(); // We should really switch to transform * pt...
  return transform;
}

LadybugCameraIntrinsics::LadybugCameraIntrinsics(istream& in) {
  // -- Load intrinsics.
  in.read((char*)&camera_id_, sizeof(int));
  in.read((char*)&rect_center_u_, sizeof(double)); // Data is stored as u, v for *rotated* image.
  in.read((char*)&rect_center_v_, sizeof(double)); // We want to work with the natural image.  See below.
  in.read((char*)&focal_length_, sizeof(double));

  in.read((char*)&rot_x_, sizeof(double));
  in.read((char*)&rot_y_, sizeof(double));
  in.read((char*)&rot_z_, sizeof(double));
  in.read((char*)&trans_x_, sizeof(double));
  in.read((char*)&trans_y_, sizeof(double));
  in.read((char*)&trans_z_, sizeof(double));

  in.read((char*)&warp_rows_, sizeof(int)); // Data is stored as cols, rows for the *rotated* image. 
  in.read((char*)&warp_cols_, sizeof(int)); // We want to work with the natural image.
  in.read((char*)&rect_rows_, sizeof(int));
  in.read((char*)&rect_cols_, sizeof(int));

  // Modify rect_center to be for the natural image.
  originalToNaturalRectified(&rect_center_u_, &rect_center_v_);
  

  // -- Load the warped to rectified remapping.
  int num_bytes = sizeof(float) * warp_rows_ * warp_cols_ * 2;
  float* warp_data = (float*) malloc(num_bytes);
  in.read((char*)warp_data, num_bytes); // warp_data is in the original (rotated) coordinate system.

  warp_to_rect_.resize(warp_rows_);
  for(int i=0; i<warp_rows_; ++i) {
    warp_to_rect_[i].resize(warp_cols_);
    for(int j=0; j<warp_cols_; ++j) {
      int u = j;
      int v = i;
      naturalToOriginalWarped(&u, &v);
      warp_to_rect_[i][j].first = warp_data[(v * warp_rows_ + u) * 2];
      warp_to_rect_[i][j].second = warp_data[(v * warp_rows_ + u) * 2 + 1];
      originalToNaturalRectified(&warp_to_rect_[i][j].second, &warp_to_rect_[i][j].first);
    }
  }

  free(warp_data);

  // -- Load the rectified to warped remapping.
  num_bytes = sizeof(float) * rect_rows_ * rect_cols_ * 2;
  float* rect_data = (float*) malloc(num_bytes);
  in.read((char*)rect_data, num_bytes);

  rect_to_warp_.resize(rect_rows_);
  for(int i=0; i<rect_rows_; ++i) {
    rect_to_warp_[i].resize(rect_cols_);
    for(int j=0; j<rect_cols_; ++j) {
      int u = j;
      int v = i;
      naturalToOriginalRectified(&u, &v);
      rect_to_warp_[i][j].first = rect_data[(v * rect_rows_ + u) * 2];
      rect_to_warp_[i][j].second = rect_data[(v * rect_rows_ + u) * 2 + 1];
      originalToNaturalWarped(&rect_to_warp_[i][j].second, &rect_to_warp_[i][j].first);
    }
  }

  free(rect_data);

  // -- Build transformations.
  MatrixXf natural_to_original = MatrixXf::Zero(4, 4);
  natural_to_original(0, 1) = -1;
  natural_to_original(1, 0) = 1;
  natural_to_original(2, 2) = 1;
  natural_to_original(3, 3) = 1;
  
  camera_to_ladybug_ = constructTransform(rot_x_, rot_y_, rot_z_, trans_x_, trans_y_, trans_z_);
  //camera_to_ladybug_ = natural_to_original * camera_to_ladybug_;
  ladybug_to_camera_ = camera_to_ladybug_.inverse();
}

IplImage* LadybugCameraIntrinsics::rectify(IplImage* warp) const {
  assert(warp->nChannels == 3);
  assert(warp->height == warp_rows_);
  assert(warp->width == warp_cols_);
  assert(warp->depth == IPL_DEPTH_8U);

  IplImage* rect = cvCreateImage(cvSize(rect_cols_, rect_rows_), warp->depth, warp->nChannels);
  cvZero(rect);

  for(int v=0; v<rect->height; ++v) {
    uchar* ptr = (uchar*)(rect->imageData + v * rect->widthStep);
    for(int u=0; u<rect->width; ++u, ptr+=3) { 
      int v_warp = rect_to_warp_[v][u].first;
      int u_warp = rect_to_warp_[v][u].second;
      if(v_warp >= 0 && v_warp < warp->height &&
	 u_warp >= 0 && u_warp < warp->width) { 

        ptr[0] = ((uchar*)(warp->imageData + v_warp * warp->widthStep))[u_warp*3 + 0];
        ptr[1] = ((uchar*)(warp->imageData + v_warp * warp->widthStep))[u_warp*3 + 1];
        ptr[2] = ((uchar*)(warp->imageData + v_warp * warp->widthStep))[u_warp*3 + 2];
      }
    }
  }

  return rect;
}

/************************************************************
 * LadybugExtrinsics
 ************************************************************/

LadybugExtrinsics::LadybugExtrinsics(const string& filedir, LadybugIntrinsics* intrinsics) :
  veloToCamera_(vector<MatrixXf>(intrinsics->cameras_.size())),
  intrinsics_(intrinsics),
  file_name_base_("velo_to_ladybug_cam")
{
  // -- Load the transformation from the velodyne frame to camera base_camera frame.
  MatrixXf transform;

  for( unsigned int cam = 0; cam < intrinsics->cameras_.size(); cam++) {
    ostringstream filename;
    filename << filedir << "/" << file_name_base_ << cam << ".eig";
    if(!boost::filesystem::exists(filename.str())) {
      cerr << "********** WARNING: " << filename.str() << " does not exist.  Using rough initial guess for velo-ladybug extrinsic calibration." << endl;
      transform = getInitialTransform(cam);
    }
    else { 
      eigen_extensions::load(filename.str(), &transform);
    }
    assert(transform.rows() == 4 && transform.cols() == 4);
    veloToCamera_[cam] = transform;
  
  }    
}

LadybugExtrinsics::LadybugExtrinsics(const string& filedir, int base_camera, LadybugIntrinsics* intrinsics) :
  veloToCamera_(vector<MatrixXf>(intrinsics->cameras_.size())),
  intrinsics_(intrinsics),
  base_camera_(base_camera),
  file_name_base_("velo_to_ladybug_cam")
{
  // -- Load the transformation from the velodyne frame to camera base_camera frame.
  MatrixXf transform;
  ostringstream filename;
  filename << filedir << "/" << file_name_base_ << base_camera_ << ".eig";
  if(!boost::filesystem::exists(filename.str())) {
    cerr << "********** WARNING: " << filename.str() << " does not exist.  Using rough initial guess for velo-ladybug extrinsic calibration." << endl;
    transform = getInitialTransform(base_camera);
  }
  else { 
    eigen_extensions::load(filename.str(), &transform);
  }
  assert(transform.rows() == 4 && transform.cols() == 4);
  veloToCamera_[base_camera_] = transform;
  
  // -- Compute the transformation from the velodyne frame to all other camera frames.
  for(size_t i = 0; i < veloToCamera_.size(); ++i) { 
    veloToCamera_[i] = veloToCamera_[base_camera_] * intrinsics_->cameras_[base_camera_].camera_to_ladybug_ * intrinsics_->cameras_[i].ladybug_to_camera_;
  }
}


MatrixXf LadybugExtrinsics::veloToCamera(int cam_num, MatrixXf cloud) const {
  assert(cloud.cols() == 4);
  VectorXf sum = cloud.colwise().sum();
  assert(sum(3) == cloud.rows());

  // -- Transform points from velodyne coordinates to camera cam_num coordinates.
  cloud = cloud * veloToCamera_[cam_num];
  return cloud;
}

std::string LadybugExtrinsics::getTransformFilename(int camera_num) {
  ostringstream filename;
  filename << "velo_to_ladybug_cam" << camera_num << ".eig";
  return filename.str();
}

MatrixXf LadybugExtrinsics::getInitialTransform(int cam_num) {
  assert(cam_num >= 0 && cam_num <= 5);

  if(cam_num == 5) {
    return MatrixXf::Identity(4, 4); // TODO: Fix this if we actually care.
  }
  
  VectorXf cal = VectorXf::Zero(6);
  //float rot_x = -4.7; // In radians.
  //float rot_y = 3.7;
  float rot_x = 0; // In radians.
  float rot_y = 0;
  float rot_z = 0;
  float tx = 0.0; // In meters.
  float ty = 0.0;
  float tz = 0.0;
  cal(0) = tx;
  cal(1) = ty;
  cal(2) = tz;
  cal(3) = rot_x;
  cal(4) = rot_y;
  cal(5) = rot_z;

  // Setup rotations.
  double degrees_per_cam = 2.0 * M_PI / 5.0;
  double offset = cam_num * degrees_per_cam - M_PI / 5.0 - degrees_per_cam; // cam1 is forward-looking left cam, cam2 is forward-looking right cam.
  MatrixXf preRz = MatrixXf::Identity(4, 4);
  preRz(0,0) = cos(offset);
  preRz(1,1) = cos(offset);
  preRz(1,0) = -sin(offset);
  preRz(0,1) = sin(offset);

  // Velodyne coordinate system has x forward, y left, and z up.
  // Camera coordinate system has x right, y down, z forward.
  MatrixXf rot = MatrixXf::Zero(4, 4);
  rot(0, 2) = 1;
  rot(1, 0) = -1;
  rot(2, 1) = -1;
  rot(3, 3) = 1;
  
  return preRz * rot;
}

/************************************************************
 * Helper functions
 ************************************************************/

IplImage* dgcToIpl(const vlr::dgc_image_t& dgc) {
  assert(dgc.nchannels == 3);
  IplImage* img = cvCreateImage(cvSize(dgc.width, dgc.height), IPL_DEPTH_8U, 3);
  cvSetZero(img);
  unsigned char* dgc_ptr = dgc.pix;
  for(int y=dgc.height-1; y>=0; --y) {
    char* ipl_ptr = (char*)(img->imageData + y * img->widthStep);
    for(int x=0; x<dgc.width; ++x) {
      ipl_ptr[0] = dgc_ptr[2];
      ipl_ptr[1] = dgc_ptr[1];
      ipl_ptr[2] = dgc_ptr[0];
      ipl_ptr += 3;
      dgc_ptr += 3;
    }
  }

  return img;
}


void spinDgcToEigen(const dgc_velodyne_spin& spin, MatrixXf* cloud, VectorXf* intensity) {
  // -- Count how many points there are.
  int num_valid = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;
      ++num_valid;
    }
  }
  *cloud = MatrixXf(num_valid, 4);
  *intensity = VectorXf(num_valid);

  // -- Put them into a matrix.
  int idx = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;

      intensity->coeffRef(idx) = spin.scans[i].p[j].intensity;

      double x = spin.scans[i].p[j].x / 100.; //Convert from cm to meters.
      double y = spin.scans[i].p[j].y / 100.;
      double z = spin.scans[i].p[j].z / 100.;
      cloud->coeffRef(idx, 0) = x;
      cloud->coeffRef(idx, 1) = y;
      cloud->coeffRef(idx, 2) = z;
      cloud->coeffRef(idx, 3) = 1;

      ++idx;
    }
  }
  assert(idx == num_valid);
}

void getSpinInVeloCoords(const dgc_velodyne_spin& spin, double synchronizer_angle, dgc_transform_t velo_offset, MatrixXf* cloud, VectorXf* intensity) {
  
  // -- Count how many points there are.
  int num_valid = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;
      ++num_valid;
    }
  }
  *cloud = MatrixXf(num_valid, 4);
  *intensity = VectorXf(num_valid);

  // -- Set the reference scan to be the one measured as close as possible to
  //    the time at which the ladybug triggered.
  int reference_scan = synchronizer_angle / (2.0 * M_PI) * spin.num_scans;
  dgc_pose_t ref = spin.scans[reference_scan].robot;
  
  int idx = 0;
  for(int i = 0; i < spin.num_scans; ++i) {

    // -- Construct transform from this scan to the reference scan's velodyne coords.
    dgc_pose_t robot = spin.scans[i].robot;
    dgc_transform_t transform;
    dgc_transform_identity(transform);

    // This scan to reference scan.
    dgc_transform_translate(transform, robot.x - ref.x, robot.y - ref.y, robot.z - ref.z);

    // Reference scan to reference velodyne.
    dgc_transform_t transform2, inv, final;
    dgc_transform_rpy(transform2, velo_offset, ref.roll, ref.pitch, ref.yaw);
    dgc_transform_inverse(transform2, inv);
    dgc_transform_left_multiply_nc(final, transform, inv);
    
    // -- Apply the transform to all points in the scan.
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;

      intensity->coeffRef(idx) = spin.scans[i].p[j].intensity;

      double x = spin.scans[i].p[j].x / 100.; //Convert from cm to meters.
      double y = spin.scans[i].p[j].y / 100.;
      double z = spin.scans[i].p[j].z / 100.;
      dgc_transform_point(&x, &y, &z, final);
      cloud->coeffRef(idx, 0) = x;
      cloud->coeffRef(idx, 1) = y;
      cloud->coeffRef(idx, 2) = z;
      cloud->coeffRef(idx, 3) = 1;

      ++idx;
    }
  }
  assert(idx == num_valid);
}

void getSpinInSmoothCoords(const dgc_velodyne_spin& spin, MatrixXf* cloud, VectorXf* intensity) {
  
  // -- Count how many points there are.
  int num_valid = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;
      ++num_valid;
    }
  }
  *cloud = MatrixXf(num_valid, 4);
  *intensity = VectorXf(num_valid);
  
  int idx = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
        continue;
      
      intensity->coeffRef(idx) = spin.scans[i].p[j].intensity;
      cloud->coeffRef(idx, 0) = spin.scans[i].p[j].x / 100. + spin.scans[i].robot.x; // Convert from cm to meters and put in smooth coords.
      cloud->coeffRef(idx, 1) = spin.scans[i].p[j].y / 100. + spin.scans[i].robot.y;
      cloud->coeffRef(idx, 2) = spin.scans[i].p[j].z / 100. + spin.scans[i].robot.z;
      cloud->coeffRef(idx, 3) = 1;
      ++idx;
    }
  }   
  assert(idx == num_valid);
}

CvScalar intensityMap(int intensity) {
  assert(intensity >= 0);
  assert(intensity < 256);
  return cvScalar(intensity, 255.0 / 5.0 + 4.0 * intensity / 5.0, intensity); // White if bright, green if dark.
}

CvScalar depthMap(VectorXf point) {
  assert(point.rows() == 4);
  point /= point(3); // Normalize homogeneous coordinates.
  double dist = point.segment(0, 3).norm();
  double increment = 15;
  double thresh0 = 5;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;

  if(dist < thresh0) {
    return cvScalar(0, 0, 255);
  }
  if(dist >= thresh0 && dist < thresh1) {
    int val = (dist - thresh0) / (thresh1 - thresh0) * 255.;
    return cvScalar(val, val, 255 - val);
  }
  else if(dist >= thresh1 && dist < thresh2) {
    int val = (dist - thresh1) / (thresh2 - thresh1) * 255.;
    return cvScalar(255, 255 - val, 0);
  }
  else if(dist >= thresh2 && dist < thresh3) {
    int val = (dist - thresh2) / (thresh3 - thresh2) * 255.;
    return cvScalar(255 - val, val, 0);
  }

  return cvScalar(0, 255, 0);
}

CvScalar colorMap(VectorXf point, int intensity, bool show_intensity) {
  if(show_intensity)
    return intensityMap(intensity);
  else
    return depthMap(point);
}

int findFirstMatchingPair(LadybugPlayback& lbp, const dgc_velodyne_index& index) { 
  // TODO: This function should make use of the sync offset (though it does not really matter).

  int spin_num = 0;
  lbp.readNextPacket();
  double llf_timestamp = lbp.getTimestamp();
  double vlf_timestamp = index.spin[spin_num].pose[0].timestamp;
  double vlf_timestamp_prev = index.spin[spin_num].pose[0].timestamp;

  while(llf_timestamp < vlf_timestamp) {
    if(!lbp.readNextPacket()) {
      return -1;
    }
    llf_timestamp = lbp.getTimestamp();
  }

  while(true) {     
    vlf_timestamp_prev = vlf_timestamp;
    vlf_timestamp = index.spin[spin_num].pose[0].timestamp;
    if(vlf_timestamp > llf_timestamp && vlf_timestamp_prev < llf_timestamp) {
      lbp.readTimestampPacket(vlf_timestamp);
      break;
    }

    ++spin_num;
    if(spin_num >= index.num_spins) {
      return -1;
    }
  }
  
  return spin_num;
}
