#include "calibration_model.h"

using namespace std;
namespace bfs = boost::filesystem;
namespace bg = boost::gregorian;
namespace bpt = boost::posix_time;
using namespace Eigen;

CalibrationModel::CalibrationModel(char *cal_filename, dgc_transform_t velodyne_offset,
				   const string& intrinsics_path, const string& log_basename) :
  vlf_(cal_filename, log_basename + ".vlf", velodyne_offset),
  point_size_(2),
  video_frame_num_(0),
  fps_(29.97),
  log_basename_(log_basename),
  transform_path_(log_basename + ".avi.extrinsics.eig"),
  video_start_time_path_(log_basename + ".avi.timestamp"),
  video_start_time_offset_path_(log_basename + ".avi.sync_offset.eig"),
  special_regions_path_(log_basename + ".avi.special_regions")
{
  // -- Load the video.
  string video_path = log_basename_ + ".avi";
  video_capture_.open(video_path);
  if(!video_capture_.isOpened()) {
    cout << "Could not open video " << video_path << endl;
    assert(video_capture_.isOpened());
  }
  video_reference_start_ = video_capture_.get(CV_CAP_PROP_POS_MSEC);
  cout << "fps: " << video_capture_.get(CV_CAP_PROP_FPS) << endl;
  cout << "num frames: " << video_capture_.get(CV_CAP_PROP_FRAME_COUNT) << endl;
  
  // -- Print out some video properties for debugging.
  //cout << "position: " << video_capture_.get(CV_CAP_PROP_POS_MSEC) << endl;
  
  // -- Load the YAML intrinsics into the camera model.
  sensor_msgs::CameraInfo cam_info;
  string camera_name;
  bool success = camera_calibration_parsers::readCalibration(intrinsics_path, camera_name, cam_info);
  assert(success);
  pinhole_.fromCameraInfo(cam_info);

  // -- Load the image and rectify it.
  video_capture_ >> raw_;
  pinhole_.rectifyImage(raw_, rect_);

  // -- Set up calibration solver.
  epnp_ = new EPnPSolver(pinhole_.cx(), pinhole_.cy(), pinhole_.fx(), pinhole_.fy());
  cout << pinhole_.fx() << " " << pinhole_.fy() << endl;

  // -- Initialize video and velo timestamps.
  loadVideoTimestamp();
  velo_start_time_ = vlf_.getIndexEntry().pose[0].timestamp;
  cout << "velo timestamp: " << setprecision(16) << velo_start_time_ << endl;
  cout << "video timestamp: " << video_start_time_ << endl;
  cout << "diff: " << velo_start_time_ - video_start_time_ << endl;

  loadSyncOffset();
  cout << "Using sync offset of " << video_start_time_offset_ << endl;
  loadTransform();
  cout << "Using initial transform of " << endl << transform_ << endl;

  // -- Load the special regions for this video.
  if(bfs::exists(special_regions_path_))
    special_regions_.load(special_regions_path_);
    
  // -- Find the first matching pair.
  cout << "Searching for first matching video frame / spin." << endl;
  bool found_pair = getMatchingPair();
  if(!found_pair) {
    cout << "No matching video frame / spin." << endl;
    assert(found_pair);
  }
}

void CalibrationModel::loadVideoTimestamp()
{
  ifstream file;
  file.open(video_start_time_path_.c_str());
  if(!file.is_open()) {
    cout << "Expecting text file with YYYY-MM-DD HH:MM:SS to exist at " << video_start_time_path_ << endl;
    cout << "One way to create this file: " << endl;
    cout << "  exiftool MOV_NAME | grep '^Track Create Date' | awk '{print $5, $6}' | sed 's/:/-/' | sed 's/:/-/' > " + video_start_time_path_ << endl;
    cout << endl;
    cout << "Make sure you do this to the original MOV file, not some processed version of it." << endl;
    assert(file.is_open());
  }
  string timestamp;
  getline(file, timestamp);
  file.close();

  bpt::ptime time(bpt::time_from_string(timestamp));
  bpt::ptime start(bg::date(1970,1,1));
  bpt::time_duration dur = time - start;
  video_start_time_ = dur.total_seconds() + 8 * 60 * 60; // Offset due to time zones.  This may get screwed up by DST on the camera.
}


CalibrationModel::~CalibrationModel()
{
  if(epnp_)
    delete epnp_;
}

void CalibrationModel::save()
{
  cout << "Saving transform to " << transform_path_ << endl << transform_ << endl;
  eigen_extensions::save(transform_, transform_path_);

  cout << "Saving offset of " << video_start_time_offset_ << " to " << video_start_time_offset_path_ << endl;
  VectorXd tmp(1);
  tmp(0) = video_start_time_offset_;
  eigen_extensions::save(tmp, video_start_time_offset_path_);

  cout << "Saving special regions to " << special_regions_path_ << endl;
  special_regions_.save(special_regions_path_);
}

bool CalibrationModel::loadSyncOffset()
{
  if(!bfs::exists(video_start_time_offset_path_)) {
    if(getenv("FORCE_SYNCHRONOUS_START")) { 
      cout << video_start_time_offset_path_ << " not found.  Assuming logs start simultaneously because env var FORCE_SYNCHRONOUS_START is set." << endl;
      video_start_time_offset_ = 0;
    }
    else { 
      cout << video_start_time_offset_path_ << " not found.  Approximating using given timestamps." << endl;
      cout << "Set env var FORCE_SYNCHRONOUS_START to ignore timestamps." << endl;
      video_start_time_offset_ = velo_start_time_ - video_start_time_;
    }
    return false;
  }
  
  VectorXd tmp;
  eigen_extensions::load(video_start_time_offset_path_, &tmp);
  video_start_time_offset_ = tmp(0);
  return true;
}


bool CalibrationModel::loadTransform()
{
  if(!bfs::exists(transform_path_)) {
    transform_ = MatrixXd::Identity(4, 4);
    cout << "No transform given, using rough initial guess." << endl;
    setInitialTransform();
    return false;
  }

  eigen_extensions::load(transform_path_, &transform_);
  return true;
}

void CalibrationModel::getSpinInCameraCoords(MatrixXd* points, VectorXd* intensities)
{
  getSpinInSmoothCoords(points, intensities);
  (*points) = transform_ * (*points);
}

void CalibrationModel::getSpinInSmoothCoords(MatrixXd* points, VectorXd* intensities)
{
  assert(points);
  dgc::dgc_velodyne_spin &spin = vlf_.spin_;
  
  int num_valid = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
	continue;
      ++num_valid;
    }
  }
  
  *points = MatrixXd(4, num_valid);
  if(intensities)
    *intensities = VectorXd(num_valid);
  int idx = 0;
  for(int i = 0; i < spin.num_scans; ++i) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; ++j) {
      if(spin.scans[i].p[j].range < 0.01)
	continue;
      points->coeffRef(0, idx) = spin.scans[i].p[j].x * 0.01 + spin.scans[i].robot.x;
      points->coeffRef(1, idx) = spin.scans[i].p[j].y * 0.01 + spin.scans[i].robot.y;
      points->coeffRef(2, idx) = spin.scans[i].p[j].z * 0.01 + spin.scans[i].robot.z;
      points->coeffRef(3, idx) = 1.0;
      if(intensities)
	intensities->coeffRef(idx) = spin.scans[i].p[j].intensity;
      ++idx;
    }
  }
}

void CalibrationModel::setInitialTransform()
{
  transform_ = MatrixXd::Identity(4, 4);
  dgc::dgc_velodyne_index_pose &pose = vlf_.velodyne_index_.spin[0].pose[0];
  //cout << pose.smooth_x << " " << vlf_.spin_.scans[0].robot.x << endl;
  transform_(0, 3) = -pose.smooth_x;
  transform_(1, 3) = -pose.smooth_y;
  transform_(2, 3) = -pose.smooth_z;

  cout << "Initial translation: " << endl;
  cout << transform_ << endl;

  
  MatrixXd rot = MatrixXd::Zero(4, 4);
  rot(2, 0) = 1;
  rot(0, 1) = -1;
  rot(1, 2) = -1;
  rot(3, 3) = 1;

  transform_ = rot * transform_;
}

void CalibrationModel::drawOverlay2(cv::Mat img, bool draw_intensity, double increment)
{
  MatrixXd smooth;
  MatrixXd camera;
  VectorXd intensities;
  cv::Mat index = computeIndex(&smooth, &camera, &intensities);
      
  // -- Get robot center if we're coloring by distance to robot.
  Eigen::VectorXd center(4);
  if(!draw_intensity) { 
    dgc_pose_t robot = vlf_.spin_.scans[0].robot;
    center(0) = robot.x;
    center(1) = robot.y;
    center(2) = robot.z;
    center(3) = 0;
  }

  // -- Color all points.
  for(int u = 0; u < index.size().width; ++u) {
    for(int v = 0; v < index.size().height; ++v) {
      int idx = index.at<int>(v, u);
      if(idx == -1)
	continue;

      assert(smooth.col(idx).rows() == center.rows());
      assert(smooth.col(idx).cols() == center.cols());
      
      cv::Scalar color;
      if(draw_intensity)
	color = getIntensityColor(intensities(idx));
      else
	color = getDepthColor(smooth.col(idx) - center, increment);
      
      cv::circle(img, cv::Point2i(u, v), point_size_, color, -1);
    }
  }
}

void CalibrationModel::drawOverlay(cv::Mat img, bool draw_intensity)
{
  MatrixXd points;
  VectorXd intensities;
  getSpinInCameraCoords(&points, &intensities);

  for(int i = 0; i < points.cols(); ++i) { 
    cv::Point2d uv;
    bool in_img = projectPoint(points.col(i), img.size(), &uv);
    if(in_img) {
      if(draw_intensity)
	cv::circle(img, uv, point_size_, getIntensityColor(intensities(i)), -1);
      else
	cv::circle(img, uv, point_size_, getDepthColor(points.col(i)), -1);
    }
  }
}

bool CalibrationModel::projectPoint(const VectorXd& point, cv::Size size, cv::Point2d* uv)
{
  assert(point.rows() == 4);
  assert(fabs(point(3) - 1) < 1e-3);

  // -- Ignore points behind the camera.
  if(point(2) <= 0)
    return false;
  
  // -- Project and draw.
  cv::Point3d xyz(point(0), point(1), point(2));
  pinhole_.project3dToPixel(xyz, *uv);

  if(uv->x < 0 || uv->x >= size.width || uv->y < 0 || uv->y >= size.height)
    return false;
  else
    return true;
}

cv::Mat CalibrationModel::computeIndex(Eigen::MatrixXd* smooth, Eigen::MatrixXd* camera, VectorXd* intensities)
{
  getSpinInCameraCoords(camera); // Rough guess of camera coords.  For visualization only.
  getSpinInSmoothCoords(smooth, intensities);
  
  cv::Mat index(rect_.size(), CV_32SC1, -1);
  for(int i = 0; i < camera->cols(); ++i) { 
    cv::Point2d uv;
    bool in_img = projectPoint(camera->col(i), index.size(), &uv); // Sometimes uv is actually outside the image.
    if(in_img && ceil(uv.x) < index.size().width && ceil(uv.y) < index.size().height) {
      int old = index.at<int>(uv);
      if(old == -1)
	index.at<int>(uv) = i;
      else { // Closer points overwrite further points.
	if(camera->col(i).norm() < camera->col(old).norm())
	  index.at<int>(uv) = i;
      }
    }
  }

  return index;
}

cv::Scalar CalibrationModel::getIntensityColor(double intensity)
{
  assert(intensity >= 0);
  assert(intensity < 256);
  return cv::Scalar(intensity, 255.0 / 5.0 + 4.0 * intensity / 5.0, intensity); // White if bright, green if dark.
}

cv::Scalar CalibrationModel::getDepthColor(const VectorXd& point, double increment)
{
  assert(point.rows() == 4);
  assert(fabs(point(3) - 1) < 1e-3);

  double dist = point.segment(0, 3).norm();
  double thresh0 = 5;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;

  if(dist < thresh0) {
    return cv::Scalar(0, 0, 255, 0);
  }
  if(dist >= thresh0 && dist < thresh1) {
    int val = (dist - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Scalar(val, val, 255 - val, 0);
  }
  else if(dist >= thresh1 && dist < thresh2) {
    int val = (dist - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Scalar(255, 255 - val, 0, 0);
  }
  else if(dist >= thresh2 && dist < thresh3) {
    int val = (dist - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Scalar(255 - val, val, 0, 0);
  }

  return cv::Scalar(0, 255, 0, 0);
}

void CalibrationModel::rotate(double roll, double pitch, double yaw) {
  MatrixXd Rx = MatrixXd::Identity(4,4);
  Rx(1, 1) = cos(roll);
  Rx(2, 2) = cos(roll);
  Rx(1, 2) = -sin(roll);
  Rx(2, 1) = sin(roll);
  
  MatrixXd Ry = MatrixXd::Identity(4,4);
  Ry(0, 0) = cos(pitch);
  Ry(2, 2) = cos(pitch);
  Ry(2, 0) = -sin(pitch);
  Ry(0, 2) = sin(pitch);
  
  MatrixXd Rz = MatrixXd::Identity(4,4);
  Rz(0, 0) = cos(yaw);
  Rz(1, 1) = cos(yaw);
  Rz(0, 1) = -sin(yaw);
  Rz(1, 0) = sin(yaw);
  
  transform_ = Rz * Ry * Rx * transform_;
}

void CalibrationModel::translate(double x, double y, double z) {
  transform_(0, 3) += x;
  transform_(1, 3) += y;
  transform_(2, 3) += z;
}

void CalibrationModel::addCorrespondence(const Eigen::VectorXd& xyz, Eigen::VectorXd& uv)
{
  epnp_->addCorrespondence(xyz, uv);
}

void CalibrationModel::solveEPnP()
{
  cout << transform_ << endl;
  transform_ = epnp_->solve();
  cout << transform_ << endl;
}

size_t CalibrationModel::numCorrespondences() const
{
  return epnp_->numCorrespondences();
}
  
bool CalibrationModel::getMatchingPair()
{
  double tol = 0.51 / fps_; // 0.5 / fps_ is maximum theoretical discrepancy between (faster) video frame and (slower) velo spin.
  while(fabs(getCurrentVideoTime() - getCurrentVeloTime()) > tol) {
    if(getCurrentVeloTime() < getCurrentVideoTime()) { 
      bool valid = vlf_.increment(1);
      if(!valid)
	return false;
    }
    
    else {
      if(!video_capture_.grab())
	return false;
      ++video_frame_num_;

      // -- Problem with last frame?
      if(video_frame_num_ == video_capture_.get(CV_CAP_PROP_FRAME_COUNT) - 1)
	return false;
    }
    //cout << "spin " << vlf_.spin_num_ << ", img " << video_frame_num_ << ", spin - img = " << getCurrentVeloTime() - getCurrentVideoTime() << endl;
  }

  video_capture_.retrieve(raw_);
  pinhole_.rectifyImage(raw_, rect_);
  //cout << "position: " << video_capture_.get(CV_CAP_PROP_POS_MSEC) << endl;

  return true;
}

double CalibrationModel::getCurrentVeloTime()
{
  //return vlf_.getIndexEntry().pose[vlf_.getIndexEntry().num_poses - 1].timestamp;
  return vlf_.getIndexEntry().pose[0].timestamp;
}

double CalibrationModel::getCurrentVideoTime()
{
  //double oldts = video_start_time_ + video_start_time_offset_ + video_frame_num_ / fps_;
  double newts = video_start_time_ + video_start_time_offset_ + (video_capture_.get(CV_CAP_PROP_POS_MSEC) - video_reference_start_) / 1000.0;
  //cout << "Old - new: " << setprecision(16) << oldts - newts << endl;
  return newts;
  //return video_start_time_ + video_start_time_offset_ + video_frame_num_ / fps_;
}

void CalibrationModel::addVideoTimeOffset(double seconds)
{
  video_start_time_offset_ += seconds;
  getMatchingPair();
}

void CalibrationModel::recenter()
{
  MatrixXd points;
  VectorXd intensities;
  getSpinInSmoothCoords(&points, &intensities);

  MatrixXd rot = MatrixXd::Identity(4, 4);
  rot.block(0, 0, 3, 3) = transform_.block(0, 0, 3, 3);
  
  cout << rot * -points.rowwise().mean() << endl;
  cout << endl;
  cout << transform_.block(0, 3, 3, 1) << endl;
  cout << endl;

  transform_.block(0, 3, 3, 1) = -(rot * points.rowwise().mean()).block(0, 0, 3, 1);
}

bool CalibrationModel::advance(int num)
{
  bool success = true;
  success &= vlf_.increment(num);
  success &= getMatchingPair();
  return success;
}  

void CalibrationModel::writeDepthData(const std::string& path)
{
  MatrixXd smooth;
  MatrixXd camera;
  VectorXd intensities;
  cv::Mat index = computeIndex(&smooth, &camera, &intensities);

  // -- Get number of points.
  int num_pts = 0;
  for(int u = 0; u < index.size().width; ++u) {
    for(int v = 0; v < index.size().height; ++v) {
      int idx = index.at<int>(v, u);
      if(idx == -1)
	continue;

      ++num_pts;
    }
  }

  // -- Write the data to file.
  ofstream file(path.c_str());
  assert(file.is_open());
  file.write((char*)&num_pts, sizeof(int));
  
  for(int u = 0; u < index.size().width; ++u) {
    for(int v = 0; v < index.size().height; ++v) {
      int idx = index.at<int>(v, u);
      if(idx == -1)
	continue;

      float buf;
      buf = intensities(idx); file.write((char*)&buf, sizeof(float));
      buf = smooth(0, idx); file.write((char*)&buf, sizeof(float));
      buf = smooth(1, idx); file.write((char*)&buf, sizeof(float));
      buf = smooth(2, idx); file.write((char*)&buf, sizeof(float));
      buf = camera(0, idx); file.write((char*)&buf, sizeof(float));
      buf = camera(1, idx); file.write((char*)&buf, sizeof(float));
      buf = camera(2, idx); file.write((char*)&buf, sizeof(float));
      file.write((char*)&u, sizeof(int));
      file.write((char*)&v, sizeof(int));
    }
  }

  file.close();
}
