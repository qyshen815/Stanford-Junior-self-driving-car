#include "external_camera_model.h"

using namespace std;
using namespace Eigen;
namespace bg = boost::gregorian;
namespace bpt = boost::posix_time;
namespace bfs = boost::filesystem;

ExternalCameraModel::ExternalCameraModel(string intrinsics_path, string video_path) :
  video_frame_num_(0),
  fps_(29.97),
  video_start_time_offset_(0),
  video_start_time_(0)
{
  string avi_path = video_path;
  string avi_timestamp_path = video_path + ".timestamp";
  string avi_sync_offset_path = video_path + ".sync_offset.eig";
  string avi_extrinsics_path = video_path + ".extrinsics.eig";
  
  // -- Load the YAML intrinsics into the camera model.
  sensor_msgs::CameraInfo cam_info;
  string camera_name;
  bool success = camera_calibration_parsers::readCalibration(intrinsics_path, camera_name, cam_info);
  assert(success);
  pinhole_.fromCameraInfo(cam_info);

  // -- Load the video.
  video_capture_.open(avi_path);
  if(!video_capture_.isOpened()) {
    cout << "Could not open video " << avi_path << endl;
    assert(video_capture_.isOpened());
  }
  video_reference_start_ = video_capture_.get(CV_CAP_PROP_POS_MSEC);
  
  // -- Load timestamp.
  loadVideoTimestamp(avi_timestamp_path);
  
  // -- Load sync offset.
  assert(bfs::exists(avi_sync_offset_path));
  VectorXd tmp;
  eigen_extensions::load(avi_sync_offset_path, &tmp);
  video_start_time_offset_ = tmp(0);
  cout << "Using sync offset of " << video_start_time_offset_ << endl;

  // -- Load extrinsics.
  assert(bfs::exists(avi_extrinsics_path));
  eigen_extensions::load(avi_extrinsics_path, &transform_);
  cout << "Using transform of " << endl << transform_ << endl;

  // -- Load special regions.
  string special_regions_path = video_path + ".special_regions";
  special_regions_.load(special_regions_path);
  
  // -- Load the first image and rectify it.
  advance(1);
}


void ExternalCameraModel::loadVideoTimestamp(string path)
{
  ifstream file;
  file.open(path.c_str());
  if(!file.is_open()) {
    cout << "Expecting text file with YYYY-MM-DD HH:MM:SS to exist at " << path << endl;
    cout << "One way to create this file: " << endl;
    cout << "  exiftool MOV_NAME | grep '^Track Create Date' | awk '{print $4, $5}' | sed 's/:/-/' | sed 's/:/-/' > " + path << endl;
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

bool ExternalCameraModel::advance(int num)
{
  bool success = true;
  for(int i = 0; i < num; ++i) { 
    if(!video_capture_.grab()) {
      success = false;
      break;
    }
    ++video_frame_num_;
  }
  video_capture_.retrieve(raw_);
  pinhole_.rectifyImage(raw_, rect_);
  return success;
}


double ExternalCameraModel::getTimestamp()
{
  //return video_start_time_ + video_start_time_offset_ + video_frame_num_ / fps_;
  return video_start_time_ + video_start_time_offset_ + (video_capture_.get(CV_CAP_PROP_POS_MSEC) - video_reference_start_) / 1000.0;
}

double ExternalCameraModel::getRelativeTimestamp()
{
  return (video_capture_.get(CV_CAP_PROP_POS_MSEC) - video_reference_start_) / 1000.0;
}

bool ExternalCameraModel::xyzCameraToUVRect(const VectorXd& point, cv::Point2d* uv) const
{
  assert(point.rows() == 4);
  assert(fabs(point(3) - 1) < 1e-3);

  // -- Ignore points behind the camera.
  if(point(2) <= 0)
    return false;
  
  // -- Project and draw.
  cv::Point3d xyz(point(0), point(1), point(2));
  pinhole_.project3dToPixel(xyz, *uv);

  if(uv->x < 0 || uv->x >= rect_.size().width || uv->y < 0 || uv->y >= rect_.size().height)
    return false;
  else
    return true;
}

double ExternalCameraModel::getFps() const
{
  return fps_;
}

double ExternalCameraModel::overlap(const Label& a, const Label& b) const
{
  int min_x = max(a.minx(), b.minx());
  int max_x = min(a.maxx(), b.maxx());
  int min_y = max(a.miny(), b.miny());
  int max_y = min(a.maxy(), b.maxy());

  if(min_x >= max_x || min_y >= max_y)
    return 0.0;
  
  double overlap_area = (max_x - min_x) * (max_y - min_y);
  double area_b = (b.width_ * b.height_);

  assert(area_b >= 0.0 && overlap_area >= 0.0);
  assert(overlap_area <= area_b);
  
  return overlap_area / area_b;
}

bool ExternalCameraModel::excluded(const Label& label) const
{
  double pct = 0.0;
  for(size_t i = 0; i < special_regions_.labels_.size(); ++i) {
    if(special_regions_.labels_[i].class_name_.compare("exclude") != 0)
      continue;

    pct += overlap(special_regions_.labels_[i], label);
    if(pct > 0.75)
      return true;
  }

  return false;
}
