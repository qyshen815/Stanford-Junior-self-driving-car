#include <segmentation_and_tracking/scene.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;

Scene::Scene(const std::string& path) :
  segmentation_(NULL),
  path_(path)
{
  img_ = cv::imread(path_ + ".jpg");
  assert(img_.size().width != 0 && img_.size().height != 0); // Way to fail silently opencv.
  loadPointCloudBinary(path_ + ".dat");

  string segmentation_path = path_ + "_segmentation.dat";
  if(bfs::exists(segmentation_path))
    segmentation_ = new Segmentation(segmentation_path);
  else
    assert(!segmentation_);
}

Scene::~Scene()
{
  if(segmentation_)
    delete segmentation_;
}

void Scene::saveSegmentation() const
{
  assert(segmentation_);
  string segmentation_path = path_ + "_segmentation.dat";
  segmentation_->save(segmentation_path);
}

void Scene::addTrackedObject(const TrackedObject& to)
{
  if(!segmentation_)
    segmentation_ = new Segmentation();

  segmentation_->tracked_objects_.push_back(to);
}

TrackedObject& Scene::getTrackedObject(int id)
{
  // -- If there's no segmentation, create one.
  if(!segmentation_)
    segmentation_ = new Segmentation();

  // -- Find tracked object with the desired id.
  for(size_t i = 0; i < segmentation_->tracked_objects_.size(); ++i) { 
    if(id == segmentation_->tracked_objects_[i].id_)
      return segmentation_->tracked_objects_[i];
  }

  // -- If id didn't exist, create it.
  TrackedObject to;
  to.id_ = id;
  segmentation_->tracked_objects_.push_back(to);
  return segmentation_->tracked_objects_.back();
}

void Scene::loadPointCloudBinary(const string& path)
{
  ifstream file(path.c_str());
  int num_points;
  file.read((char*)&num_points, sizeof(int));
  
  cloud_cam_ = MatrixXf::Zero(num_points, 3);
  cloud_smooth_ = MatrixXf::Zero(num_points, 3);
  intensities_ = VectorXf::Zero(num_points);
  cam_points_ = MatrixXi::Zero(num_points, 2);
  for(int i = 0; i < num_points; ++i) {
    file.read((char*)&cloud_smooth_.coeffRef(i, 0), sizeof(float));
    file.read((char*)&cloud_smooth_.coeffRef(i, 1), sizeof(float));
    file.read((char*)&cloud_smooth_.coeffRef(i, 2), sizeof(float));
    file.read((char*)&cloud_cam_.coeffRef(i, 0), sizeof(float));
    file.read((char*)&cloud_cam_.coeffRef(i, 1), sizeof(float));
    file.read((char*)&cloud_cam_.coeffRef(i, 2), sizeof(float));
    file.read((char*)&intensities_.coeffRef(i), sizeof(float));
    file.read((char*)&cam_points_.coeffRef(i, 0), sizeof(int));
    file.read((char*)&cam_points_.coeffRef(i, 1), sizeof(int));
  }
	    
}

void Scene::loadPointCloud(const string& path)
{

  int num_points = 0;
  {
    ifstream file(path.c_str());
    string line;
    while(!file.eof()) {
      getline(file, line);
      ++num_points;
    }
    file.close();
  }
  
  cloud_cam_ = MatrixXf::Zero(num_points, 3);
  cloud_smooth_ = MatrixXf::Zero(num_points, 3);
  intensities_ = VectorXf::Zero(num_points);
  cam_points_ = MatrixXi::Zero(num_points, 2);
  
  ifstream file(path.c_str());
  for(int i = 0; i < num_points; ++i) { 
    file >> cloud_smooth_(i, 0);
    file >> cloud_smooth_(i, 1);
    file >> cloud_smooth_(i, 2);
    file >> cloud_cam_(i, 0);
    file >> cloud_cam_(i, 1);
    file >> cloud_cam_(i, 2);
    file >> intensities_(i);
    file >> cam_points_(i, 0);
    file >> cam_points_(i, 1);
  }
  file.close();
}

cv::Scalar getDepthColorBW(double depth)
{
  double thresh0 = 5;
  double thresh1 = 30;
  double val;
  if(depth < thresh0)
    val = 1.0;
  else if(depth < thresh1) 
    val = (thresh1 - (depth - thresh0)) / thresh1;
  else
    val = 0.0;

  val *= 255;
  return cv::Scalar(val, val, val);
}

cv::Scalar getDepthColor(double depth)
{
  double increment = 15;
  double thresh0 = 5;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;

  if(depth < thresh0) {
    return cv::Scalar(0, 0, 255);
  }
  if(depth >= thresh0 && depth < thresh1) {
    int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Scalar(val, val, 255 - val);
  }
  else if(depth >= thresh1 && depth < thresh2) {
    int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Scalar(255, 255 - val, 0);
  }
  else if(depth >= thresh2 && depth < thresh3) {
    int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Scalar(255 - val, val, 0);
  }

  return cv::Scalar(0, 255, 0);
}


cv::Mat Scene::getDepthOverlay() const
{
  int radius = 1;
  cv::Mat overlay = img_.clone();
  for(int i = 0; i < cam_points_.rows(); ++i) {
    cv::Scalar color = getDepthColor(cloud_cam_.row(i).norm());
    cv::circle(overlay, cv::Point(cam_points_(i, 0), cam_points_(i, 1)), radius, color);
  }
  return overlay;
}

cv::Mat Scene::getSegmentationOverlay(int id) const
{
  int radius = 1;
  cv::Mat overlay = img_.clone();

  // -- Draw all points in B&W.
  for(int i = 0; i < cam_points_.rows(); ++i) {
    cv::Scalar color = getDepthColorBW(cloud_cam_.row(i).norm());
    cv::circle(overlay, cv::Point(cam_points_(i, 0), cam_points_(i, 1)), radius, color);
  }

  // -- Draw segmented objects in color.
  if(segmentation_) {
    for(size_t i = 0; i < segmentation_->tracked_objects_.size(); ++i) {
      if(id != -1 && segmentation_->tracked_objects_[i].id_ != id)
	continue;
      vector<int>& indices = segmentation_->tracked_objects_[i].indices_;
      for(size_t j = 0; j < indices.size(); ++j) {
	cv::Scalar color = getDepthColor(cloud_cam_.row(indices[j]).norm());
	cv::Point point(cam_points_(indices[j], 0), cam_points_(indices[j], 1));
	cv::circle(overlay, point, radius, color);
      }
    }
  }

  return overlay;
}

