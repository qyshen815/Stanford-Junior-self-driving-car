#ifndef SCENE_H
#define SCENE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>

class TrackedObject;
class Segmentation;
class Scene;
class Sequence;

class TrackedObject
{
public:
  int id_;
  std::vector<int> indices_;
  //Scene* scene_;
  //Segmentation* segmentation_;
  
  TrackedObject();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

class Segmentation
{
public:
  std::vector<TrackedObject> tracked_objects_;
  //Scene* scene_;
  
  Segmentation();
  Segmentation(const std::string& path);
  void save(const std::string& path) const;
  void serialize(std::ostream& out) const;
  
private:
  void deserialize(std::istream& in);
};

class Scene
{
public:
  cv::Mat img_;
  //! The points in the camera coordinate system.  npts x 3.
  Eigen::MatrixXf cloud_cam_;
  //! The points in smooth coords.  npts x 3.
  Eigen::MatrixXf cloud_smooth_;
  Eigen::VectorXf intensities_;
  //! npts x 2.  The points in the camera-centered depth image.  There can be parallax effects here,
  //! i.e. many points with different depths mixed together.  This can screw up attempts to draw sharp
  //! boundaries.
  Eigen::MatrixXi cam_points_;
  //! npts x 2.  The points in a velodyne-centered depth image.  There should be no parallax here.
  //Eigen::MatrixXf velo_depth_image_points_;
  Segmentation* segmentation_;

  //! path + ".png" and path + ".txt" should be the image and point cloud data for this Scene.
  //! segmentation_ will be filled if path + "_segmentation.txt" exists; otherwise it will be NULL.
  Scene(const std::string& path);
  ~Scene();
  cv::Mat getDepthOverlay() const;
  //! Returns an image that shows segmented object id.  If id == -1, then all objects are shown.
  cv::Mat getSegmentationOverlay(int id = -1) const;
  void saveSegmentation() const;
  void addTrackedObject(const TrackedObject& to);
  //! Adds a new (empty) tracked object if id doesn't exist.
  TrackedObject& getTrackedObject(int id);
  
private:
  std::string path_;
  
  void loadPointCloud(const std::string& path);
  void loadPointCloudBinary(const std::string& path);
  void loadSegmentation(const std::string& path);
};

class Sequence
{
public:
  //! Assumes that all files in the root dir are part of the sequence.
  Sequence(const std::string& root);
  size_t size() const;
  boost::shared_ptr<Scene> getScene(size_t idx) const;
  void saveSegmentations() const;
  
private:
  std::string root_;
  std::vector<std::string> names_;
  std::vector< boost::shared_ptr<Scene> > scenes_;

  //! User must delete the Scene.
  Scene* loadScene(size_t idx) const;
};


#endif // SCENE_H
