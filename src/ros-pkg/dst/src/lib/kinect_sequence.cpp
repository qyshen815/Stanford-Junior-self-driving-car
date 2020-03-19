#include <dst/kinect_sequence.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace dst
{

  KinectSequence::KinectSequence() :
    Serializable()
  {
  }

  void KinectSequence::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save instead.");
  }

  void KinectSequence::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load instead.");
  }

  void KinectSequence::save(const std::string& dir) const
  {
    ROS_ASSERT(!bfs::exists(dir));
    bfs::create_directory(dir);

    for(size_t i = 0; i < images_.size(); ++i) {
      ostringstream oss;
      oss << "img" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), images_[i]);
    }

    for(size_t i = 0; i < pointclouds_.size(); ++i) {
      ostringstream oss;
      oss << "pcd" << setw(4) << setfill('0') << i << ".pcd";
      pcl::io::savePCDFileBinary(dir + "/" + oss.str(), *pointclouds_[i]);
    }
    
    ROS_ASSERT(seed_images_.empty() || seed_images_.size() == images_.size());
    for(size_t i = 0; i < seed_images_.size(); ++i) {
      ostringstream oss;
      oss << "seed" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), seed_images_[i]);
    }

    ROS_ASSERT(segmentations_.empty() || segmentations_.size() == images_.size());
    for(size_t i = 0; i < segmentations_.size(); ++i) {
      ostringstream oss;
      oss << "segmentation" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), segmentations_[i]);
    }

    ostringstream oss;
    oss << dir << "/cam_info";
    cv::FileStorage fs(oss.str(), cv::FileStorage::WRITE);
    fs << "CameraInfo";
    camera_info_.write(fs);
    fs.release();
  }

  void KinectSequence::load(const std::string& dir)
  {
    // -- Get filenames.
    vector<string> img_names;
    vector<string> seed_names;
    vector<string> segmentation_names;
    vector<string> pcd_names;
    
    bfs::recursive_directory_iterator it(dir), eod;
    BOOST_FOREACH(bfs::path const & p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().substr(0, 4).compare("seed") == 0 &&  bfs::extension(p).compare(".png") == 0)
	seed_names.push_back(p.string());
      else if(p.leaf().substr(0, 4).compare("segm") == 0 &&  bfs::extension(p).compare(".png") == 0)
	segmentation_names.push_back(p.string());
      else if(p.leaf().substr(0, 3).compare("img") == 0 &&  bfs::extension(p).compare(".png") == 0)
	img_names.push_back(p.string());
      else if(bfs::extension(p).compare(".pcd") == 0)
	pcd_names.push_back(p.string());
    }
    ROS_ASSERT(img_names.size() == pcd_names.size());

    // -- Sort all filenames.
    sort(img_names.begin(), img_names.end());
    sort(pcd_names.begin(), pcd_names.end());
    sort(segmentation_names.begin(), segmentation_names.end());
    sort(seed_names.begin(), seed_names.end());

    // -- Load images and pointclouds.
    images_.resize(img_names.size());
    pointclouds_.resize(pcd_names.size());
    for(size_t i = 0; i < img_names.size(); ++i) {
      images_[i] = cv::imread(img_names[i], 1);
      pointclouds_[i] = KinectCloud::Ptr(new KinectCloud());
      pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_names[i], *pointclouds_[i]);
    }

    // -- Load segmentations.
    if(segmentation_names.size() == img_names.size()) { 
      segmentations_.resize(segmentation_names.size());
      for(size_t i = 0; i < segmentation_names.size(); ++i) { 
	//cout << "Loading " << segmentation_names[i] << endl;
	segmentations_[i] = cv::imread(segmentation_names[i], 0);
      }
    }
    else if(segmentation_names.empty()) {
      segmentations_.resize(img_names.size());
      for(size_t i = 0; i < segmentations_.size(); ++i)
	segmentations_[i] = cv::Mat1b(images_[0].size(), 127);
    }
    else
      ROS_FATAL("A KinectSequence must have either all segmentations or no segmentations.");

    // -- Load seed labels.
    if(seed_names.size() == img_names.size()) { 
      seed_images_.resize(seed_names.size());
      for(size_t i = 0; i < seed_names.size(); ++i)
	seed_images_[i] = cv::imread(seed_names[i], 0);
    }
    else if(seed_names.empty()) {
      seed_images_.resize(img_names.size());
      for(size_t i = 0; i < seed_images_.size(); ++i)
	seed_images_[i] = cv::Mat1b(images_[0].size(), 127);
    }
    else
      ROS_FATAL("A KinectSequence must have either all seed images or no seed images.");

    // -- Load camera info.
    cv::FileStorage fs(dir + "/cam_info", cv::FileStorage::READ);
    camera_info_.read(fs["CameraInfo"]);
  }
  
  KinectSequence::KinectSequence(const KinectSequence& seq)
  {
    ROS_ASSERT(seq.images_.size() == seq.pointclouds_.size());
    images_.resize(seq.images_.size());
    pointclouds_.resize(seq.images_.size());

    for(size_t i = 0; i < images_.size(); ++i) {
      images_[i] = seq.images_[i].clone();
      pointclouds_[i] = seq.pointclouds_[i]->makeShared();
    }
  }
  
  KinectSequence& KinectSequence::operator=(const KinectSequence& seq)
  {
    if(&seq == this)
      return *this;

    ROS_ASSERT(seq.images_.size() == seq.pointclouds_.size());
    images_.resize(seq.images_.size());
    pointclouds_.resize(seq.images_.size());
    
    for(size_t i = 0; i < images_.size(); ++i) {
      images_[i] = seq.images_[i].clone();
      pointclouds_[i] = seq.pointclouds_[i]->makeShared();
    }

    return *this;
  }  
  
} // namespace dst
