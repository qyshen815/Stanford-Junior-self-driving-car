#include <dst/scene_alignment_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  SceneAlignmentNPG::SceneAlignmentNPG(pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl,
				       pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
				       pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
				       pipeline2::Outlet<DepthProjector::Output>* index_otl,
				       int num_neighbors,
				       double sigma) :
    NodePotentialGenerator(),
    transformed_otl_(transformed_otl),
    kdtree_otl_(kdtree_otl),
    prev_seg_otl_(prev_seg_otl),
    index_otl_(index_otl),
    num_neighbors_(num_neighbors),
    sigma_(sigma)
  {
    registerInput(transformed_otl_->getNode());
    registerInput(kdtree_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());
    registerInput(index_otl_->getNode());
  }
  
  void SceneAlignmentNPG::_compute()
  {
    // -- Don't do anything if this is the first frame.
    if(!kdtree_otl_->pull().previous_kdtree_) {
      source_otl_.push(NULL);
      sink_otl_.push(NULL);
      return;
    }

    // -- Reallocate potentials if necessary.
    cv::Mat1i prev_index = index_otl_->pull().previous_index_;
    ROS_ASSERT(prev_index.rows > 0);
    if(source_potentials_.rows() != prev_index.rows ||
       source_potentials_.cols() != prev_index.cols) {
      source_potentials_ = MatrixXd::Zero(prev_index.rows, prev_index.cols);
      sink_potentials_ = MatrixXd::Zero(prev_index.rows, prev_index.cols);
    }
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);
    
    // -- Compute the labels of previous points.
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    KinectCloud::Ptr prev_pcd = index_otl_->pull().previous_pcd_;
    ROS_ASSERT(labels_.empty());
    ROS_ASSERT(prev_pcd);
    ROS_ASSERT(prev_seg.rows > 0);

    labels_.resize(prev_pcd->size(), 127);
    for(int y = 0; y < prev_index.rows; ++y) { 
      for(int x = 0; x < prev_index.cols; ++x) {
	int idx = prev_index(y, x);
	if(idx == -1)
	  continue;
	ROS_ASSERT(idx >= 0 && (size_t)idx < prev_pcd->size());
	labels_[idx] = prev_seg(y, x);
      }
    }

    // -- Compute the image points for the current pointcloud.
    KinectCloud::Ptr transformed_pcd = transformed_otl_->pull();
    KinectCloud::Ptr curr_pcd = index_otl_->pull().current_pcd_;
    cv::Mat1i curr_index = index_otl_->pull().current_index_;
    ROS_ASSERT(transformed_pcd->size() == curr_pcd->size());
    ROS_ASSERT(imgpts_.empty());
    imgpts_.resize(curr_pcd->size(), cv::Point2i(-1, -1));
    for(int y = 0; y < curr_index.rows; ++y) {
      for(int x = 0; x < curr_index.cols; ++x) {
	int idx = curr_index(y, x);
	if(idx == -1)
	  continue;
	ROS_ASSERT(idx >= 0 && (size_t)idx < curr_pcd->size());
	imgpts_[idx] = cv::Point2i(x, y);
      }
    }
    
    // HighResTimer hrt("OrganizedNeighborSearch init");
    // hrt.start();
    // typedef pcl::OrganizedNeighborSearch<pcl::PointXYZRGB> Search;
    // typedef boost::shared_ptr<Search> SearchPtr;
    // SearchPtr ons(new Search());
    // ons->setInputCloud(prev_pcd);
    // hrt.stop();
    // cout << hrt.reportMilliseconds() << endl;
    
    // -- Assign node potentials based on nearby points in the previous cloud.
    KdTreeNode::KdTreeType& prev_kdtree = *kdtree_otl_->pull().previous_kdtree_;
    ROS_ASSERT(transformed_pcd);
    vector<int> indices(num_neighbors_);
    vector<float> distances(num_neighbors_);
    for(size_t i = 0; i < transformed_pcd->size(); ++i) {
      if(imgpts_[i].x == -1 && imgpts_[i].y == -1)
	continue;

      int num_found = prev_kdtree.nearestKSearch(transformed_pcd->at(i), num_neighbors_, indices, distances); // TODO: not const.  Is this thread safe?
      
      //prev_kdtree.radiusSearch(transformed_pcd->at(i), radius_, indices, distances, 5);
      if(num_found == 0)
	continue;
      
      //cout << indices[0] << " " << distances[0] << " " << labels_[indices[0]] << " " << imgpts_[indices[0]] << endl;
      int prev_idx = indices[0];
      if(labels_[prev_idx] == 255) {
	ROS_ASSERT(imgpts_[i].x >= 0 && imgpts_[i].y >= 0);
	source_potentials_(imgpts_[i].y, imgpts_[i].x) = exp(-distances[0] / sigma_);
	sink_potentials_(imgpts_[i].y, imgpts_[i].x) = 0.0;
      }
      else if(labels_[prev_idx] == 0) {
	ROS_ASSERT(imgpts_[i].x >= 0 && imgpts_[i].y >= 0);
	source_potentials_(imgpts_[i].y, imgpts_[i].x) = 0.0;
	sink_potentials_(imgpts_[i].y, imgpts_[i].x) = exp(-distances[0] / sigma_);
      }
    }

    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void SceneAlignmentNPG::_display() const
  {
    if(!kdtree_otl_->pull().previous_kdtree_)
      return;
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);
    displayNodePotentials(index_otl_->pull().current_img_);
  }

  void SceneAlignmentNPG::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();
    labels_.clear();
    imgpts_.clear();
  }

  std::string SceneAlignmentNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "SceneAlignmentNPG";
    return oss.str();
  }

  
  
} // namespace dst
