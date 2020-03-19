#include <dst/bilateral_npg.h>

using namespace Eigen;

namespace dst
{

  BilateralNPG::BilateralNPG(pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
			     pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl,
			     pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
			     pipeline2::Outlet<DepthProjector::Output>* index_otl,
			     double sigma_dist,
			     double sigma_color,
    			     double sigma_img_dist) :
    NodePotentialGenerator(),
    kdtree_otl_(kdtree_otl),
    transformed_otl_(transformed_otl),
    prev_seg_otl_(prev_seg_otl),
    index_otl_(index_otl),
    sigma_dist_(sigma_dist),
    sigma_color_(sigma_color),
    sigma_img_dist_(sigma_img_dist),
    img_radius_(10)
  {
    registerInput(kdtree_otl_->getNode());
    registerInput(transformed_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());
    registerInput(index_otl_->getNode());
  }

  inline
  double BilateralNPG::computeTerm(const pcl::PointXYZRGB& curr_pt,
				   const pcl::PointXYZRGB& prev_pt) const
  {
    double dr = curr_pt.r - prev_pt.r;
    double dg = curr_pt.g - prev_pt.g;
    double db = curr_pt.b - prev_pt.b;
    double color_term = sqrt(dr*dr + dg*dg + db*db) / sigma_color_;
    double distance_term = pcl::euclideanDistance(curr_pt, prev_pt) / sigma_dist_;

    // cout << "Distance: " << pcl::euclideanDistance(curr_pt, prev_pt) << endl;
    // cout << "exp(-dist / sigma): " << exp(-distance_term) << endl; 
    
//    cout << "color: " << color_term << ", dist: " << distance_term << ", val: " << exp(-color_term - distance_term) << endl;
    
    return exp(-color_term - distance_term);
  }

  inline
  double BilateralNPG::computeTerm(const cv::Point2i& curr_pt,
				   const cv::Vec3b& curr_color,
				   const cv::Point2i& prev_pt,
				   const cv::Vec3b& prev_color) const
  {
    double db = curr_color[0] - prev_color[0];
    double dg = curr_color[1] - prev_color[1];
    double dr = curr_color[2] - prev_color[2];
    double color_term = sqrt(dr*dr + dg*dg + db*db) / sigma_color_;
    double dx = curr_pt.x - prev_pt.x;
    double dy = curr_pt.y - prev_pt.y;
    double distance_term = sqrt(dx*dx + dy*dy) / sigma_img_dist_;
    return exp(-color_term - distance_term);
  }

  void BilateralNPG::getNeighborhood(cv::Mat3b img,
				     const cv::Point2i& pt,
				     double radius,
				     std::vector<cv::Point2i>* neighborhood) const
  {
    int d = ceil(radius);
    for(int y = pt.y - d; y <= pt.y + d; ++y) {
      if(y < 0)
	continue;
      if(y >= img.rows)
	continue;
      
      for(int x = pt.x - d; x <= pt.x + d; ++x) {
	if(x < 0)
	  continue;
	if(x >= img.cols)
	  continue;

	double dx = x - pt.x;
	double dy = y - pt.y;
	double dist = dx*dx + dy*dy;
	if(dist > radius*radius)
	  continue;
	
	neighborhood->push_back(cv::Point2i(x, y));
      }
    }
  }
				     
  
  inline
  double BilateralNPG::computeBilateralNoDepth(const cv::Point2i& curr_pt,
					       cv::Mat3b curr_img,
					       cv::Mat3b prev_img,
					       cv::Mat1b prev_seg)
  {
    ImageRegionIterator it(curr_img.size(), curr_pt, img_radius_);
    double numerator = 0;
    double denominator = 0;
    for(; !it.done(); ++it) {
      double z = computeTerm(curr_pt, curr_img(curr_pt),
			     *it, prev_img(*it));
      double label = 0.0;
      if(prev_seg(*it) == 0)
	label = -1.0;
      else if(prev_seg(*it) == 255)
	label = 1.0;
      
      numerator += z * label;
      denominator += z;
    }

    ROS_ASSERT(denominator != 0);
    return numerator / denominator;
  }
  
  inline
  double BilateralNPG::computeBilateral(const pcl::PointXYZRGB& curr_pt,
					const KdTreeNode::KdTreeType& prev_kdtree,
					const std::vector<cv::Point2i>& prev_rindex,
					cv::Mat1b prev_seg,
					const KinectCloud& prev_pcd)
  {
    double radius = 0.075; // TODO: Switch BilateralNPG over to using the depth index rather than kdtree.
    indices_.clear();
    distances_.clear();
    prev_kdtree.radiusSearch(curr_pt, radius, indices_, distances_);
    // cout << "------------------------------------------------------------" << endl;
    // cout << "Got " << distances_.size() << " neighbors." << endl;
      
    double numerator = 0.0;
    double denominator = 0.0;
    for(size_t i = 0; i < indices_.size(); ++i) {
      cv::Point2i prev_img_pt = prev_rindex[indices_[i]];
      if(prev_img_pt.x == -1 && prev_img_pt.y == -1)
	continue;

      // if(rand() % 10 != 0)
      // 	continue;
      
      double label = 0.0;
      if(prev_seg(prev_img_pt) == 255)
	label = 1.0;
      else if(prev_seg(prev_img_pt) == 0)
	label = -1.0;
	    
      const pcl::PointXYZRGB& prev_pt = prev_pcd[indices_[i]];
      double z = computeTerm(curr_pt, prev_pt);
      denominator += z;
      numerator += z * label;
    }

    // -- Don't normalize; instead, pass the output through a sigmoid.
    //    If all weights are tiny, then BilateralNPG won't have a strong preference.
    return 2.0 * (1.0 / (1.0 + exp(-numerator))) - 1.0;
    
    // -- Normalize.
    // double potential;
    // if(denominator == 0)
    //   potential = 0;
    // else
    //   potential = numerator / denominator;

    // return potential;
  }
  
  void BilateralNPG::_compute()
  {
    // -- Don't compute if this is the first frame.
    if(!kdtree_otl_->pull().previous_kdtree_)
      return;
    
    // -- Pull in data from parent nodes.
    KinectCloud& prev_pcd = *kdtree_otl_->pull().previous_pcd_;
    KdTreeNode::KdTreeType& prev_kdtree = *kdtree_otl_->pull().previous_kdtree_;
    KinectCloud& transformed = *transformed_otl_->pull();
    //std::vector<cv::Point2i>& curr_rindex = *index_otl_->pull().current_rindex_;
    std::vector<cv::Point2i>& prev_rindex = *index_otl_->pull().previous_rindex_;
    cv::Mat1i curr_index = index_otl_->pull().current_index_;
    cv::Mat1i prev_index = index_otl_->pull().previous_index_;
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    cv::Mat3b curr_img = index_otl_->pull().current_img_;
    cv::Mat3b prev_img = index_otl_->pull().previous_img_;

    // -- Reallocate potentials if necessary.
    ROS_ASSERT(prev_seg.rows > 0);
    if(source_potentials_.rows() != prev_seg.rows ||
       source_potentials_.cols() != prev_seg.cols) {
      source_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
      sink_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
    }
    ROS_ASSERT(source_potentials_.rows() > 0 && source_potentials_.cols() > 0);
    ROS_ASSERT(sink_potentials_.rows() > 0 && sink_potentials_.cols() > 0);
    
    // -- Find all nearest neighbors, compute filter.
    for(int y = 0; y < curr_index.rows; ++y) {
      for(int x = 0; x < curr_index.cols; ++x) {
	int idx = curr_index(y, x);
	double potential = 0;
	if(idx != -1) {
	  //cout << idx << " " << y << " " << x << " " << curr_index.cols * y + x << endl; // TODO: depth index vs ordered pointcloud.
	  potential = computeBilateral(transformed[idx], prev_kdtree,
				       prev_rindex, prev_seg, prev_pcd);
	}
	else {
	  // cv::Point2i curr_pt(x, y);
	  // potential = computeBilateralNoDepth(curr_pt, curr_img,
	  // 				      prev_img, prev_seg);
	}
	if(potential > 0)
	  source_potentials_(y, x) = potential;
	else
	  sink_potentials_(y, x) = -potential;
      }
    }

    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void BilateralNPG::_display() const
  {
    if(!kdtree_otl_->pull().previous_kdtree_)
      return;
    displayNodePotentials(index_otl_->pull().current_img_);
  }

  void BilateralNPG::_flush()
  {
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
    source_potentials_.setZero();
    sink_potentials_.setZero();
  }

  void BilateralNPG::_reset()
  {

  }

  std::string BilateralNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "BilateralNPG";
    return oss.str();
  }

} // namespace dst
