#include <dst/optical_flow_node.h>

using namespace std;

namespace dst
{
  
  OpticalFlowNode::OpticalFlowNode(pipeline2::Outlet<DepthProjector::Output>* index_otl) :
    ComputeNode(),
    optflow_otl_(this),
    index_otl_(index_otl),
    max_corners_(10000),
    quality_level_(0.005),
    min_distance_(1),
    edge_radius_(3),
    sigma_(0.043)
  {
    registerInput(index_otl_->getNode());
  }

  void OpticalFlowNode::safePointRound(const cv::Size& sz,
				       const cv::Point2f& fpt,
				       cv::Point2i* ipt)
  {
    ipt->x = min(sz.width - 1, max(0, (int)fpt.x));
    ipt->y = min(sz.height - 1, max(0, (int)fpt.y));
  }

  void OpticalFlowNode::_compute()
  {
    cv::Mat3b img = index_otl_->pull().current_img_;
    cv::Mat3b prev_img = index_otl_->pull().previous_img_;

    if(prev_img.rows == 0) {
      Output empty;
      empty.prev_points_ = NULL;
      empty.points_ = NULL;
      empty.status_ = NULL;
      empty.edge_scores_ = NULL;
      optflow_otl_.push(empty);
      return;
    }

    ROS_ASSERT(points_.empty());
    ROS_ASSERT(points_f_.empty());
    ROS_ASSERT(prev_points_.empty());
    ROS_ASSERT(prev_points_f_.empty());
    
    // -- Compute optical flow.
    cv::Mat1b gray_prev_img;
    cv::cvtColor(prev_img, gray_prev_img, CV_BGR2GRAY); // TODO: Maybe cache this in another node.
    cv::goodFeaturesToTrack(gray_prev_img, prev_points_f_, max_corners_, quality_level_, min_distance_);
    cv::calcOpticalFlowPyrLK(prev_img, img, prev_points_f_, points_f_, status_, error_);

    // -- Clean up the points; they might lie outside the image.
    prev_points_.resize(prev_points_f_.size());
    points_.resize(points_f_.size());
    ROS_ASSERT(prev_points_.size() == points_.size());
    for(size_t i = 0; i < prev_points_f_.size(); ++i) {
      safePointRound(prev_img.size(), prev_points_f_[i], &prev_points_[i]);
      safePointRound(img.size(), points_f_[i], &points_[i]);
    }
    
    // -- Compute edge scores.
    cv::Mat1i depth_index = index_otl_->pull().current_index_;
    KinectCloud::Ptr pcd = index_otl_->pull().current_pcd_;
    edge_scores_.resize(points_.size()); // Won't reallocate unless necessary.
    int neighborhood_size = pow(edge_radius_*2 + 1, 2);
    std::vector<double> distances;
    distances.reserve(neighborhood_size);
    for(size_t i = 0; i < points_.size(); ++i) {
      int x0 = min(depth_index.cols - 1, max(0, (int)points_[i].x));
      int y0 = min(depth_index.rows - 1, max(0, (int)points_[i].y));
      int idx0 = depth_index(y0, x0);
      if(idx0 == -1) {
	edge_scores_[i] = 0.0; // Trust places where we don't have depth.
	continue;
      }
      ROS_FATAL_STREAM_COND(idx0 < 0 || idx0 >= (int)pcd->size(), "idx0 is " << idx0 << " but pcd->size() is " << pcd->size() << "; point is (x, y) " << x0 << " " << y0);
      double ref = pcd->at(idx0).z;
      
      // -- Compute sorted z-distances of neighbors.
      distances.clear(); // won't cause reallocation.
      int min_x = max(0, x0 - edge_radius_);
      int max_x = min(depth_index.cols - 1, x0 + edge_radius_);
      int min_y = max(0, y0 - edge_radius_);
      int max_y = min(depth_index.rows - 1, y0 + edge_radius_);
      for(int x = min_x; x <= max_x; ++x) {
	for(int y = min_y; y <= max_y; ++y) {
	  int idx = depth_index(y, x);
	  if(idx != -1) {
	    double z = pcd->at(idx).z;
	    double pct = fabs(z - ref) / ref;
	    distances.push_back(pct);
	  }
	}
      }

      // TODO: This could be sorted on insertion.
      std::sort(distances.begin(), distances.end());
	
      // -- Find biggest change in z-distances.
      double max_change = 0;
      if(distances.size() > 1) { 
	for(size_t j = 1; j < distances.size(); ++j) {
	  double d = fabs(distances[j] - distances[j-1]);
	  if(d > max_change)
	    max_change = d;
	}
      }
      edge_scores_[i] = 1.0 - exp(-max_change / sigma_);
    }
      
    // -- Fill the outlet.
    output_.img_ = index_otl_->pull().current_img_;
    output_.prev_img_ = index_otl_->pull().previous_img_;
    output_.prev_points_ = &prev_points_;
    output_.points_ = &points_;
    output_.status_ = &status_;
    output_.edge_scores_ = &edge_scores_;
    optflow_otl_.push(output_);
  }

  void OpticalFlowNode::_display() const
  {
    cv::Mat3b prev_img = index_otl_->pull().previous_img_;
    if(prev_img.rows == 0)
      return;

    cv::Mat3b curr_img = index_otl_->pull().current_img_;
    KinectCloud::Ptr pcd = index_otl_->pull().current_pcd_;
    cv::Mat1i depth_index = index_otl_->pull().current_index_;
    cv::Mat1f zbuf(depth_index.size(), 0);
    float max = -std::numeric_limits<float>::max();
    for(int y = 0; y < zbuf.rows; ++y) {
      for(int x = 0; x < zbuf.cols; ++x) {
	int idx = depth_index(y, x);
	if(idx != -1) {
	  zbuf(y, x) = pcd->at(idx).z;
	  if(zbuf(y, x) > max)
	    max = zbuf(y, x);
	}
      }
    }
    zbuf /= max; // normalize so max element == 1.
    zbuf *= 255;
    cv::imwrite("debug/" + getRunName() + "-zbuffer.png", zbuf);
	    
    // -- Show flow vectors with edge scores.
    cv::Mat3b vis = curr_img.clone();
    for(size_t i = 0; i < prev_points_.size(); ++i) { 
      if(status_[i]) {
	//cv::circle(vis, prev_points_[i], 2, cv::Scalar(255, 0, 0));
	cv::line(vis, prev_points_[i], points_[i], cv::Scalar(0, 0, 0));
	cv::circle(vis, points_[i], 2, cv::Scalar(0, (1.0 - edge_scores_[i]) * 255, edge_scores_[i] * 255));
      }
    }
    cv::imwrite("debug/" + getRunName() + "-optical_flow.png", vis);
  }

  void OpticalFlowNode::_flush()
  {
    prev_points_.clear();
    points_.clear();
    prev_points_f_.clear();
    points_f_.clear();
    edge_scores_.clear();
    status_.clear();
    error_.clear();

    Output empty;
    empty.prev_points_ = NULL;
    empty.points_ = NULL;
    empty.status_ = NULL;
    empty.edge_scores_ = NULL;
    optflow_otl_.push(empty);
  }

  void OpticalFlowNode::_reset()
  {
  }    
  
  std::string OpticalFlowNode::_getName() const
  {
    ostringstream oss;
    oss << "OpticalFlowNode_max_corners:" << max_corners_ << "_quality_level:" << quality_level_ << "_min_distance:" << min_distance_;
    return oss.str();
  }

} // namespace dst
