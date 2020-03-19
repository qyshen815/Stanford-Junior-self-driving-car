#include <dst/label_flow_npg.h>

using namespace Eigen;

namespace dst
{
  
  LabelFlowNPG::LabelFlowNPG(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
			     pipeline2::Outlet<cv::Mat1b>* prev_seg_otl) :
    NodePotentialGenerator(),
    optflow_otl_(optflow_otl),
    prev_seg_otl_(prev_seg_otl)
  {
    registerInput(optflow_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());
  }
  
  void LabelFlowNPG::_compute()
  {
    // -- Get inputs.
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0) {
      source_otl_.push(NULL);
      sink_otl_.push(NULL);
      return;
    }
    
    if(source_potentials_.rows() != prev_seg.rows ||
       source_potentials_.cols() != prev_seg.cols) {
      source_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
      sink_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
    }

    OpticalFlowNode::Output flow = optflow_otl_->pull();
    if(!flow.prev_points_) {
      source_otl_.push(&source_potentials_); // zero
      sink_otl_.push(&sink_potentials_);
      return;      
    }
    
    ROS_ASSERT(flow.points_);
    ROS_ASSERT(flow.status_);
    ROS_ASSERT(flow.edge_scores_);
    const std::vector<cv::Point2i>& prev_points = *flow.prev_points_;
    const std::vector<cv::Point2i>& points = *flow.points_;
    const std::vector<uchar>& status = *flow.status_;
    const std::vector<double>& edge_scores = *flow.edge_scores_;

    ROS_ASSERT(status.size() == points.size());
    ROS_ASSERT(status.size() == prev_points.size());
    
    // -- Compute potentials.
    for(size_t i = 0; i < status.size(); ++i) {
      if(status[i] == 0)
	continue;

      int label = prev_seg(prev_points[i].y, prev_points[i].x);
      ROS_ASSERT(edge_scores[i] >= 0 && edge_scores[i] <= 1.0);
      
      if(label == 255) { 
	source_potentials_(points[i].y, points[i].x) = 1.0 - edge_scores[i];
	sink_potentials_(points[i].y, points[i].x) = 0.0;
      }
      else if(label == 0) { 
    	source_potentials_(points[i].y, points[i].x) = 0.0;
	sink_potentials_(points[i].y, points[i].x) = 1.0 - edge_scores[i];
      }
    }

    // -- Fill outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void LabelFlowNPG::_display() const
  {
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0)
      return;

    displayNodePotentials(optflow_otl_->pull().img_);
  }

  void LabelFlowNPG::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  std::string LabelFlowNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "LabelFlowNPG";
    return oss.str();
  }
  
} // namespace dst
