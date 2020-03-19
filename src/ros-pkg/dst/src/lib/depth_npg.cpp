#include <dst/depth_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  DepthNPG::DepthNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
		     pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
		     float sigma, size_t buffer_size) :
    NodePotentialGenerator(),
    index_otl_(index_otl),
    prev_seg_otl_(prev_seg_otl),
    sigma_(sigma),
    buffer_size_(buffer_size)
  {
    registerInput(index_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());
  }

  void DepthNPG::_compute()
  {
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0)
      return;

    // -- Reallocate potentials if necessary.
    ROS_ASSERT(prev_seg.rows > 0);
    if(source_potentials_.rows() != prev_seg.rows ||
       source_potentials_.cols() != prev_seg.cols) {
      source_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
      sink_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
    }

    // -- Find max and min z of the foreground in the last frame.
    cv::Mat1i prev_index = index_otl_->pull().previous_index_;
    KinectCloud& prev_pcd = *index_otl_->pull().previous_pcd_;
    float max = -std::numeric_limits<float>::max();
    float min = std::numeric_limits<float>::max();
    for(int y = 0; y < prev_index.rows; ++y) {
      for(int x = 0; x < prev_index.cols; ++x) {
	if(prev_seg(y, x) != 255 || prev_index(y, x) == -1)
	  continue;
	float z = prev_pcd[prev_index(y, x)].z;
	if(z > max)
	  max = z;
	if(z < min)
	  min = z;
      }
    }

    // -- Update the buffer.
    max_buffer_.push_back(max);
    min_buffer_.push_back(min);
    if(max_buffer_.size() == buffer_size_)
      max_buffer_.pop_front();
    if(min_buffer_.size() == buffer_size_)
      min_buffer_.pop_front();

    // -- Use the largest max and min from the buffer.
    deque<float>::iterator it;
    for(it = max_buffer_.begin(); it != max_buffer_.end(); ++it)
      if(*it > max)
	max = *it;
    for(it = min_buffer_.begin(); it != min_buffer_.end(); ++it)
      if(*it < min)
	min = *it;

    // -- Assign node potentials in current frame.
    cv::Mat1i index = index_otl_->pull().current_index_;
    KinectCloud& pcd = *index_otl_->pull().current_pcd_;
    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x) {
	if(index(y, x) == -1)
	  continue;
	float z = pcd[index(y, x)].z;
	if(z < min || z > max) { 
	  float delta = fmin(fabs(z - min), fabs(z - max));
	  sink_potentials_(y, x) = 1.0 - exp(-delta / sigma_);
	}
      }
    }

    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void DepthNPG::_display() const
  {
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0)
      return;

    displayNodePotentials(index_otl_->pull().current_img_);
  }

  void DepthNPG::_flush()
  {
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
    source_potentials_.setZero();
    sink_potentials_.setZero();
  }

  std::string DepthNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "DepthNPG_sigma:" << sigma_;
    return oss.str();
  }

} // namespace dst
