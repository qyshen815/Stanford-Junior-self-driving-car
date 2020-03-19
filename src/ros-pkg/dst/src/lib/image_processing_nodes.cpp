#include <dst/image_processing_nodes.h>

using namespace std;

namespace dst
{

  IntensityImageNode::IntensityImageNode(pipeline2::Outlet<cv::Mat3b>* color_otl) :
    ComputeNode(),
    outlet_(this),
    color_otl_(color_otl)
  {
    registerInput(color_otl_->getNode());
  }

  void IntensityImageNode::_compute()
  {
    cv::Mat3b color = color_otl_->pull();
    cv::cvtColor(color, intensity_, CV_BGR2GRAY);
    ROS_ASSERT(intensity_.rows == color.rows);
    ROS_ASSERT(intensity_.cols == color.cols);
    outlet_.push(intensity_);
  }

  void IntensityImageNode::_display() const
  {
  }

  void IntensityImageNode::_flush()
  {
    outlet_.push(cv::Mat1b());
  }

  std::string IntensityImageNode::_getName() const
  {
    ostringstream oss;
    oss << "IntensityImageNode";
    return oss.str();
  }

  HSVImageNode::HSVImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl) :
    ComputeNode(),
    outlet_(this),
    rgb_otl_(rgb_otl)
  {
    registerInput(rgb_otl_->getNode());
  }
  
  void HSVImageNode::_compute()
  {
    cv::Mat3b rgb = rgb_otl_->pull();
    cv::cvtColor(rgb, hsv_, CV_BGR2HSV);
    ROS_ASSERT(hsv_.rows == rgb.rows);
    ROS_ASSERT(hsv_.cols == rgb.cols);
    outlet_.push(hsv_);
  }

  void HSVImageNode::_display() const
  {
  }

  void HSVImageNode::_flush()
  {
    outlet_.push(cv::Mat3f());
  }

  std::string HSVImageNode::_getName() const
  {
    ostringstream oss;
    oss << "HSVImageNode";
    return oss.str();
  }

  IntegralImageNode::IntegralImageNode(pipeline2::Outlet<cv::Mat1b>* intensity_otl) :
    ComputeNode(),
    outlet_(this),
    intensity_otl_(intensity_otl)
  {
    registerInput(intensity_otl_->getNode());
  }

  void IntegralImageNode::_compute()
  {
    cv::Mat1b intensity = intensity_otl_->pull();
    cv::integral(intensity, integral_);
    ROS_ASSERT(integral_.rows == intensity.rows + 1);
    ROS_ASSERT(integral_.cols == intensity.cols + 1);
    outlet_.push(integral_);
  }

  void IntegralImageNode::_display() const
  {

  }

  void IntegralImageNode::_flush()
  {
    outlet_.push(cv::Mat1i());
  }

  std::string IntegralImageNode::_getName() const
  {
    ostringstream oss;
    oss << "IntegralImageNode";
    return oss.str();
  }

  
} // namespace dst
