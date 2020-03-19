#include <dst/node_potentials.h>

using namespace std;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{

  ColorHistogramNPG::ColorHistogramNPG(pipeline2::Outlet<cv::Mat3b>* image_otl,
				       pipeline2::Outlet<cv::Mat1b>* seed_otl,
				       pipeline2::Outlet<cv::Mat3b>* prev_image_otl,
				       pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
				       double smoothing,
				       int num_bins) :
    NodePotentialGenerator(),
    image_otl_(image_otl),
    seed_otl_(seed_otl),
    prev_image_otl_(prev_image_otl),
    prev_seg_otl_(prev_seg_otl),
    smoothing_(smoothing),
    num_bins_(num_bins),
    num_vals_per_bin_(0)
  {
    registerInput(image_otl_->getNode());
    registerInput(seed_otl_->getNode());
    registerInput(prev_image_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());

    // -- Set up storage for histogram.
    ROS_ASSERT(256 % num_bins_ == 0);
    num_vals_per_bin_ = 256 / num_bins_;

    // ROS_ASSERT(num_bins_ == 8);
    // ROS_ASSERT(num_vals_per_bin_ == 32);
    // ROS_ASSERT(getIdx(0) == 0);
    // ROS_ASSERT(getIdx(31) == 0);
    // ROS_ASSERT(getIdx(32) == 1);
    // ROS_ASSERT(getIdx(255) == 7);
  
    initializeHistogram(&source_hist_);
    initializeHistogram(&sink_hist_);
  }

  void ColorHistogramNPG::clearHistograms()
  {
    initializeHistogram(&source_hist_); // Shouldn't reallocate because the size doesn't change.
    initializeHistogram(&sink_hist_);
  }
  
  void ColorHistogramNPG::initializeHistogram(Hist* hist) const
  {
    Hist& hr = *hist;
  
    hr.resize(num_bins_);
    for(size_t i = 0; i < hr.size(); ++i) {
      hr[i].resize(num_bins_);
      for(size_t j = 0; j < hr[i].size(); ++j) { 
	hr[i][j].resize(num_bins_);
	for(size_t k = 0; k < hr[i][j].size(); ++k)
	  hr[i][j][k] = smoothing_;
      }
    }
  }

  void ColorHistogramNPG::_compute()
  {
    cv::Mat3b img = image_otl_->pull();
    cv::Mat1b seed = seed_otl_->pull();
    cv::Mat3b prev_img = prev_image_otl_->pull();
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    
    ROS_ASSERT(seed.rows == img.rows);
    ROS_ASSERT(seed.cols == img.cols);
    ROS_ASSERT(source_potentials_.rows() == sink_potentials_.rows());
    ROS_ASSERT(source_potentials_.cols() == sink_potentials_.cols());
  
    // -- Reallocate potentials storage only if necessary.
    if(img.rows != source_potentials_.rows() ||
       img.cols != source_potentials_.cols()) {
      source_potentials_ = MatrixXd::Zero(img.rows, img.cols);
      sink_potentials_ = MatrixXd::Zero(img.rows, img.cols);
    }

    // -- Learn the distribution.
    fillHistogram(seed, img, 0, &sink_hist_);
    fillHistogram(seed, img, 255, &source_hist_);
    // Use the previous segmentation & previous image if available.
    if(prev_seg.rows != 0) {
      ROS_ASSERT(prev_seg.rows == prev_img.rows);
      ROS_ASSERT(prev_seg.cols == prev_img.cols);
      fillHistogram(prev_seg, prev_img, 0, &sink_hist_);
      fillHistogram(prev_seg, prev_img, 255, &source_hist_);
    }
    
    // -- Set the potentials.
    //    This might be slow because Eigen is col-major.
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
	int b = img.at<cv::Vec3b>(y, x)[0];
	int g = img.at<cv::Vec3b>(y, x)[1];
	int r = img.at<cv::Vec3b>(y, x)[2];
	double num_src = source_hist_[getIdx(b)][getIdx(g)][getIdx(r)];
	double num_snk = sink_hist_[getIdx(b)][getIdx(g)][getIdx(r)];
	double p = num_src / (num_snk + num_src); // probability of being source
	// source_potentials_(y, x) = -log(1.0 - p); // high cost to cut this node if p is close to one.
	// sink_potentials_(y, x) = -log(p);
	source_potentials_(y, x) = p;
	sink_potentials_(y, x) = 1.0 - p;
      }
    }

    // -- Fill the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  int ColorHistogramNPG::getIdx(uchar val) const
  {
    return val / num_vals_per_bin_; // integer division, so this is essentially floor(val / num_vals_per_bin_).
  }

  void ColorHistogramNPG::fillHistogram(cv::Mat1b seed, cv::Mat3b img,
					int label, Hist* hist) const
  {
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
	int l = seed.at<uchar>(y, x);
	if(l == label) {
	  int b = img.at<cv::Vec3b>(y, x)[0];
	  int g = img.at<cv::Vec3b>(y, x)[1];
	  int r = img.at<cv::Vec3b>(y, x)[2];
	  ++(*hist)[getIdx(b)][getIdx(g)][getIdx(r)];
	}
      }
    }
  }

  void ColorHistogramNPG::_display() const
  {
    displayNodePotentials(image_otl_->pull());
  }

  void ColorHistogramNPG::_reset()
  {
    clearHistograms();
  }
  
  void ColorHistogramNPG::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  std::string ColorHistogramNPG::_getName() const
  {
    ostringstream oss;
    oss << "ColorHistogramNPG";
    return oss.str();
  }

} // namespace dst
