#include <dst/edge_potentials.h>

using namespace std;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{

  CannyEPG::CannyEPG(pipeline2::Outlet<cv::Mat3b>* image_otl,
		     double threshold1,
		     double threshold2) :
    EdgePotentialGenerator(),
    image_otl_(image_otl),
    threshold1_(threshold1),
    threshold2_(threshold2)
  {
    registerInput(image_otl_->getNode());
  }

  void CannyEPG::fillPotentials(int y, int x, int idx)
  {
    // -- Canny edge pixels connect up and left always.
    if(canny_(y, x) != 0) {
      if(y > 0) { 
	int idx2 = getIdx(y-1, x, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
      if(x > 0) { 
	int idx2 = getIdx(y, x-1, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
    }
    // -- Non-canny edge pixels connect down and right always,
    //    up and left only if neighbor is not a canny edge pixel.
    else { 
      if(y > 0 && canny_(y-1, x) == 0) { 
	int idx2 = getIdx(y-1, x, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
      if(x > 0 && canny_(y, x-1) == 0) { 
	int idx2 = getIdx(y, x-1, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
      if(x < canny_.cols - 1) {
	int idx2 = getIdx(y, x+1, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
      if(y < canny_.rows - 1) {
	int idx2 = getIdx(y+1, x, canny_.cols);
	potentials_.insertBack(idx, idx2) = 1.0;
      }
    }
  }
  
  void CannyEPG::_compute()
  {
    // -- Compute the edge image.
    cv::Mat3b img = image_otl_->pull();
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    cv::Canny(gray, canny_, threshold1_, threshold2_);

    // -- Fill the edge potentials.
    int num_nodes = img.rows * img.cols;
    //if(potentials_.rows() != num_nodes || potentials_.cols() != num_nodes)
    potentials_ = SparseMatrix<double, RowMajor>(num_nodes, num_nodes); // TODO: Don't reallocate every time.

    for(int y = 0; y < canny_.rows; ++y) {
      for(int x = 0; x < canny_.cols; ++x) {
	int idx = getIdx(y, x, canny_.cols);
	potentials_.startVec(idx);
	
	fillPotentials(y, x, idx);
      }
    }
    potentials_.finalize();
    
    // -- Load the outlet.
    edge_otl_.push(&potentials_);
  }
  
  void CannyEPG::_display() const
  {
    cv::imwrite("debug/" + getRunName() + "-edge_image.png", canny_);
    displayEdges(image_otl_->pull());
  }
  
  void CannyEPG::_flush()
  {
    canny_ = 127; // reset the image to all 127s.
    potentials_.setZero();
    edge_otl_.push(NULL);
  }
  
  std::string CannyEPG::_getName() const
  {
    ostringstream oss;
    oss << "CannyEPG_threshold1:" << threshold1_ << "_threshold2:" << threshold2_;
    return oss.str();
  }
    
} // namespace dst
