#include <dst/color_delta_epg.h>

using namespace std;

namespace dst
{

  ColorDeltaEPG::ColorDeltaEPG(pipeline2::Outlet<cv::Mat3b>* image_otl) :
    EdgePotentialGenerator(),
    image_otl_(image_otl)
  {
    registerInput(image_otl_->getNode());
  }
  
  double ColorDeltaEPG::computeCost(const cv::Vec3b& p, const cv::Vec3b& q) const
  {
    double norm = sqrt((p[0] - q[0])*(p[0] - q[0]) +
		       (p[1] - q[1])*(p[1] - q[1]) +
		       (p[2] - q[2])*(p[2] - q[2]));
    //cout << p << ", " << q << ", " << norm << " " << exp(-norm / 30.0) << endl;
    return exp(-norm / 30.0); // TODO: Parameterize this.
  }
  
  void ColorDeltaEPG::processPixel(int y, int x)
  {
    cv::Mat3b img = image_otl_->pull();
    int idx = getIdx(y, x, img.cols);
    potentials_.startVec(idx);
    
    if(y > 0) {
      int idx2 = getIdx(y-1, x, img.cols);
      potentials_.insertBack(idx, idx2) = computeCost(img(y, x), img(y-1, x));
    }
    if(x > 0) {
      int idx2 = getIdx(y, x-1, img.cols);
      potentials_.insertBack(idx, idx2) = computeCost(img(y, x), img(y, x-1));
    }
    if(x < img.cols - 1) {
      int idx2 = getIdx(y, x+1, img.cols);
      potentials_.insertBack(idx, idx2) = computeCost(img(y, x), img(y, x+1));
    }
    if(y < img.rows - 1) {
      int idx2 = getIdx(y+1, x, img.cols);
      potentials_.insertBack(idx, idx2) = computeCost(img(y, x), img(y+1, x));
    }
  }
  
  void ColorDeltaEPG::_compute()
  {
    cv::Mat3b img = image_otl_->pull();

    // TODO: Don't reallocate every time.
    int num_nodes = img.rows * img.cols;
    potentials_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(num_nodes, num_nodes);
    potentials_.reserve(num_nodes * 10);

    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
	processPixel(y, x);
      }
    }
    
    edge_otl_.push(&potentials_);
  }

  void ColorDeltaEPG::_display() const
  {
    displayEdges(image_otl_->pull());
  }

  void ColorDeltaEPG::_flush()
  {
    potentials_.setZero();
    edge_otl_.push(NULL);
  }

  std::string ColorDeltaEPG::_getName() const
  {
    std::ostringstream oss;
    oss << "ColorDeltaEPG";
    return oss.str();
  }

} // namespace dst
