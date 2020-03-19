#include <dst/edge_potential_generator.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  EdgePotentialGenerator::EdgePotentialGenerator() :
    edge_otl_(this)
  {
  }

  void drawLine(cv::Point pt, const cv::Point& pt1, double potential, cv::Mat3b vis)
  {
    if(pt1.x < 0 || pt1.x >= vis.cols)
      return;
    if(pt1.y < 0 || pt1.y >= vis.rows)
      return;

    int dx = sign(pt1.x - pt.x);
    int dy = sign(pt1.y - pt.y);
    for(; (pt.x != pt1.x) || (pt.y != pt1.y); pt.x += dx, pt.y += dy) {
      vis(pt)[0] = vis(pt)[0] * (1.0 - potential);
      vis(pt)[1] = vis(pt)[1] * (1.0 - potential);
      vis(pt)[2] = vis(pt)[2] * (1.0 - potential);
    }
  }
  
  void EdgePotentialGenerator::displayEdges(cv::Mat3b img) const
  {
    double scale = 7;
    cv::Size sz(img.cols * scale, img.rows * scale);
    cv::Mat3b vis;
    cv::resize(img, vis, sz, cv::INTER_NEAREST);

    SparseMatrix<double, RowMajor> normalized = potentials_;
    double max = -std::numeric_limits<double>::max();
    double min =  std::numeric_limits<double>::max();
    for(int i = 0; i < normalized.rows(); ++i) {
      SparseMatrix<double, RowMajor>::InnerIterator it(normalized, i);
      for(; it; ++it) {
	if(it.value() > max)
	  max = it.value();
	if(it.value() < min)
	  min = it.value();
      }
    }
    normalized /= max;
    cout << _getName() << ": range of edge weights is " << min << " to " << max << endl;

    // -- Draw non-zero edges.
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
	int idx0 = getIdx(y, x, img.cols);

	SparseMatrix<double, RowMajor>::InnerIterator it(normalized, idx0);
	for(; it; ++it) {
	  int idx1 = it.col();
	  int y1 = idx1 / img.cols;
	  int x1 = idx1 - y1 * img.cols;

	  cv::Point pt(x*scale, y*scale);
	  cv::Point pt1(x1*scale, y1*scale);
	  drawLine(pt, pt1, it.value(), vis);
	}
      }
    }

    cv::imwrite("debug/" + getRunName() + ".png", vis);
  }
  
}
