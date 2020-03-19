#include <dst/node_potential_generator.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  NodePotentialGenerator::NodePotentialGenerator() :
    source_otl_(this),
    sink_otl_(this)
  {
  }

  double sigmoid(double z)
  {
    return 1.0 / (1.0 + exp(-z));
  }
  
  void NodePotentialGenerator::displayNodePotentials(const cv::Mat3b background) const
  {
    // -- Just the potentials.
    cv::Mat3b raw;
    raw = cv::Mat3b(source_potentials_.rows(),
		    source_potentials_.cols(),
		    cv::Vec3b(0, 0, 0));
    
    for(int y = 0; y < raw.rows; ++y) { 
      for(int x = 0; x < raw.cols; ++x) {
	// -1 for bg, +1 for fg.
	double val = 2.0 * sigmoid(5.0 * (source_potentials_(y, x) - sink_potentials_(y, x))) - 1.0;
	val = min(0.9, max(-0.9, val));
	if(val < 0)
	  raw(y, x)[1] = 255 * -val;
	else
	  raw(y, x)[2] = 255 * val;	  
      }
    }

    double scale = 3;
    cv::Mat3b scaled_raw;
    cv::Size sz;
    sz.width = raw.cols * scale;
    sz.height = raw.rows * scale;
    cv::resize(raw, scaled_raw, sz);
    cv::imwrite("debug/" + getRunName() + "-raw.png", scaled_raw);

    // -- Overlay.
    cv::Mat3b vis;
    if(background.rows == 0)
      return;

    vis = background.clone();
    ROS_ASSERT(vis.rows > 0 && vis.cols > 0);
    for(int y = 0; y < vis.rows; ++y) { 
      for(int x = 0; x < vis.cols; ++x) {
	double val = sigmoid(5.0 * (source_potentials_(y, x) - sink_potentials_(y, x))); // 1.0 for foreground.
	val = min(0.9, max(0.1, val));
	vis(y, x)[0] = vis(y, x)[0] * val;
	vis(y, x)[1] = vis(y, x)[1] * val;
	vis(y, x)[2] = vis(y, x)[2] * val;
      }
    }

    cv::Mat3b scaled;
    sz.width = vis.cols * scale;
    sz.height = vis.rows * scale;
    cv::resize(vis, scaled, sz);

    cv::imwrite("debug/" + getRunName() + "-overlay.png", scaled);
  }
  
} // namespace dst
