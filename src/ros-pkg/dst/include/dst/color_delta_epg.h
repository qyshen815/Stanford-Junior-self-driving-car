#ifndef COLOR_DELTA_EPG_H
#define COLOR_DELTA_EPG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dst/edge_potential_generator.h>
#include <dst/helper_functions.h>

namespace dst
{

  class ColorDeltaEPG : public EdgePotentialGenerator
  {
  public:
    ColorDeltaEPG(pipeline2::Outlet<cv::Mat3b>* image_otl);
    
  protected:
    pipeline2::Outlet<cv::Mat3b>* image_otl_;
    
    double computeCost(const cv::Vec3b& p, const cv::Vec3b& q) const;
    void processPixel(int y, int x);
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // COLOR_DELTA_EPG_H
