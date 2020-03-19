#ifndef CANNY_EPG_H
#define CANNY_EPG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dst/edge_potential_generator.h>

namespace dst
{

  class CannyEPG : public EdgePotentialGenerator
  {
  public:
    CannyEPG(pipeline2::Outlet<cv::Mat3b>* image_otl,
	     double threshold1,
	     double threshold2);

  protected:
    //! TODO: Make this cv::Mat1b, add a node to do BGR->gray if necessary.
    pipeline2::Outlet<cv::Mat3b>* image_otl_;
    cv::Mat1b canny_;
    double threshold1_;
    double threshold2_;

    void fillPotentials(int y, int x, int idx);
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // CANNY_EPG_H
