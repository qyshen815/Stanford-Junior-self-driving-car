#ifndef IMAGE_PROCESSING_NODES_H
#define IMAGE_PROCESSING_NODES_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pipeline2/pipeline2.h>

namespace dst
{

  class IntensityImageNode : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet<cv::Mat1b> outlet_;
    IntensityImageNode(pipeline2::Outlet<cv::Mat3b>* color_otl);

  protected:
    pipeline2::Outlet<cv::Mat3b>* color_otl_;
    cv::Mat1b intensity_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

  class HSVImageNode : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet<cv::Mat3f> outlet_;
    HSVImageNode(pipeline2::Outlet<cv::Mat3b>* rgb_otl);

  protected:
    pipeline2::Outlet<cv::Mat3b>* rgb_otl_;
    cv::Mat3f hsv_;
    
  
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
  class IntegralImageNode : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet<cv::Mat1i> outlet_;
    
    IntegralImageNode(pipeline2::Outlet<cv::Mat1b>* intensity_otl);
    
  protected:
    pipeline2::Outlet<cv::Mat1b>* intensity_otl_;
    cv::Mat1i integral_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // IMAGE_PROCESSING_NODES_H
