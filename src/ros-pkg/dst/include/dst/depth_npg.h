#ifndef DEPTH_NPG_H
#define DEPTH_NPG_H

#include <deque>
#include <dst/depth_projector.h>
#include <dst/node_potential_generator.h>

namespace dst
{

  class DepthNPG : public NodePotentialGenerator
  {
  public:
    DepthNPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
	     pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
	     float sigma,
	     size_t buffer_size);

  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    float sigma_;
    size_t buffer_size_;
    std::deque<float> max_buffer_;
    std::deque<float> min_buffer_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // DEPTH_NPG_H
