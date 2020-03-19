#ifndef SEED_NPG_H
#define SEED_NPG_H

#include <opencv2/core/core.hpp>
#include <dst/node_potential_generator.h>

namespace dst
{

  class SeedNPG : public NodePotentialGenerator
  {
  public:
    SeedNPG(pipeline2::Outlet<cv::Mat1b>* seed_otl);
    
  protected:
    pipeline2::Outlet<cv::Mat1b>* seed_otl_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // SEED_NPG_H
