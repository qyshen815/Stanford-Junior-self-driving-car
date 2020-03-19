#ifndef LABEL_FLOW_NPG_H
#define LABEL_FLOW_NPG_H

#include <dst/node_potential_generator.h>
#include <dst/optical_flow_node.h>

namespace dst
{

  class LabelFlowNPG : public NodePotentialGenerator
  {
  public:
    LabelFlowNPG(pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
		 pipeline2::Outlet<cv::Mat1b>* prev_seg_otl);

  protected:
    pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;

    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // LABEL_FLOW_NPG_H
