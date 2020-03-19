#ifndef SEQUENCE_SEGMENTATION_SOLVER_H
#define SEQUENCE_SEGMENTATION_SOLVER_H

#include <bag_of_tricks/high_res_timer.h>
#include <maxflow/graph.h>
#include <dst/sequence_segmentation_model.h>

namespace dst
{
  
  class SequenceSegmentationSolver
  {
  public:
    typedef boost::shared_ptr<SequenceSegmentationSolver> Ptr;
    typedef boost::shared_ptr<const SequenceSegmentationSolver> ConstPtr;
  
    SequenceSegmentationModel::ConstPtr model_;
  
    SequenceSegmentationSolver();
  };

} // namespace dst

#endif // SEQUENCE_SEGMENTATION_SOLVER_H
