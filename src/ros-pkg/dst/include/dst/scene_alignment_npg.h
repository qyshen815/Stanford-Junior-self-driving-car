#ifndef SCENE_ALIGNMENT_NPG_H
#define SCENE_ALIGNMENT_NPG_H

#include <dst/scene_alignment_node.h>
#include <dst/node_potential_generator.h>
#include <dst/kdtree_node.h>

namespace dst
{

  class SceneAlignmentNPG : public NodePotentialGenerator
  {
  public:
    //! @param radius The maximum radius to search for neighbors of a point.
    SceneAlignmentNPG(pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl,
		      pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
		      pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
		      pipeline2::Outlet<DepthProjector::Output>* index_otl,
		      int num_neighbors = 1,
		      double sigma = 0.0217);

  protected:
    pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl_;
    pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double radius_;
    int num_neighbors_;
    double sigma_;
    
    //! Aligned with previous cloud.
    //! 0 = BG, 255 = FG, 127 = no data.
    std::vector<int> labels_;
    std::vector<cv::Point2i> imgpts_;
    
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
  
}

#endif // SCENE_ALIGNMENT_NPG_H
