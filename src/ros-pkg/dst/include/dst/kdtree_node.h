#ifndef KDTREE_NODE_H
#define KDTREE_NODE_H

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>

namespace dst
{

  class KdTreeNode : public pipeline2::ComputeNode
  {
  public:
    typedef pcl::KdTreeFLANN<pcl::PointXYZRGB> KdTreeType;
    
    typedef struct
    {
      KdTreeType::Ptr current_kdtree_;
      KdTreeType::Ptr previous_kdtree_;
      KinectCloud::Ptr current_pcd_;
      KinectCloud::Ptr previous_pcd_;
    } Output;

    pipeline2::Outlet<Output> kdtree_otl_;

    KdTreeNode(pipeline2::Outlet<KinectCloud::Ptr>* pcd_otl,
	       double radius);
    ~KdTreeNode();

  protected:
    pipeline2::Outlet<KinectCloud::Ptr>* pcd_otl_;
    double radius_;
    KdTreeType::Ptr current_kdtree_;
    KdTreeType::Ptr previous_kdtree_;
    KinectCloud::Ptr current_pcd_;
    KinectCloud::Ptr previous_pcd_;

    void swap();
    void _compute();
    void _display() const;
    void _flush();
    //! Clears the cached pointclouds and trees.
    void _reset();
    std::string _getName() const;
  };
  
}

#endif // KDTREE_NODE_H
