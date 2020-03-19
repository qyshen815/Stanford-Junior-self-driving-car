#ifndef SURFACE_NORMAL_NODE_H
#define SURFACE_NORMAL_NODE_H

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <dst/kdtree_node.h>
#include <dst/depth_projector.h>

namespace dst
{

  class SurfaceNormalNode : public pipeline2::ComputeNode
  {
  public:
    typedef pcl::PointCloud<pcl::Normal> Normals;
    pipeline2::Outlet<Normals::Ptr> normals_otl_;
    
    SurfaceNormalNode(pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
		      pipeline2::Outlet<DepthProjector::Output>* index_otl,
		      double radius = 0.025);

  protected:
    pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double radius_;
    Normals::Ptr normals_;

    void normalToColor(const pcl::Normal& normal,
		       cv::Vec3b* color) const;
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // SURFACE_NORMAL_NODE_H
