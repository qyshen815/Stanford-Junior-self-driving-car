#ifndef ORGANIZED_SURFACE_NORMAL_NODE_H
#define ORGANIZED_SURFACE_NORMAL_NODE_H

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <dst/depth_projector.h>
#include <dst/image_region_iterator.h>

namespace dst
{

  class OrganizedSurfaceNormalNode : public pipeline2::ComputeNode
  {
  public:
    typedef pcl::PointCloud<pcl::Normal> Normals;
    pipeline2::Outlet<Normals::Ptr> normals_otl_;
    
    OrganizedSurfaceNormalNode(pipeline2::Outlet<DepthProjector::Output>* index_otl,
			       int radius);

  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double radius_;
    double z_thresh_;
    Normals::Ptr normals_;
    std::vector<int> indices_;
    std::vector<int> inliers_;
    std::vector<bool> valid_;
    std::vector<float> weights_;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator_;

    void computeNormal(const KinectCloud& pcd,
		       const pcl::PointXYZRGB& center,
		       const std::vector<int>& indices,
		       pcl::Normal* normal);
    void computeNormal(const KinectCloud& pcd,
		       const pcl::PointXYZRGB& pt,
		       const cv::Point2i& img_pt,
		       pcl::Normal* normal);
    void normalToColor(const pcl::Normal& normal,
		       cv::Vec3b* color) const;
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
    
}

#endif // ORGANIZED_SURFACE_NORMAL_NODE_H
