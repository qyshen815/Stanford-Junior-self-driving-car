#ifndef SURFACE_NORMAL_EPG_H
#define SURFACE_NORMAL_EPG_H

#include <pcl/features/normal_3d.h>
#include <dst/edge_potential_generator.h>
#include <dst/depth_projector.h>

namespace dst
{
  class SurfaceNormalEPG : public EdgePotentialGenerator
  {
  public:
    typedef pcl::PointCloud<pcl::Normal> Normals;
    SurfaceNormalEPG(pipeline2::Outlet<Normals::Ptr>* normals_otl,
		     pipeline2::Outlet<DepthProjector::Output>* index_otl,
		     pipeline2::Outlet<cv::Mat3b>* image_otl);

  protected:
    pipeline2::Outlet<Normals::Ptr>* normals_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    pipeline2::Outlet<cv::Mat3b>* image_otl_;

    double computePotential(int idx0, int idx1, Normals::Ptr normals) const;
    void fillPotentials(int y, int x, int idx);
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
}

#endif // SURFACE_NORMAL_EPG_H
