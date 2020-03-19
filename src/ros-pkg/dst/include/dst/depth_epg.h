#ifndef DEPTH_EPG_H
#define DEPTH_EPG_H

#include <opencv2/core/core.hpp>
#include <opencv_candidate/Camera.h>
#include <dst/edge_potential_generator.h>
#include <dst/surface_normal_node.h>
#include <dst/depth_projector.h>

namespace dst
{
  
  //! Outputs close to zero for known depth discontinuities & areas with missing data.
  //! Outputs close to one for known depth continuities.
  class DepthEPG : public EdgePotentialGenerator
  {
  public:
    typedef SurfaceNormalNode::Normals Normals;
    
    //! Outputs close to zero for known depth discontinuities
    //! and close to one otherwise (including the missing data case).
    pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* > weights_otl_;
    
    DepthEPG(pipeline2::Outlet<DepthProjector::Output>* index_otl,
	     pipeline2::Outlet<Normals::Ptr>* normals_otl,
	     int radius2d,
	     double sigma_norm,
	     double sigma_euc);
    ~DepthEPG();
    
  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    pipeline2::Outlet<Normals::Ptr>* normals_otl_;
    int radius2d_;
    double sigma_norm_;
    double sigma_euc_;
    Eigen::SparseMatrix<double, Eigen::RowMajor> weights_;

    double getWeight(int y0, int x0, int y, int x) const;
    double getPotential(int y0, int x0, int y, int x) const;
    void processDepthPoint(int y0, int x0);
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };

}

#endif // DEPTH_EPG_H
