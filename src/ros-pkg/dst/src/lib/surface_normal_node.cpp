#include <dst/surface_normal_node.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;

namespace dst
{

  SurfaceNormalNode::SurfaceNormalNode(pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
				       pipeline2::Outlet<DepthProjector::Output>* index_otl,
				       double radius) :
    ComputeNode(),
    normals_otl_(this),
    kdtree_otl_(kdtree_otl),
    index_otl_(index_otl),
    radius_(radius),
    normals_(new Normals())
  {
    registerInput(kdtree_otl_->getNode());
    registerInput(index_otl_->getNode());
  }
  
  void SurfaceNormalNode::_compute()
  {
    KinectCloud::Ptr pcd = kdtree_otl_->pull().current_pcd_;

    // TODO: Maybe use the integral image method.
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setRectSize(5, 5);
    ne.setMaxDepthChangeFactor(0.02);
    ne.setNormalSmoothingSize(3.0);
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setDepthDependentSmoothing(false);
    ne.setInputCloud(pcd);
    ne.compute(*normals_);
    
    // normals_->resize(pcd->size());
    // for(size_t x = 0; x < pcd->width; ++x) {
    //   for(size_t y = 0; y < pcd->height; ++y) {
    // 	int idx = y * pcd->width + x;
    // 	ne.computePointNormal(x, y, normals_->at(idx));
    //   }
    // }

    // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    // ne.setInputCloud(pcd);
    // ne.setSearchMethod(kdtree_otl_->pull().current_kdtree_);
    // ne.setRadiusSearch(radius_);
    // ne.compute(*normals_);

    for(size_t i = 0; i < normals_->size(); i+=95) {
      cout << pcd->at(i) << endl;
      cout << normals_->at(i) << endl;
      cout << endl;
    }
    
    for(size_t i = 0; i < normals_->size(); ++i) {
      pcl::flipNormalTowardsViewpoint(pcd->at(i), 0, 0, 0,
				      normals_->at(i).normal[0],
				      normals_->at(i).normal[1],
				      normals_->at(i).normal[2]);
    }

    normals_otl_.push(normals_);
  }

  void SurfaceNormalNode::normalToColor(const pcl::Normal& p,
					cv::Vec3b* color) const
  {
    cv::Vec3b& c = *color;
    if(isnan(p.normal[0])) { 
      c = cv::Vec3b(0, 0, 0);
      return;
    }
    
    ROS_ASSERT(p.normal[0] <= 1.0 && p.normal[0] >= -1.0);
    c[0] = fabs(p.normal[0]) * 255;
    c[1] = fabs(p.normal[1]) * 255;
    c[2] = fabs(p.normal[2]) * 255;
  }
  
  void SurfaceNormalNode::_display() const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    cv::Mat3b vis(index.size(), cv::Vec3b(0, 0, 0));

    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
	if(index(y, x) == -1)
	  continue;

	normalToColor(normals_->at(index(y, x)), &vis(y, x));
      }
    }

    cv::imwrite("debug/" + getRunName() + ".png", vis);

    KinectCloud::Ptr pcd = kdtree_otl_->pull().current_pcd_;
    pcl::PointCloud<pcl::PointXYZRGBNormal> cn;
    pcl::concatenateFields(*pcd, *normals_, cn);
    pcl::io::savePCDFileBinary("debug/" + getRunName() + ".pcd", cn);
  }

  void SurfaceNormalNode::_flush()
  {
    normals_->clear();
    normals_otl_.push(Normals::Ptr());
  }

  std::string SurfaceNormalNode::_getName() const
  {
    std::ostringstream oss;
    oss << "SurfaceNormalNode";
    return oss.str();
  }
    
} // namespace dst
